import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import numpy as np

from align_pico_manus import choose_alignment_clock
from export_dataset import (
    DEFAULT_MATCH_GATE_MS,
    DEFAULT_MAX_P95_SKEW_MS,
    DEFAULT_MAX_SKEW_MS,
    DEFAULT_MIN_COMPLETE_COVERAGE,
    DEFAULT_MIN_COVERAGE,
    _check_quality,
    complete_frame_mask,
    common_valid_interval,
    controller_rows_to_wrist,
    controller_to_wrist_metadata,
    load_video_timestamps,
    match_manus_frames,
    match_video_details,
    probe_video_size,
    resegment_complete_rows,
)
from pico_receiver import VideoDiskWriter


class AlignmentClockTests(unittest.TestCase):
    def test_complete_qpc_is_selected_for_whole_session(self):
        pico = [{"wall_ns": 100, "qpc_ns": 10}]
        manus = {
            "left": [{"wall_ns": 101, "qpc_ns": 11}],
            "right": [{"wall_ns": 102, "qpc_ns": 12}],
        }
        tactile = {
            "left": [{"wall_ns": 103, "qpc_ns": 13}],
            "right": [{"wall_ns": 104, "qpc_ns": 14}],
        }
        self.assertEqual(choose_alignment_clock(pico, manus, tactile), "qpc_ns")

    def test_one_missing_qpc_forces_whole_session_wall_fallback(self):
        pico = [{"wall_ns": 100, "qpc_ns": 10}]
        manus = {
            "left": [{"wall_ns": 101, "qpc_ns": 11}],
            "right": [{"wall_ns": 102, "qpc_ns": None}],
        }
        self.assertEqual(choose_alignment_clock(pico, manus), "wall_ns")

    def test_common_interval_is_intersection_of_all_required_streams(self):
        pico = [{"wall_ns": 0}, {"wall_ns": 100}]
        manus = {
            "left": [{"wall_ns": 10}, {"wall_ns": 90}],
            "right": [{"wall_ns": 20}, {"wall_ns": 80}],
        }
        tactile = {
            "left": [{"wall_ns": 15}, {"wall_ns": 75}],
            "right": [{"wall_ns": 25}, {"wall_ns": 85}],
        }
        video = {"wall_ns": np.asarray([30, 70]), "qpc_ns": np.empty(0)}
        start, end, _ = common_valid_interval(
            pico, manus, tactile, video, "wall_ns", require_video=True,
        )
        self.assertEqual((start, end), (30, 70))


class ProvenanceTests(unittest.TestCase):
    def test_strict_alignment_policy_defaults(self):
        self.assertEqual(DEFAULT_MATCH_GATE_MS, 30.0)
        self.assertEqual(DEFAULT_MAX_SKEW_MS, 30.0)
        self.assertEqual(DEFAULT_MAX_P95_SKEW_MS, 20.0)
        self.assertEqual(DEFAULT_MIN_COVERAGE, 0.99)
        self.assertEqual(DEFAULT_MIN_COMPLETE_COVERAGE, 0.99)

    def test_video_size_is_read_from_the_coded_stream(self):
        with tempfile.TemporaryDirectory() as temp:
            video = Path(temp) / "vst.h264"
            video.write_bytes(b"not decoded in this unit test")
            result = SimpleNamespace(
                stdout='{"streams":[{"width":4096,"height":1536}]}',
            )
            with mock.patch("export_dataset.shutil.which", return_value="ffprobe"), \
                    mock.patch("export_dataset.subprocess.run", return_value=result):
                self.assertEqual((4096, 1536), probe_video_size(video))

    def test_complete_frame_mask_requires_every_requested_route(self):
        hand_valid = {
            "left": np.asarray([True, True, True, True]),
            "right": np.asarray([True, True, True, True]),
        }
        video_valid = np.asarray([True, False, True, True])
        tactile = {
            "left": {"valid": np.asarray([True, True, False, True])},
            "right": {"valid": np.asarray([True, True, True, False])},
        }
        mask = complete_frame_mask(
            hand_valid, video_valid, tactile, require_video=True,
        )
        np.testing.assert_array_equal(mask, [True, False, False, False])

    def test_resegment_splits_every_removed_row_gap(self):
        base = np.asarray([0, 0, 0, 1, 1, 1])
        kept = np.asarray([0, 2, 3, 4, 5])
        out = resegment_complete_rows(base, kept)
        np.testing.assert_array_equal(out, [0, 1, 2, 2, 2])

    def test_controller_to_wrist_calibration_is_embedded_numerically(self):
        calib = {
            "left": ([0.1, 0.2, 0.3], [0.0, 0.0, 0.0, 1.0]),
            "right": None,
        }
        metadata = controller_to_wrist_metadata(calib)
        self.assertEqual(metadata["schema"], "controller_to_wrist_v1")
        self.assertEqual(metadata["left"]["pos"], [0.1, 0.2, 0.3])
        self.assertEqual(metadata["right"]["pos"], [0.0, 0.0, 0.0])

        controller = np.asarray([[1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0]])
        wrist = controller_rows_to_wrist(
            controller, (metadata["left"]["pos"], metadata["left"]["quat"]),
        )
        np.testing.assert_allclose(
            wrist[0], [1.1, 2.2, 3.3, 0.0, 0.0, 0.0, 1.0], atol=1e-12,
        )

    def test_video_details_keep_signed_offset_and_both_source_clocks(self):
        video = {
            "wall_ns": np.asarray([1000, 2000], dtype=np.int64),
            "qpc_ns": np.asarray([100, 200], dtype=np.int64),
        }
        out = match_video_details(
            np.asarray([190], dtype=np.int64), video,
            clock_key="qpc_ns", gate_ms=1.0,
        )
        self.assertTrue(out["valid"][0])
        self.assertEqual(out["frame_idx"][0], 1)
        self.assertEqual(out["recv_wall_ns"][0], 2000)
        self.assertEqual(out["recv_qpc_ns"][0], 200)
        self.assertAlmostEqual(float(out["offset_ms"][0]), 0.00001, places=8)

    def test_manus_details_use_requested_clock(self):
        manus = {
            "left": [{"wall_ns": 9000, "qpc_ns": 100}],
            "right": [{"wall_ns": 1000, "qpc_ns": 110}],
        }
        out = match_manus_frames(
            np.asarray([105], dtype=np.int64), manus,
            clock_key="qpc_ns", gate_ms=1.0,
        )
        self.assertTrue(out["left"]["valid"][0])
        self.assertTrue(out["right"]["valid"][0])

    def test_strict_quality_gate_rejects_coverage_and_skew(self):
        errors = []
        _check_quality(
            "VST",
            {"coverage": 0.98, "p95_ms": 21.0, "max_ms": 31.0},
            min_coverage=0.99,
            max_p95_ms=20.0,
            max_offset_ms=30.0,
            errors=errors,
        )
        self.assertEqual(len(errors), 3)


class VideoWriterTests(unittest.TestCase):
    def test_async_writer_emits_paired_sidecars_for_picture_only(self):
        with tempfile.TemporaryDirectory() as directory:
            video = Path(directory) / "vst.h264"
            writer = VideoDiskWriter(str(video))
            writer.submit(b"config", False, 1000, 100)
            writer.submit(b"picture", True, 2000, 200)
            writer.close()
            self.assertEqual(video.read_bytes(), b"configpicture")
            self.assertEqual(
                (Path(directory) / "vst.ts.jsonl").read_text().strip(), "2000"
            )
            self.assertEqual(
                (Path(directory) / "vst.qpc.ts.jsonl").read_text().strip(), "200"
            )

    def test_video_timestamp_loader_rejects_unpaired_counts(self):
        with tempfile.TemporaryDirectory() as directory:
            wall = Path(directory) / "vst.ts.jsonl"
            qpc = Path(directory) / "vst.qpc.ts.jsonl"
            wall.write_text("1\n2\n", encoding="utf-8")
            qpc.write_text("10\n", encoding="utf-8")
            with self.assertRaises(ValueError):
                load_video_timestamps(wall, qpc)


if __name__ == "__main__":
    unittest.main()
