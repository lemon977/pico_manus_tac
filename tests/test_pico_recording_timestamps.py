import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from pico_recording_timestamps import (
    PicoRecordingError,
    build_wall_timestamps,
    convert,
)


class PicoRecordingTimestampTests(unittest.TestCase):
    def test_pts_zero_is_anchored_to_pico_wall_clock(self):
        meta = {"recording_sync": {
            "video_start_wall_ns": 1_000_000_000,
            "video_pose_timebase": "relative_pts_sec",
        }}
        from fractions import Fraction
        self.assertEqual(
            [1_000_000_000, 1_033_333_333, 1_066_666_667],
            build_wall_timestamps(meta, Fraction(1, 90_000), [0, 3000, 6000]),
        )

    def test_convert_validates_count_and_writes_one_wall_per_frame(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            video = root / "CameraRecord_test.mp4"
            video.write_bytes(b"video")
            (root / "meta.json").write_text(json.dumps({
                "video": {"file_name": video.name},
                "recording_sync": {
                    "video_start_wall_ns": 2_000_000_000,
                    "video_pose_timebase": "relative_pts_sec",
                },
                "recording_diagnostics": {
                    "video_probe_finalized": {"sample_count": 2},
                },
            }), encoding="utf-8")
            probe = mock.Mock(
                returncode=0,
                stdout=json.dumps({
                    "frames": [
                        {"best_effort_timestamp": 0},
                        {"best_effort_timestamp": 3000},
                    ],
                    "streams": [{"time_base": "1/90000"}],
                }),
            )
            with mock.patch("subprocess.run", return_value=probe):
                result = convert(root, ffprobe="ffprobe")
            self.assertEqual(2, result["frames"])
            self.assertEqual(
                ["2000000000", "2033333333"],
                Path(result["sidecar"]).read_text().splitlines(),
            )

    def test_rejects_non_monotonic_pts(self):
        from fractions import Fraction
        meta = {"recording_sync": {"video_start_wall_ns": 1}}
        with self.assertRaises(PicoRecordingError):
            build_wall_timestamps(meta, Fraction(1, 1000), [2, 1])


if __name__ == "__main__":
    unittest.main()
