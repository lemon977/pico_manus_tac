import json
import os
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace
from unittest import mock

import h5py

from batch_data_manager import (
    ensure_collection_not_running,
    execute_delete,
    list_batch_indices,
    plan_delete,
)
from align_pico_manus import _tactile_directory_session_names
from session_layout import (
    flat_session_paths,
    grouped_session_paths,
    new_session_paths,
    resolve_session,
)
from pico_receiver import Recorder
from manus_collector import Collector as ManusCollector
from tactile_collector import TactileCollector


class BatchLayoutTests(unittest.TestCase):
    def test_manus_service_startup_does_not_create_phantom_session(self):
        with tempfile.TemporaryDirectory() as temp:
            sessions = Path(temp) / "sessions"
            collector = ManusCollector(SimpleNamespace(
                log_dir=str(sessions), hdf5=False, service=True,
                no_wait=False, duration=0.0, hands="both",
            ))
            self.assertIsNone(collector.session)
            self.assertEqual([], list(sessions.iterdir()))

    def test_numbered_session_uses_prefix_directories(self):
        paths = new_session_paths("data/sessions", "expert_012", data_root="data")
        self.assertEqual(Path("data/sessions/expert/012/raw"), paths["dir"])
        self.assertEqual(paths["dir"], paths["tactile_dir"])
        self.assertEqual(Path("data/sessions/expert/012/aligned.jsonl"), paths["aligned"])
        self.assertEqual(Path("data/sessions/expert/012/dataset.hdf5"), paths["export"])
        self.assertEqual(Path("data/sessions/expert/012/manifest.json"), paths["manifest"])

    def test_timestamp_session_is_not_misread_as_batch_index(self):
        paths = new_session_paths("data/sessions", "20260827_105821", data_root="data")
        self.assertEqual(Path("data/sessions/20260827_105821/raw"), paths["dir"])

    def test_tactile_meta_session_accepts_grouped_directory(self):
        path = Path("data/tactile_raw/3/001/tactile.jsonl")
        self.assertIn("3_001", _tactile_directory_session_names(path))
        flat = Path("data/tactile_raw/3_001/tactile.jsonl")
        self.assertEqual({"3_001"}, _tactile_directory_session_names(flat))
        task_first = Path("data/sessions/3/001/raw/tactile.jsonl")
        self.assertIn("3_001", _tactile_directory_session_names(task_first))

    def test_resolve_keeps_legacy_flat_batch_readable(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            raw = root / "data/raw/expert_001"
            raw.mkdir(parents=True)
            (raw / "pico.jsonl").write_text("{}\n", encoding="utf-8")
            paths = resolve_session("expert_001", data_root=root / "data",
                                    logs_dir=root / "logs")
            self.assertIsNotNone(paths)
            self.assertTrue(paths["legacy_flat"])
            self.assertEqual(raw, paths["dir"])

    def test_resolve_keeps_legacy_grouped_batch_readable(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            expected = grouped_session_paths(
                root / "data/raw", "expert_001", data_root=root / "data")
            expected["dir"].mkdir(parents=True)
            expected["pico"].write_text("{}\n", encoding="utf-8")
            paths = resolve_session("expert_001", data_root=root / "data",
                                    logs_dir=root / "logs")
            self.assertIsNotNone(paths)
            self.assertTrue(paths["legacy_grouped"])
            self.assertEqual(expected["dir"], paths["dir"])

    def test_pico_and_tactile_writers_use_same_batch_path(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            recorder = Recorder(root / "sessions")
            try:
                reply = recorder.start("expert_004")
                self.assertIn(str(root / "sessions/expert/004/raw/pico.jsonl"), reply)
                recorder.write(
                    {"functionName": "Tracking", "data": {}},
                    recv_wall_ns=111,
                    recv_qpc_ns=222,
                )
            finally:
                recorder.stop()
            record = json.loads(
                (root / "sessions/expert/004/raw/pico.jsonl").read_text(encoding="utf-8")
            )
            self.assertEqual(record["recv_wall_ns"], 111)
            self.assertEqual(record["recv_qpc_ns"], 222)

            tactile = TactileCollector.__new__(TactileCollector)
            tactile.log_dir = root / "sessions"
            tactile.log_dir.mkdir(exist_ok=True)
            session_dir, _, _, _ = tactile._safe_paths("expert_004")
            self.assertEqual((root / "sessions/expert/004/raw").resolve(), session_dir)

    def test_pico_no_vst_session_never_opens_a_video_file(self):
        class FakeVideo:
            writer_errors = 0
            max_write_backlog = 0

            def start_recording(self, _path):
                raise AssertionError("no-VST session opened the video writer")

            def stop_recording(self):
                return 0, 0

        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            recorder = Recorder(
                root / "sessions", video_rx=FakeVideo(),
                video_inner={"width": 4096, "height": 1536, "fps": 30},
                video_autosave=True,
            )
            reply = recorder.start("expert_005", record_video=False)
            self.assertTrue(reply.startswith("OK recording"))
            self.assertTrue(recorder.stop().startswith("OK stopped"))
            raw = root / "sessions/expert/005/raw"
            self.assertTrue((raw / "pico.jsonl").is_file())
            self.assertFalse((raw / "vst.h264").exists())


class BatchDeleteTests(unittest.TestCase):
    def _make_session(self, root: Path, prefix: str, index: int) -> None:
        session = f"{prefix}_{index:03d}"
        paths = new_session_paths(root / "data/sessions", session,
                                  data_root=root / "data")
        paths["dir"].mkdir(parents=True)
        (paths["dir"] / "pico.jsonl").write_text(
            json.dumps({"marker": index}) + "\n", encoding="utf-8")
        (paths["dir"] / "manus.jsonl").write_text("{}\n", encoding="utf-8")
        paths["tactile_dir"].mkdir(parents=True, exist_ok=True)
        paths["tactile"].write_text("{}\n", encoding="utf-8")
        paths["tactile_meta"].write_text(
            json.dumps({"session": session}), encoding="utf-8")
        paths["aligned"].parent.mkdir(parents=True, exist_ok=True)
        paths["aligned"].write_text("{}\n", encoding="utf-8")
        paths["export"].parent.mkdir(parents=True, exist_ok=True)
        with h5py.File(paths["export"], "w") as handle:
            handle.attrs["source_pico"] = str(paths["pico"].resolve())
        paths["manifest"].write_text(json.dumps({
            "session": session,
            "files": {"pico": {"path": str(paths["pico"])}}
        }), encoding="utf-8")

    def test_delete_archives_target_and_preserves_survivor_indices(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            for index in (1, 2, 3):
                self._make_session(root, "expert", index)

            plan = plan_delete(root, "expert", [2])
            self.assertEqual({}, plan["renumber"])
            archive = execute_delete(root, plan)

            self.assertEqual([1, 3], list_batch_indices(root, "expert"))
            third = new_session_paths(root / "data/sessions", "expert_003",
                                       data_root=root / "data")
            marker = json.loads(third["pico"].read_text(encoding="utf-8"))
            self.assertEqual(3, marker["marker"])
            meta = json.loads(third["tactile_meta"].read_text(encoding="utf-8"))
            self.assertEqual("expert_003", meta["session"])
            with h5py.File(third["export"], "r") as handle:
                self.assertIn("sessions\\expert\\003\\raw", handle.attrs["source_pico"])
            self.assertTrue((archive / "002/session/raw/pico.jsonl").is_file())
            self.assertTrue((archive / "batch_edit.json").is_file())
            second = new_session_paths(root / "data/sessions", "expert_002",
                                       data_root=root / "data")
            self.assertFalse(second["dir"].exists())
            audit = json.loads((archive / "batch_edit.json").read_text(encoding="utf-8"))
            self.assertEqual(audit["numbering_policy"], "preserve_existing_indices")
            self.assertEqual(audit["renumber"], {})

    def test_legacy_flat_survivor_keeps_original_number_and_layout(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            for index in (1, 2):
                session = f"oldbatch_{index:03d}"
                paths = flat_session_paths(root / "data/raw", session,
                                           data_root=root / "data")
                paths["dir"].mkdir(parents=True)
                paths["pico"].write_text(
                    json.dumps({"marker": index}) + "\n", encoding="utf-8")
                paths["manus"].write_text("{}\n", encoding="utf-8")
            execute_delete(root, plan_delete(root, "oldbatch", [1]))

            survivor = flat_session_paths(root / "data/raw", "oldbatch_002",
                                          data_root=root / "data")
            self.assertTrue(survivor["pico"].is_file())
            marker = json.loads(survivor["pico"].read_text(encoding="utf-8"))
            self.assertEqual(2, marker["marker"])
            self.assertEqual([2], list_batch_indices(root, "oldbatch"))

    def test_delete_refuses_while_collection_window_is_running(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            lock = root / ".run/collect_windows.pid"
            lock.parent.mkdir()
            lock.write_text(str(os.getpid()), encoding="ascii")
            with self.assertRaisesRegex(RuntimeError, "采集程序仍在运行"):
                ensure_collection_not_running(root)

    def test_delete_fails_closed_when_collection_lock_is_exclusive(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            lock = root / ".run/collect_windows.pid"
            lock.parent.mkdir()
            lock.write_text("123", encoding="ascii")
            with mock.patch.object(Path, "read_text", side_effect=PermissionError):
                with self.assertRaisesRegex(RuntimeError, "独占锁"):
                    ensure_collection_not_running(root)


if __name__ == "__main__":
    unittest.main()
