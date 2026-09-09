import contextlib
import io
import json
import queue
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest import mock

import pico_record
from record_control import wait_queue_drained


class FakeWindowsConsole:
    def __init__(self, key):
        self.key = key
        # 第一次供录制开始前清空缓冲区；第二次供正式轮询。
        self.kbhit_results = iter((False, True))

    def kbhit(self):
        return next(self.kbhit_results, False)

    def getwch(self):
        return self.key


class NoKeyWindowsConsole:
    def kbhit(self):
        return False

    def getwch(self):  # pragma: no cover - kbhit 恒为 False
        raise AssertionError("unexpected key read")


class WindowsRecordingKeyTests(unittest.TestCase):
    def _press(self, key, *, operator_control=False):
        console = FakeWindowsConsole(key)
        output = io.StringIO()
        with mock.patch.object(pico_record, "msvcrt", console), \
                mock.patch.object(pico_record.sys, "platform", "win32"), \
                contextlib.redirect_stdout(output):
            result = pico_record._wait_foreground(
                [], "key_test", {"flag": False}, 800,
                operator_control=operator_control,
            )
        return result, output.getvalue()

    def test_plain_mode_enter_and_q_stop_foreground_recording(self):
        for key in ("\r", "Q"):
            with self.subTest(key=key):
                (issue, action), _ = self._press(key)
                self.assertIsNone(issue)
                self.assertEqual("stop", action)

    def test_operator_mode_has_distinct_enter_h_q_actions(self):
        for key, expected in (("\r", "stop"), ("h", "restart"), ("Q", "quit")):
            with self.subTest(key=key):
                (issue, action), output = self._press(key, operator_control=True)
                self.assertIsNone(issue)
                self.assertEqual(expected, action)
                self.assertIn(key.upper() if key != "\r" else "Enter", output)

    def test_operator_exit_codes_are_distinct_from_success_and_failure(self):
        self.assertEqual(10, pico_record._EXIT_OPERATOR_RESTART)
        self.assertEqual(11, pico_record._EXIT_OPERATOR_QUIT)
        self.assertEqual(12, pico_record._EXIT_READY_TIMEOUT)
        self.assertEqual(13, pico_record._EXIT_SENSOR_FAULT)

    def test_ready_window_requires_continuous_stability(self):
        since, ready = pico_record._advance_ready_window(None, True, 10.0, 2.0)
        self.assertEqual(10.0, since)
        self.assertFalse(ready)
        since, ready = pico_record._advance_ready_window(since, True, 11.9, 2.0)
        self.assertFalse(ready)
        since, ready = pico_record._advance_ready_window(since, False, 12.0, 2.0)
        self.assertIsNone(since)
        self.assertFalse(ready)
        since, ready = pico_record._advance_ready_window(None, True, 20.0, 2.0)
        since, ready = pico_record._advance_ready_window(since, True, 22.0, 2.0)
        self.assertTrue(ready)

    def test_recording_fault_window_debounces_and_reports_recovery(self):
        since, confirmed, recovered = pico_record._advance_fault_window(
            None, True, 10.0, 1.5,
        )
        self.assertFalse(confirmed)
        self.assertFalse(recovered)
        since, confirmed, recovered = pico_record._advance_fault_window(
            since, True, 11.4, 1.5,
        )
        self.assertFalse(confirmed)
        since, confirmed, recovered = pico_record._advance_fault_window(
            since, False, 11.45, 1.5,
        )
        self.assertIsNone(since)
        self.assertTrue(recovered)
        since, _, _ = pico_record._advance_fault_window(None, True, 20.0, 1.5)
        _, confirmed, _ = pico_record._advance_fault_window(since, True, 21.5, 1.5)
        self.assertTrue(confirmed)

    def test_sensor_failure_marker_is_auditable(self):
        with tempfile.TemporaryDirectory() as directory:
            raw = Path(directory) / "raw"
            raw.mkdir()
            with mock.patch.object(
                pico_record, "_session_paths", return_value={"dir": raw},
            ):
                path = pico_record._write_capture_failure(
                    "expert_001", stage="recording", reason="VST stale",
                    max_age_ms=800, fault_grace_s=1.5,
                )
            self.assertEqual(raw / "capture.failure.json", path)
            payload = json.loads(path.read_text(encoding="utf-8"))
            self.assertEqual("invalid_sensor_dropout", payload["outcome"])
            self.assertEqual("recording", payload["stage"])
            self.assertEqual("VST stale", payload["reason"])

    def test_foreground_monitor_returns_distinct_sensor_fault_action(self):
        service = {"key": "pico", "label": "PICO", "port": 63910}
        with mock.patch.object(pico_record, "msvcrt", NoKeyWindowsConsole()), \
                mock.patch.object(pico_record.sys, "platform", "win32"), \
                mock.patch.object(
                    pico_record, "_cmd", return_value=("ERR offline", False),
                ), contextlib.redirect_stdout(io.StringIO()):
            issue, action = pico_record._wait_foreground(
                [service], "expert_001", {"flag": False}, 800,
                operator_control=True, fault_grace_s=0.0, health_poll_s=0.0,
            )
        self.assertIn("PICO", issue)
        self.assertEqual("sensor_fault", action)

    def test_recording_health_detects_async_writer_error(self):
        service = {
            "key": "pico", "label": "PICO", "expected_hands": "both",
            "monitor_motion_freshness": True, "require_vst": True,
        }
        status = (
            "OK REC session=expert_001 hands=both devices=PICO pose_age_ms=1 "
            "head=1 ctrl_l=1 ctrl_r=1 vst_on=1 vst_age_ms=1 "
            "writer_errors=1 vst_writer_errors=0"
        )
        issue = pico_record._recording_issue(
            service, status, True, "expert_001", 800,
        )
        self.assertIn("writer_errors=1", issue)

    def test_recording_health_detects_motion_writer_backlog(self):
        service = {
            "key": "manus", "label": "MANUS", "expected_hands": "both",
            "monitor_motion_freshness": False,
        }
        status = (
            "OK REC session=expert_001 hands=both gloves=left,right "
            "writer_errors=0 write_backlog=513"
        )
        issue = pico_record._recording_issue(
            service, status, True, "expert_001", 800,
        )
        self.assertIn("write_backlog=513", issue)

    def test_compact_status_keeps_all_operator_health_signals(self):
        cases = (
            ({"key": "pico", "label": "PICO"},
             "OK REC pose_age_ms=7 vst_age_ms=8 vst_record_frames=99 "
             "write_backlog=2 vst_write_backlog=3",
             ("PICO:p7ms", "v8ms/99", "q2/3")),
            ({"key": "manus", "label": "MANUS"},
             "OK REC age_ms=l=4,r=6 frames=123 write_backlog=5",
             ("MANUS:lr4/6ms", "n123", "q5")),
            ({"key": "tactile", "label": "TACTILE"},
             "OK REC age_ms=l=9,r=10 frames=l=88,r=89 warnings=1",
             ("TACT:lr9/10ms", "n88/89", "w1")),
        )
        for service, status, expected in cases:
            with self.subTest(service=service["key"]):
                summary = pico_record._compact_recording_status(
                    service, status, True,
                )
                for token in expected:
                    self.assertIn(token, summary)
        combined = " | ".join(
            pico_record._compact_recording_status(service, status, True)
            for service, status, _ in cases
        )
        self.assertLessEqual(len(combined), 80)

    def test_queue_drain_wait_is_bounded_and_observes_completion(self):
        work = queue.Queue()
        work.put("pending")
        started = time.monotonic()
        self.assertFalse(wait_queue_drained(work, 0.03))
        self.assertLess(time.monotonic() - started, 0.5)

        def finish_item():
            time.sleep(0.02)
            work.get_nowait()
            work.task_done()

        thread = threading.Thread(target=finish_item)
        thread.start()
        self.assertTrue(wait_queue_drained(work, 0.5))
        thread.join()

if __name__ == "__main__":
    unittest.main()
