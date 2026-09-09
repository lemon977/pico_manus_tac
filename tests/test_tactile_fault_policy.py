import threading
import unittest

from tactile_collector import TransientHealthTracker
from tactile_pairing import FrameSubscription, PairingError, ReaderHealth


class _FakeReader:
    def __init__(self) -> None:
        self.stream_id = "test-stream"
        self.health = ReaderHealth()
        self.error = None
        self.stop_event = threading.Event()
        self._samples_changed = threading.Condition()
        self._samples = []
        self._next_stream_seq = 0
        self._notification_seq = 0

    def health_snapshot(self) -> ReaderHealth:
        return self.health

    def notify_subscribers(self) -> None:
        with self._samples_changed:
            self._notification_seq += 1
            self._samples_changed.notify_all()


class FrameSubscriptionHealthTests(unittest.TestCase):
    def _subscription(self, reader: _FakeReader) -> FrameSubscription:
        return FrameSubscription(
            reader,
            side="left",
            start_mono_ns=0,
            start_wall_ns=0,
            next_sequence=0,
        )

    def test_timeout_and_parser_discard_are_transient(self) -> None:
        reader = _FakeReader()
        subscription = self._subscription(reader)
        reader.health = ReaderHealth(timeouts=1, parser_discarded=736)

        _current, delta = subscription._checked_health()

        self.assertEqual(delta.timeouts, 1)
        self.assertEqual(delta.parser_discarded, 736)
        self.assertIsNone(subscription._failure)

    def test_crc_error_remains_fail_closed(self) -> None:
        reader = _FakeReader()
        subscription = self._subscription(reader)
        reader.health = ReaderHealth(parser_crc_errors=1)

        with self.assertRaises(PairingError):
            subscription._checked_health()
        self.assertIsNotNone(subscription._failure)


class TransientHealthTrackerTests(unittest.TestCase):
    def test_third_incident_in_window_reaches_threshold(self) -> None:
        tracker = TransientHealthTracker(incident_limit=3, window_s=5.0)
        delta = ReaderHealth(timeouts=1, parser_discarded=736)

        first = tracker.observe("left", delta, now_mono=0.0)
        second = tracker.observe("left", delta, now_mono=1.0)
        third = tracker.observe("left", delta, now_mono=2.0)

        self.assertFalse(first["fault_threshold_reached"])
        self.assertFalse(second["fault_threshold_reached"])
        self.assertTrue(third["fault_threshold_reached"])
        snapshot = tracker.snapshot(now_mono=2.0)
        self.assertEqual(snapshot["incident_count_by_side"]["left"], 3)
        self.assertEqual(
            snapshot["counters_by_side"]["left"]["parser_discarded"],
            2208,
        )

    def test_old_incident_expires_from_active_window(self) -> None:
        tracker = TransientHealthTracker(incident_limit=3, window_s=5.0)
        delta = ReaderHealth(timeouts=1)
        tracker.observe("right", delta, now_mono=0.0)

        snapshot = tracker.snapshot(now_mono=6.0)

        self.assertEqual(snapshot["incident_count_by_side"]["right"], 1)
        self.assertEqual(snapshot["active_window_incidents_by_side"]["right"], 0)


if __name__ == "__main__":
    unittest.main()
