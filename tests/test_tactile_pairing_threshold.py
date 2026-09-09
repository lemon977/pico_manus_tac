import unittest

from tactile_pairing import ActivityEvidence, ScoringConfig, select_activity_winner


class WeakPressureThresholdTests(unittest.TestCase):
    def _evidence(self, label, score, active_frames=25, peak_cells=2,
                  press_frames=40):
        return ActivityEvidence(
            requested_side="left",
            candidate_label=label,
            port=label,
            baseline_frames=40,
            press_frames=press_frames,
            baseline_score=0.0,
            raw_press_score=score,
            score=score,
            active_frame_count=active_frames,
            peak_active_cells=peak_cells,
        )

    def test_sustained_weak_pressure_can_identify_left(self):
        winner = self._evidence("candidate_1", 25.0)
        quiet = self._evidence("candidate_2", 2.0, active_frames=0, peak_cells=0)
        self.assertEqual(
            select_activity_winner("left", (winner, quiet), ScoringConfig()),
            "candidate_1",
        )

    def test_zero_signal_still_fails(self):
        evidence = self._evidence("candidate_1", 0.0, active_frames=0, peak_cells=0)
        with self.assertRaises(Exception):
            select_activity_winner("left", (evidence, evidence), ScoringConfig())

    def test_human_reaction_window_accepts_34_of_240_active_frames(self):
        winner = self._evidence(
            "candidate_1", 25.0, active_frames=34, press_frames=240)
        quiet = self._evidence(
            "candidate_2", 2.0, active_frames=0, peak_cells=0,
            press_frames=240)
        self.assertEqual(
            select_activity_winner("left", (winner, quiet), ScoringConfig()),
            "candidate_1",
        )

    def test_short_spike_still_fails(self):
        winner = self._evidence(
            "candidate_1", 25.0, active_frames=23, press_frames=240)
        quiet = self._evidence(
            "candidate_2", 2.0, active_frames=0, peak_cells=0,
            press_frames=240)
        with self.assertRaises(Exception):
            select_activity_winner("left", (winner, quiet), ScoringConfig())


if __name__ == "__main__":
    unittest.main()
