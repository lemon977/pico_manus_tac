import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from manus_collector import _calibration_file_info
from export_dataset import failed_manus_calibration_sides
from pico_record import _check_ready
from session_layout import new_session_paths, write_manifest


class ManusCalibrationGateTests(unittest.TestCase):
    def test_export_calibration_gate_checks_both_sides_not_joint_selection(self):
        evidence = {
            "calibration": {
                "left": {"sdk_applied": True},
                "right": {"sdk_applied": True},
            }
        }
        self.assertEqual(failed_manus_calibration_sides(evidence), [])

        evidence["calibration"]["right"]["sdk_applied"] = False
        self.assertEqual(failed_manus_calibration_sides(evidence), ["right"])

    def test_file_evidence_contains_sha256(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "manus_left.mcal"
            path.write_bytes(b"personal calibration")
            with patch.dict(os.environ, {"MANUS_CALIB_LEFT": str(path)}):
                info = _calibration_file_info("left")
        self.assertTrue(info["exists"])
        self.assertEqual(info["file_name"], "manus_left.mcal")
        self.assertEqual(len(info["sha256"]), 64)

    def test_readiness_rejects_sdk_calibration_failure(self):
        status = (
            "OK idle session=x frames=0 hands=both gloves=left,right "
            "age_ms=l=1,r=1 nodes=ln=25,rn=25 calib=l=1,r=0 "
            "writer_errors=0 write_backlog=0"
        )
        reasons = _check_ready(
            "", status, "", do_pico=False, do_manus=True,
            do_tactile=False, hands=frozenset(("left", "right")),
            max_age_ms=500, require_vst=False,
        )
        self.assertTrue(any("right" in reason and "标定" in reason for reason in reasons))

    def test_manifest_embeds_capture_calibration_evidence(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir) / "data"
            paths = new_session_paths(root / "sessions", "sample_001", data_root=root)
            paths["dir"].mkdir(parents=True)
            evidence = {
                "left": {"sha256": "a" * 64, "sdk_applied": True},
                "right": {"sha256": "b" * 64, "sdk_applied": True},
            }
            paths["manus_meta"].write_text(json.dumps({
                "calibration_required": True,
                "calibration": evidence,
            }), encoding="utf-8")
            manifest = write_manifest(paths)
            doc = json.loads(manifest.read_text(encoding="utf-8"))
        self.assertEqual(doc["manus_calibration"], evidence)
        self.assertTrue(doc["manus_calibration_required"])


if __name__ == "__main__":
    unittest.main()
