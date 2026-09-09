import json
from pathlib import Path
import tempfile
import unittest

from pico_camera_params import (
    device_serial_from_status,
    select_camera_params,
    snapshot_camera_params,
)


VALID_PARAMS = {
    "reference_resolution": {"w": 3248, "h": 2464},
    "left": {
        "intrinsics": {"fx": 1, "fy": 2, "cx": 3, "cy": 4},
        "distortion": {"model": "equiDis62", "coeffs": [0.1]},
    },
    "right": {
        "intrinsics": {"fx": 5, "fy": 6, "cx": 7, "cy": 8},
        "distortion": {"model": "equiDis62", "coeffs": [0.2]},
    },
}

FULL_PARAMS = {
    "width": 2048, "height": 1536,
    "outputWidth": 4096, "outputHeight": 1536, "fps": 30,
    "left": {
        "fx": 1, "fy": 2, "cx": 3, "cy": 4,
        "hasExtrinsics": True,
        "distortion": {"model": "equiDis62", "coeffs": [0.1]},
        "extrinsic": list(range(12)),
    },
    "right": {
        "fx": 5, "fy": 6, "cx": 7, "cy": 8,
        "hasExtrinsics": True,
        "distortion": {"model": "equiDis62", "coeffs": [0.2]},
        "extrinsic": list(range(12)),
    },
}


class PicoCameraParamsTest(unittest.TestCase):
    def test_device_serial_requires_exactly_one_device(self):
        self.assertEqual(
            device_serial_from_status("OK idle devices=PICO123 pose_age_ms=1"),
            "PICO123",
        )
        with self.assertRaises(ValueError):
            device_serial_from_status("OK idle devices=(none)")
        with self.assertRaises(ValueError):
            device_serial_from_status("OK idle devices=A,B")

    def test_select_and_snapshot_exact_source(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            source = root / "config" / "SERIAL1" / "camera_params.json"
            source.parent.mkdir(parents=True)
            content = (json.dumps(VALID_PARAMS, indent=2) + "\n").encode()
            source.write_bytes(content)

            selection = select_camera_params(
                "OK idle devices=SERIAL1", root / "config")
            target, meta_target = snapshot_camera_params(
                root / "session" / "raw", selection)

            self.assertEqual(target.read_bytes(), content)
            meta = json.loads(meta_target.read_text(encoding="utf-8"))
            self.assertEqual(meta["device_serial"], "SERIAL1")
            self.assertFalse(meta["extrinsics_included"])

    def test_full_capturelib_params_include_extrinsics(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            source = root / "config" / "SERIAL1" / "camera_params.json"
            source.parent.mkdir(parents=True)
            source.write_text(json.dumps(FULL_PARAMS), encoding="utf-8")

            selection = select_camera_params(
                "OK idle devices=SERIAL1", root / "config")

            self.assertTrue(selection.extrinsics_included)
            self.assertIn("extrinsics", selection.contains)

    def test_missing_device_config_is_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            with self.assertRaises(ValueError):
                select_camera_params("OK idle devices=UNKNOWN", temp)


if __name__ == "__main__":
    unittest.main()
