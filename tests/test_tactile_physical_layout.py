import unittest
from pathlib import Path

import numpy as np

from export_dataset import tactile_spatial_views
from tactile_layout import (
    TACTILE_FINGER_ORDER,
    TACTILE_FINGER_REGIONS,
    TACTILE_LAYOUT_SCHEMA,
    TACTILE_PHYSICAL_ACTIVE_COUNT,
    active_coordinates,
    physical_invalid_grid_coordinates,
)
from tactile_pairing import load_pairing_config


ROOT = Path(__file__).resolve().parents[1]
CELLMAP_HEX = (
    "ffe70000ffe70000ffe70000ffe70000ffe70000ffe70000ffaf0000ffef0000"
    "ffff0000ffff0000ffff0000ffff0000ffff0000ffff0000ffff0000ffff0000"
    "ffff0000ffff0000ffff0000ffff0000ffff0000ffff0000ffff0000ffff0000"
)
WIRE_LAYOUT = {
    "rows": 24,
    "cols": 16,
    "active_count": 369,
    "cellmap_hex": CELLMAP_HEX,
}


class Hs13PhysicalLayoutTests(unittest.TestCase):
    def test_hs13_physical_fingers_are_5x4x8_without_palm(self):
        coords = active_coordinates(WIRE_LAYOUT)
        coord_to_wire = {coord: index + 1 for index, coord in enumerate(coords)}
        aligned = {"values": np.arange(1, 370, dtype=np.int16)[None, :]}

        spatial = tactile_spatial_views(aligned, WIRE_LAYOUT)

        self.assertEqual(set(spatial), {"fingers", "fingers_active_mask"})
        self.assertEqual(spatial["fingers"].shape, (1, 5, 4, 8))
        self.assertEqual(spatial["fingers_active_mask"].shape, (5, 4, 8))
        self.assertEqual(
            int(spatial["fingers_active_mask"].sum()),
            TACTILE_PHYSICAL_ACTIVE_COUNT,
        )

        for finger_index, finger in enumerate(TACTILE_FINGER_ORDER):
            r0, _r1, c0, _c1 = TACTILE_FINGER_REGIONS[finger]
            mask = spatial["fingers_active_mask"][finger_index]
            self.assertEqual(int(mask.sum()), 32 if finger == "thumb" else 28)
            for across in range(4):
                for tip_to_base in range(8):
                    actual = int(
                        spatial["fingers"][0, finger_index, across, tip_to_base]
                    )
                    if finger != "thumb" and tip_to_base == 7:
                        self.assertFalse(mask[across, tip_to_base])
                        self.assertEqual(actual, 0)
                    else:
                        self.assertTrue(mask[across, tip_to_base])
                        self.assertEqual(
                            actual,
                            coord_to_wire[(r0 + tip_to_base, c0 + across)],
                        )

    def test_default_pairing_profile_matches_physical_layout(self):
        config = load_pairing_config(ROOT / "config" / "tactile_pairing.json")
        profile_name = config.side_profiles["left"]
        self.assertEqual(profile_name, config.side_profiles["right"])
        profile = config.profiles[profile_name]

        self.assertEqual(
            profile["kind"], "five_fingertip_arrays_4x8_four_4x7"
        )
        self.assertIs(profile["mask_verified"], True)
        self.assertIs(profile["palm_present"], False)
        self.assertEqual(profile["physical_active_count"], 144)
        self.assertEqual(
            {tuple(cell) for cell in profile["post_decode_invalid_cells"]},
            set(physical_invalid_grid_coordinates()),
        )
        self.assertEqual(TACTILE_LAYOUT_SCHEMA, profile_name)


if __name__ == "__main__":
    unittest.main()
