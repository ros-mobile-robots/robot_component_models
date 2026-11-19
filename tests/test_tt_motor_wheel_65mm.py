# SPDX-FileCopyrightText: 2026 ROS Mobile Robots
# SPDX-License-Identifier: BSD-3-Clause
"""Tests of the wheel's hub: the bulges, the notches, the bowl's pins and the checks around them.

generators/tt_motor_wheel_65mm.py builds the bulges at the hub tube's foot from two rounds seen
along the axle, one on each bulge and one in each notch. It refuses values that give no such
shape, and make_rim() refuses a rim whose union lost part of a bulge, which OCCT can do without
an error. The tests change the generator's values with mock and look at what the functions
return or refuse. Run from the repository's root, with the packages of
generators/requirements.txt:

  python3 -m unittest discover -s tests
"""
import math
from typing import Any
from typing import ContextManager
from typing import Dict
import unittest
from unittest import mock

from generators import tt_motor_wheel_65mm as wheel

TOLERANCE = 1e-9  # mm, for values that the tests derive with other formulas

# A bulge's round of the radius that a width of 3.47 mm 1.0 mm below the top gives: with it, a
# notch's round of 1.4 mm touches the bulges' rounds where they face the axle
SMALL_BULGE_RADIUS = (1.0 ** 2 + (wheel.HUB_BULGE_WIDTH / 2) ** 2) / (2 * 1.0)
NOTCH_ROUND_THAT_FOLDS_BACK = 1.4
# With SMALL_BULGE_RADIUS, a notch's round of 2.235 mm puts the notches' bottoms 0.004 mm
# inside the hub tube's surface
NOTCH_ROUND_ON_THE_TUBE = 2.235
# A bulge's round as narrow as the measured width and a large notch's round. Measured about the
# axle from the bulge's centre line, the notch's round takes over at 14.5 deg, and the measured
# width ends at 16.4 deg
NARROW_BULGE_RADIUS = 1.735
LARGE_NOTCH_ROUND = 4.0
# The pins' radius at which their feet reach onto the bowl's round, and at which they reach
# onto the boss's foot round
PINS_ON_THE_ROUND = 6.42
PINS_ON_THE_BOSS = 5.74
# fin() asks the edge's inner corner to lie at least two rounds' radii inside the outer corner,
# measured along the radius. The inner corner moves out as the round grows, so at the model's
# other values the largest round that passes is about 4.080 mm
FIN_ROUND_PAST_THE_SPACING_RULE = 4.1
# The values with which OCCT's union lost part of a bulge and still returned a valid solid:
# the notches' bottoms 0.05 mm inside the hub tube, a small circle of the bulges' tops
UNION_LOSES_A_BULGE = {
    'HUB_COLLAR_D': 14.0, 'HUB_PLATEAU': 0.05, 'HUB_BULGE_RADIUS': SMALL_BULGE_RADIUS,
    'HUB_NOTCH_ROUND': 2.3998, 'HUB_BULGE_ROUND': 1.6, 'CUP_BACK_ROUND_ALONG_SIDE': 4.5,
    'FIN_BARREL_CORNER_R': 18.6, 'FIN_ROUND': 3.0, 'HUB_PIN_R': 6.4,
}


def with_values(values: Dict[str, float]) -> ContextManager[Any]:
    """Return a patch of the generator's values that also sets the value that follows from them.

    CUP_BACK_ROUND_ALONG_PLATE follows from HUB_COLLAR_D and HUB_PLATEAU when the module loads.
    """
    collar_d = values.get('HUB_COLLAR_D', wheel.HUB_COLLAR_D)
    plate = values.get('HUB_PLATEAU', wheel.HUB_PLATEAU)
    along_plate = wheel.CUP_D / 2 - collar_d / 2 - plate
    changed: Dict[str, Any] = {'CUP_BACK_ROUND_ALONG_PLATE': along_plate, **values}
    return mock.patch.multiple(wheel, **changed)


class BulgeAndNotchRoundsTest(unittest.TestCase):
    """The rounds of the bulges and the notches, seen along the axle."""

    def test_bulge_width_depth_is_where_the_round_has_the_measured_width(self) -> None:
        """Take the depth below the top: the round's chord there is the measured 3.47 mm."""
        radius, depth = wheel.HUB_BULGE_RADIUS, wheel.bulge_width_depth()

        chord = 2 * math.sqrt(radius ** 2 - (radius - depth) ** 2)

        self.assertAlmostEqual(chord, wheel.HUB_BULGE_WIDTH, delta=TOLERANCE)

    def test_bulge_width_depth_raises_a_value_error_for_a_round_narrower_than_the_width(
            self) -> None:
        """Make the round's diameter less than 3.47 mm: an error, no chord is that wide."""
        with with_values({'HUB_BULGE_RADIUS': wheel.HUB_BULGE_WIDTH / 2 - 0.01}):
            with self.assertRaisesRegex(ValueError, "at least as wide as the bulge's measured"):
                wheel.bulge_width_depth()

    def test_collar_rounds_has_a_notch_round_that_touches_both_bulges_rounds(self) -> None:
        """Measure between the notch round's and a bulge round's centres: the sum of the radii."""
        bulge_centre, bulge_r, notch_centre, notch_r = wheel.collar_rounds()
        pitch = math.pi / wheel.SPOKES

        apart = math.hypot(notch_centre * math.cos(pitch) - bulge_centre,
                           notch_centre * math.sin(pitch))

        self.assertAlmostEqual(apart, bulge_r + notch_r, delta=TOLERANCE)
        self.assertAlmostEqual(bulge_centre + bulge_r, wheel.HUB_COLLAR_D / 2, delta=TOLERANCE)

    def test_collar_rounds_has_the_measured_width_on_the_bulges_round(self) -> None:
        """Compare the width's end and the touch point, seen from the bulge round's centre."""
        bulge_centre, bulge_r, notch_centre, _ = wheel.collar_rounds()
        pitch = math.pi / wheel.SPOKES

        width_end = math.atan2(wheel.HUB_BULGE_WIDTH / 2, bulge_r - wheel.bulge_width_depth())
        touch = math.atan2(notch_centre * math.sin(pitch),
                           notch_centre * math.cos(pitch) - bulge_centre)

        self.assertLess(width_end, touch)

    def test_collar_rounds_raises_a_value_error_for_a_width_past_the_bulges_round(self) -> None:
        """Let the notch's round take over before the width's end: an error, it would be wider."""
        with with_values({'HUB_BULGE_RADIUS': NARROW_BULGE_RADIUS,
                          'HUB_NOTCH_ROUND': LARGE_NOTCH_ROUND}):
            with self.assertRaisesRegex(ValueError, 'measured width must lie on its round'):
                wheel.collar_rounds()

    def test_collar_rounds_raises_a_value_error_for_a_notch_round_that_folds_back(self) -> None:
        """Touch the bulges' rounds where they face the axle: an error, the outline would fold."""
        with with_values({'HUB_BULGE_RADIUS': SMALL_BULGE_RADIUS,
                          'HUB_NOTCH_ROUND': NOTCH_ROUND_THAT_FOLDS_BACK}):
            with self.assertRaisesRegex(ValueError, 'where they face away from the axle'):
                wheel.collar_rounds()

    def test_collar_rounds_raises_a_value_error_for_notch_bottoms_on_the_tube(self) -> None:
        """Put the notches' bottoms within 0.05 mm of the hub tube's surface: an error."""
        with with_values({'HUB_BULGE_RADIUS': SMALL_BULGE_RADIUS,
                          'HUB_NOTCH_ROUND': NOTCH_ROUND_ON_THE_TUBE}):
            with self.assertRaisesRegex(ValueError, "off the hub tube's surface"):
                wheel.collar_rounds()

    def test_bulge_top_end_lies_one_side_round_before_the_bulges_end(self) -> None:
        """Take where a bulge's level top ends: HUB_BULGE_ROUND before the bulges' end plane."""
        self.assertAlmostEqual(wheel.bulge_top_end(),
                               wheel.HUB_COLLAR_END_Z - wheel.HUB_BULGE_ROUND, delta=TOLERANCE)

    def test_bulge_top_end_raises_a_value_error_for_a_side_round_below_the_notches(self) -> None:
        """Make the side round end below the notches' bottoms: an error."""
        lowest = wheel.HUB_COLLAR_D / 2 - wheel.notch_bottom()
        with with_values({'HUB_BULGE_ROUND': lowest + 0.01}):
            with self.assertRaisesRegex(ValueError, "and the notches' bottoms"):
                wheel.bulge_top_end()


class PinsTest(unittest.TestCase):
    """The pins stand on the bowl's floor, between the boss and the floor's round."""

    def test_pins_on_floor_is_true_with_the_feet_ending_where_the_round_starts(self) -> None:
        """Take the model's pins: their feet end at the round's start, on the floor."""
        self.assertAlmostEqual(wheel.HUB_PIN_R + wheel.HUB_PIN_D / 2, wheel.BOWL_FLOOR_D / 2,
                               delta=TOLERANCE)
        self.assertTrue(wheel.pins_on_floor())

    def test_pins_on_floor_is_false_for_feet_on_the_floors_round(self) -> None:
        """Move the pins out until the round rises more under their feet than they reach in."""
        with with_values({'HUB_PIN_R': PINS_ON_THE_ROUND}):
            self.assertFalse(wheel.pins_on_floor())

    def test_pins_on_floor_is_false_for_feet_on_the_boss_foot_round(self) -> None:
        """Move the pins in onto the round at the boss's foot: not on the floor."""
        with with_values({'HUB_PIN_R': PINS_ON_THE_BOSS}):
            self.assertFalse(wheel.pins_on_floor())


class HubAndFinTest(unittest.TestCase):
    """The cup's rounded back edge and the fin's edge stay where the hub has room for them."""

    def test_hub_raises_a_value_error_for_a_cup_round_that_runs_into_the_spokes(self) -> None:
        """Make the cup's round longer along its outside than the room behind the spokes."""
        room = wheel.CUP_BACK_Z - wheel.SPOKE_PLATE_BACK_Z
        with with_values({'CUP_BACK_ROUND_ALONG_SIDE': room + 0.01}):
            with self.assertRaisesRegex(ValueError, 'need room behind the spokes'):
                wheel.hub()

    def test_fin_raises_a_value_error_for_corners_closer_than_two_radii_along_the_radius(
            self) -> None:
        """Make the rounds larger than half the corners' distance along the radius: an error."""
        with with_values({'FIN_ROUND': FIN_ROUND_PAST_THE_SPACING_RULE}):
            with self.assertRaisesRegex(ValueError, "the fin's edge must run from the barrel"):
                wheel.fin()


class RimUnionTest(unittest.TestCase):
    """make_rim() checks that the union of the rim's solids kept the bulges."""

    def test_make_rim_raises_a_value_error_when_the_union_loses_part_of_a_bulge(self) -> None:
        """Build with the values whose union lost part of a bulge: an error, not a rim."""
        with with_values(UNION_LOSES_A_BULGE):
            with self.assertRaisesRegex(ValueError, "the rim's union must keep the bulges"):
                wheel.make_rim()


if __name__ == '__main__':
    unittest.main()
