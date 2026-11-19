# SPDX-FileCopyrightText: 2026 ROS Mobile Robots
# SPDX-License-Identifier: BSD-3-Clause
"""Tests of the wheel's hub: the bulges, the notches, the bowl's pins and the checks around them.

generators/tt_motor_wheel_65mm.py builds the bulges at the hub tube's foot from two rounds seen
along the axle, one on each bulge and one in each notch, and from a bulge's side view, which
runs down into the tube without a flat ring. It refuses values that give no such shape, and
make_rim() refuses a rim whose union lost part of a bulge, which OCCT can do without an error.
The tests change the generator's values with mock and look at what the functions return or
refuse. Run from the repository's root, with the packages of
generators/requirements.txt:

  python3 -m unittest discover -s tests
"""
import math
from typing import Any
from typing import ContextManager
from typing import Dict
from typing import Iterator
import unittest
from unittest import mock

import cadquery as cq
import numpy as np

from generators import tt_motor_wheel_65mm as wheel

TOLERANCE = 1e-9  # mm, for values that the tests derive with other formulas
SURFACE = 0.01    # mm: how close to a surface a point counts as on it
# The notches' bottoms as read on a photo of the hub's motor side: r 5.2 to 5.5 mm, and the
# middle that the model takes, with the notch round rounded to 0.35 mm
NOTCH_BOTTOM_READ = (5.2, 5.5)
NOTCH_BOTTOM_MODEL = 5.35
NOTCH_BOTTOM_TOLERANCE = 0.005

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
# other values the largest round that passes is about 4.176 mm
FIN_ROUND_PAST_THE_SPACING_RULE = 4.2
# Larger than the tube's chamfered end (9.95 - 2 * 0.5 = 8.95 mm), smaller than the tube
BORE_THROUGH_THE_CHAMFER = 9.5
# A bulge's round of 2.8 mm with the model's notch round puts the notches' bottoms on r 4.92,
# inside the hub tube
BULGE_RADIUS_WITH_NOTCHES_IN_THE_TUBE = 2.8
# Bulge rounds at which rounding once made the side view's square roots negative at the joint of
# its two rounds
ROUNDS_AT_THE_JOINT = (1.45, 1.6, 1.61, 1.7)
# A bulge's round of 2.2 mm leaves a foot round of 0.46 mm. Halfway along it, a point 0.08 mm
# inside its surface lies inside the tube, so the union check has to probe where the foot stands
# 0.4 mm off the tube. A round of 2.3 mm leaves no such place: the build stops
LARGER_BULGE_ROUND = 2.2
BULGE_ROUND_WITH_A_TOO_SMALL_FOOT = 2.3
# A notch's round of 0.1 mm puts the notches' bottoms 0.15 mm off the tube. Halfway between them
# and the tube, a point 0.08 mm inside the foot round lies inside the tube, so the union check
# has no point to probe there
NOTCH_ROUND_NEAR_THE_TUBE = 0.1
# The largest angle (deg) between the bulges' and the tube's surfaces where they meet that counts
# as one smooth surface. A foot round tangent to a line 0.05 mm inside the tube met it at 17.3
TANGENT = 0.1
# A box that takes the top off one bulge, from the plate to its round, as a union could lose it
BULGE_TOP_LOST = ((wheel.HUB_COLLAR_D / 2 - 0.5, -1.0, wheel.CUP_BACK_Z),
                  (wheel.HUB_COLLAR_D / 2 + 0.5, 1.0, wheel.HUB_COLLAR_END_Z))


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
            with self.assertRaisesRegex(ValueError, 'outside the hub tube'):
                wheel.collar_rounds()

    def test_collar_rounds_raises_a_value_error_for_notch_bottoms_inside_the_tube(self) -> None:
        """Make the bulges' round smaller: the notches' bottoms fall inside the tube, an error."""
        with with_values({'HUB_BULGE_RADIUS': BULGE_RADIUS_WITH_NOTCHES_IN_THE_TUBE}):
            with self.assertRaisesRegex(ValueError, 'outside the hub tube'):
                wheel.collar_rounds()

    def test_notch_bottom_lies_in_the_photos_reading_off_the_tube(self) -> None:
        """Take the notches' bottoms: the middle of r 5.2 to 5.5, outside the hub tube."""
        bottom = wheel.notch_bottom()

        self.assertAlmostEqual(bottom, NOTCH_BOTTOM_MODEL, delta=NOTCH_BOTTOM_TOLERANCE)
        self.assertTrue(NOTCH_BOTTOM_READ[0] < bottom < NOTCH_BOTTOM_READ[1])
        self.assertGreater(bottom, wheel.HUB_TUBE_D / 2)


def side_view(steps: int = 2000) -> Iterator[float]:
    """Yield z from the level top's end to where the bulges meet the tube, in even steps."""
    top_end = wheel.bulge_end()[0]
    for k in range(steps + 1):
        yield top_end + (wheel.HUB_COLLAR_END_Z - top_end) * k / steps


class BulgeEndTest(unittest.TestCase):
    """A bulge's side view runs from its level top down into the hub tube, without a flat ring."""

    def test_bulge_profile_meets_the_tube_where_the_bulges_meet_it(self) -> None:
        """Take the side view at HUB_COLLAR_END_Z: the tube's radius, 17.5 mm before its end."""
        self.assertAlmostEqual(wheel.bulge_profile(wheel.HUB_COLLAR_END_Z), wheel.HUB_TUBE_D / 2,
                               delta=TOLERANCE)

    def test_bulge_profile_meets_the_tube_tangentially(self) -> None:
        """Step 0.0001 mm back from where the bulges meet the tube: the side view is level."""
        step = 1e-4
        rise = wheel.bulge_profile(wheel.HUB_COLLAR_END_Z - step) - wheel.HUB_TUBE_D / 2

        self.assertLess(rise / step, 1e-3)

    def test_bulge_profile_has_a_value_at_the_joint_of_its_rounds(self) -> None:
        """Take the joint and its neighbouring heights for several rounds: no math error."""
        for bulge_round in ROUNDS_AT_THE_JOINT:
            with with_values({'HUB_BULGE_ROUND': bulge_round}):
                joint = wheel.bulge_end()[0] + bulge_round
                below, above = np.nextafter(joint, -np.inf), np.nextafter(joint, np.inf)
                for z in (float(below), joint, float(above)):
                    self.assertAlmostEqual(wheel.bulge_profile(z),
                                           wheel.HUB_COLLAR_D / 2 - bulge_round, delta=1e-6)

    def test_bulge_profile_falls_steadily_from_the_top_to_the_tube(self) -> None:
        """Step along the side view: it never rises and never jumps, so it has no step."""
        places = list(side_view())
        radii = [wheel.bulge_profile(z) for z in places]
        largest_step = max(abs(z - w) for z, w in zip(places, places[1:]))

        self.assertAlmostEqual(radii[0], wheel.HUB_COLLAR_D / 2, delta=TOLERANCE)
        self.assertAlmostEqual(radii[-1], wheel.HUB_TUBE_D / 2, delta=TOLERANCE)
        self.assertTrue(all(b <= a + TOLERANCE for a, b in zip(radii, radii[1:])))
        # a round of radius R falls fastest where it runs square to the axle: by about
        # sqrt(2 R dz) in a step dz. A flat ring would be a fall of about 1 mm in one step
        steepest = max(a - b for a, b in zip(radii, radii[1:]))
        self.assertLess(steepest, 2 * math.sqrt(2 * wheel.HUB_BULGE_ROUND * largest_step))

    def test_collar_radius_is_the_same_all_round_where_the_side_view_is_below_the_notches(
            self) -> None:
        """Take a place between the notches' bottoms and the tube: every angle has one radius."""
        between = (wheel.notch_bottom() + wheel.HUB_TUBE_D / 2) / 2
        z = next(z for z in side_view() if wheel.bulge_profile(z) < between)

        radii = {round(wheel.collar_radius(math.radians(a), z), 9) for a in range(0, 360, 3)}

        self.assertEqual(len(radii), 1)

    def test_collar_has_no_flat_ring_at_its_end(self) -> None:
        """Look at the bulges' faces across the axle: none reaches out of the tube."""
        tube = wheel.HUB_TUBE_D / 2
        outside = []
        for face in wheel.collar().faces().vals():
            if face.geomType() != 'PLANE' or abs(face.normalAt().z) < 1 - TOLERANCE:
                continue
            if face.Center().z < wheel.CUP_BACK_Z:
                continue                                  # the bottom, inside the cup's back
            reach = max(math.hypot(point.x, point.y) for edge in face.Edges()
                        for point in (edge.positionAt(t / 20) for t in range(21)))
            if reach > tube + SURFACE:
                outside.append((round(face.Center().z, 3), round(reach, 3)))

        self.assertEqual(outside, [])

    def test_bulges_and_tube_have_one_normal_where_they_meet(self) -> None:
        """Join the bulges to the hub: beside a fin and in a notch both faces have one normal."""
        joined = wheel.hub().union(wheel.collar()).val()
        tube, end = wheel.HUB_TUBE_D / 2, wheel.HUB_COLLAR_END_Z
        for degrees in (9.0, 36.0):
            angle = math.radians(degrees)
            point = cq.Vector(tube * math.cos(angle), tube * math.sin(angle), end)
            faces = [face for face in joined.Faces()
                     if face.distance(cq.Vertex.makeVertex(*point.toTuple())) < SURFACE]
            normals = [face.normalAt(point) for face in faces]
            spread = max(math.degrees(a.getAngle(b)) for a in normals for b in normals)

            self.assertEqual({face.geomType() for face in faces}, {'CYLINDER', 'TORUS'})
            self.assertLess(spread, TANGENT)

    def test_bulge_end_raises_a_value_error_for_a_round_with_no_room_for_the_foot(self) -> None:
        """Make the bulge's round reach down to the tube: no room is left for the foot round."""
        room = wheel.HUB_COLLAR_D / 2 - wheel.HUB_TUBE_D / 2
        with with_values({'HUB_BULGE_ROUND': room}):
            with self.assertRaisesRegex(ValueError, 'between its level top and the hub tube'):
                wheel.bulge_end()


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
    """The cup's rounded back edge, the fin's edge and the bore stay where the hub has room."""

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

    def test_make_rim_raises_a_value_error_for_a_bore_through_the_tubes_chamfered_end(
            self) -> None:
        """Make the bore wider than the tube's chamfered end: an error before the rim is built."""
        with with_values({'BORE_D': BORE_THROUGH_THE_CHAMFER}):
            with self.assertRaisesRegex(ValueError, 'the bore must lie inside the hub tube'):
                wheel.make_rim()


class RimUnionTest(unittest.TestCase):
    """collar_kept() tells a rim that kept the bulges from one that lost part of a bulge."""

    bulges: cq.Workplane

    @classmethod
    def setUpClass(cls) -> None:
        """Build the bulges once."""
        cls.bulges = wheel.collar()

    def test_collar_kept_is_true_for_a_rim_with_the_whole_bulges(self) -> None:
        """Give the bulges themselves as the rim: every point inside them is kept."""
        self.assertTrue(wheel.collar_kept(self.bulges, self.bulges))

    def test_collar_kept_is_false_for_a_rim_that_lost_a_bulges_top(self) -> None:
        """Cut one bulge's top away, as a union could: some point inside it is missing."""
        corner, far = BULGE_TOP_LOST
        size = (far[0] - corner[0], far[1] - corner[1], far[2] - corner[2])
        cutter = cq.Workplane('XY').box(*size, centered=False).translate(corner)

        self.assertFalse(wheel.collar_kept(self.bulges.cut(cutter), self.bulges))

    def test_collar_kept_is_false_for_a_rim_that_lost_the_foot_round(self) -> None:
        """With a bulge round of 2.2 mm, keep the bulges only up to their rounds' joint: lost."""
        with with_values({'HUB_BULGE_ROUND': LARGER_BULGE_ROUND}):
            bulges = wheel.collar()
            joint = wheel.bulge_end()[0] + LARGER_BULGE_ROUND
            below = cq.Workplane('XY').box(30.0, 30.0, joint - wheel.CUP_BACK_Z + 1.0,
                                           centered=(True, True, False))
            kept = bulges.intersect(below.translate((0.0, 0.0, wheel.CUP_BACK_Z - 1.0)))
            rim = wheel.hub().union(kept)

            self.assertFalse(wheel.collar_kept(rim, bulges))

    def test_collar_kept_is_false_for_a_rim_that_lost_the_round_below_the_notches(self) -> None:
        """Cut the bulges where their foot round falls below the notches' bottoms: lost."""
        below_notches = next(z for z in side_view()
                             if wheel.bulge_profile(z) < wheel.notch_bottom())
        cutter = cq.Workplane('XY', origin=(0.0, 0.0, below_notches)).box(
            30.0, 30.0, 20.0, centered=(True, True, False))
        rim = wheel.hub().union(self.bulges.cut(cutter))

        self.assertFalse(wheel.collar_kept(rim, self.bulges))

    def test_collar_kept_raises_a_value_error_for_notches_too_near_the_tube_to_probe(
            self) -> None:
        """Put the notches' bottoms 0.15 mm off the tube: nothing to probe below them, an error."""
        with with_values({'HUB_NOTCH_ROUND': NOTCH_ROUND_NEAR_THE_TUBE}):
            bulges = wheel.collar()
            with self.assertRaisesRegex(ValueError, 'found no point inside the bulges'):
                wheel.collar_kept(bulges, bulges)

    def test_foot_probe_heights_raises_a_value_error_for_a_foot_round_too_small(self) -> None:
        """Leave a foot round smaller than the check's 0.4 mm: an error, not an unchecked rim."""
        with with_values({'HUB_BULGE_ROUND': BULGE_ROUND_WITH_A_TOO_SMALL_FOOT}):
            with self.assertRaisesRegex(ValueError, 'too small for the check'):
                wheel.foot_probe_heights()

    def test_make_rim_raises_a_value_error_when_collar_kept_finds_a_loss(self) -> None:
        """Let collar_kept() report a loss: make_rim() stops before the bore is cut."""
        with mock.patch.object(wheel, 'collar_kept', return_value=False):
            with self.assertRaisesRegex(ValueError, "the rim's union must keep the bulges"):
                wheel.make_rim()


if __name__ == '__main__':
    unittest.main()
