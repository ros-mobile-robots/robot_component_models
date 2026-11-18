# SPDX-FileCopyrightText: 2026 ROS Mobile Robots
# SPDX-License-Identifier: BSD-3-Clause
"""Tests of the Jetson Nano kit's connectors and of the parts that fit into each other.

The HDMI and DisplayPort stack J6 is checked in its own frame, just behind its front face:
the opening faces +x and z is up, so someone who looks into a port has -y on the left. The
module, its connector, its standoffs and their screws are checked where the model places
them: a part that sits in another must not share any volume with it. Run from the
repository's root, with the packages of generators/requirements.txt:

  python3 -m unittest discover -s tests
"""
from typing import List
import unittest

import cadquery as cq

from generators import jetson_nano_dev_kit_b01 as jetson

SECTION_X = -0.1  # mm behind J6's front face, in front of the tongues
STEP = 0.05       # mm: how far inside or outside an opening's edge a point is probed
# The DisplayPort key: a point that the 1 mm chamfer of a lower corner cuts off
KEY_FROM_THE_SIDE, KEY_ABOVE_THE_BOTTOM = 0.45, 0.35


def volume(shape: cq.Shape) -> float:
    """Return the shape's volume in cubic millimetres, 0 for an empty shape."""
    return float(sum(solid.Volume() for solid in shape.Solids()))


class J6PortsTest(unittest.TestCase):
    """The HDMI and the DisplayPort opening of the stack J6."""

    shapes: List[cq.Shape]

    @classmethod
    def setUpClass(cls) -> None:
        """Build the stack once."""
        cls.shapes = [shape for _, shape in jetson.hdmi_dp_stack()]

    def is_solid(self, y: float, z: float) -> bool:
        """Return whether a part of the stack holds the point just behind the front face."""
        return any(shape.isInside(cq.Vector(SECTION_X, y, z), 1e-7) for shape in self.shapes)

    def assert_clear_opening(self, width: float, height: float, bottom: float) -> None:
        """Fail unless the port is open just inside the size and closed just outside it."""
        middle = bottom + height / 2
        for side in (-1, 1):
            self.assertFalse(self.is_solid(side * (width / 2 - STEP), middle))
            self.assertTrue(self.is_solid(side * (width / 2 + STEP), middle))
        self.assertFalse(self.is_solid(0.0, bottom + STEP))
        self.assertTrue(self.is_solid(0.0, bottom - STEP))
        self.assertFalse(self.is_solid(0.0, bottom + height - STEP))
        self.assertTrue(self.is_solid(0.0, bottom + height + STEP))

    def test_hdmi_port_has_the_clear_opening_of_its_drawing(self) -> None:
        """Probe the HDMI port: 14.00 x 4.55 mm are clear, as HDMI_OPENING says."""
        self.assert_clear_opening(*jetson.HDMI_OPENING)

    def test_displayport_has_the_clear_opening_of_its_drawing(self) -> None:
        """Probe the DisplayPort: 16.10 x 4.76 mm are clear, as DP_OPENING says."""
        self.assert_clear_opening(*jetson.DP_OPENING)

    def test_displayport_key_is_at_the_lower_left(self) -> None:
        """Probe the DisplayPort's lower corners: metal at the left one, room at the right."""
        width, _, bottom = jetson.DP_OPENING
        z = bottom + KEY_ABOVE_THE_BOTTOM

        self.assertTrue(self.is_solid(-(width / 2 - KEY_FROM_THE_SIDE), z))
        self.assertFalse(self.is_solid(width / 2 - KEY_FROM_THE_SIDE, z))

    def test_displayport_tongue_lies_away_from_the_key(self) -> None:
        """Probe both sides behind the tongue's tip: the tongue reaches further to the right.

        The key is at the lower left, and the tongue's middle lies off the opening's middle,
        away from the key.
        """
        _, _, bottom = jetson.DP_OPENING
        tongue_width, _, tip, centre = jetson.DP_TONGUE
        behind_the_tip = cq.Vector(-tip - 0.5, 0.0, bottom + centre)
        beside = tongue_width / 2 + jetson.DP_TONGUE_OFFSET / 2

        on_the_right = any(shape.isInside(behind_the_tip + cq.Vector(0.0, beside, 0.0), 1e-7)
                           for shape in self.shapes)
        on_the_left = any(shape.isInside(behind_the_tip + cq.Vector(0.0, -beside, 0.0), 1e-7)
                          for shape in self.shapes)

        self.assertTrue(on_the_right)
        self.assertFalse(on_the_left)


class ModuleSeatTest(unittest.TestCase):
    """The module in its SO-DIMM connector, on its standoffs, held by screws."""

    def test_module_and_its_connector_share_no_volume(self) -> None:
        """Intersect the module's board with each piece of the connector: nothing."""
        plate = jetson.module_plate()

        shared = [volume(shape.intersect(plate)) for _, shape in jetson.sodimm_connector()
                  if isinstance(shape, cq.Shape)]

        self.assertEqual(max(shared), 0.0)

    def test_connectors_key_rib_stands_in_the_modules_notch(self) -> None:
        """Probe the middle of the module's key notch: the connector's rib is there."""
        key_u = jetson.MODULE_KEY[0]
        in_the_notch = cq.Vector(key_u, jetson.MODULE[1] + jetson.MODULE_KEY_DEPTH / 2,
                                 sum(jetson.MODULE_Z) / 2)
        body = jetson.sodimm_connector()[0][1]
        assert isinstance(body, cq.Shape)

        self.assertTrue(body.isInside(in_the_notch, 1e-7))
        self.assertFalse(jetson.module_plate().isInside(in_the_notch, 1e-7))

    def test_standoffs_and_their_screws_share_no_volume(self) -> None:
        """Intersect each standoff with its screw from below and from above: nothing."""
        mounting = jetson.module_mounting()
        shared = []
        for (_, standoff), (_, upper), (_, lower) in zip(mounting['module_standoffs'],
                                                         mounting['module_screws'],
                                                         jetson.carrier_screws()):
            assert isinstance(standoff, cq.Shape)
            shared += [volume(standoff.intersect(upper)), volume(standoff.intersect(lower))]

        self.assertEqual(shared, [0.0] * 4)

    def test_screws_reach_into_the_standoffs(self) -> None:
        """Measure each screw's shank: it ends inside its standoff, by the shank's reach."""
        reach = jetson.STANDOFF_SCREW_SHANK[1]
        mounting = jetson.module_mounting()
        upper_ends = [shape.BoundingBox().zmin for _, shape in mounting['module_screws']
                      if isinstance(shape, cq.Shape)]
        lower_ends = [shape.BoundingBox().zmax for _, shape in jetson.carrier_screws()
                      if isinstance(shape, cq.Shape)]

        for end in upper_ends:
            self.assertAlmostEqual(end, jetson.MODULE_Z[0] - reach, places=3)
        for end in lower_ends:
            self.assertAlmostEqual(end, jetson.TOP + reach - 0.01, places=3)

    def test_module_board_and_its_screws_share_no_volume(self) -> None:
        """Intersect the module's board with the screws through its holes: nothing."""
        plate = jetson.module_plate()

        shared = [volume(shape.intersect(plate))
                  for _, shape in jetson.module_mounting()['module_screws']
                  if isinstance(shape, cq.Shape)]

        self.assertEqual(shared, [0.0, 0.0])


if __name__ == '__main__':
    unittest.main()
