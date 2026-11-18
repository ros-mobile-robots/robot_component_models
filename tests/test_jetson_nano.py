# SPDX-FileCopyrightText: 2026 ROS Mobile Robots
# SPDX-License-Identifier: BSD-3-Clause
"""Tests of the Jetson Nano kit's connectors and of the parts that fit into each other.

The HDMI and DisplayPort stack J6 is checked in its own frame, just behind its front face:
the opening faces +x and z is up, so someone who looks into a port has -y on the left. The
module, its connector, its standoffs and their screws are checked where the model places
them: a part that sits in or on another must not share any volume with it, and that includes
the gold pads on the boards' faces. tests/test_port_depths.py measures how deep the ports
are. Run from the repository's root, with the packages of generators/requirements.txt:

  python3 -m unittest discover -s tests
"""
from typing import List
from typing import Sequence
from typing import Tuple
import unittest

import cadquery as cq

from generators import board_parts
from generators import jetson_nano_dev_kit_b01 as jetson

SECTION_X = -0.1  # mm behind J6's front face, in front of the tongues
STEP = 0.05       # mm: how far inside or outside an opening's edge a point is probed
# The DisplayPort key: a point that the 1 mm chamfer of a lower corner cuts off
KEY_FROM_THE_SIDE, KEY_ABOVE_THE_BOTTOM = 0.45, 0.35


# A screw's head rests on a ring between its shank and its rim. This radius lies on that
# ring for the screws of both ends of a standoff, and inside the gold pads round their holes
ON_THE_HEADS_SEAT = 1.7
BESIDE_A_FACE = 0.01  # mm above or below a face: a point just inside one of two stacked parts


def volume(shape: cq.Shape) -> float:
    """Return the shape's volume in cubic millimetres, 0 for an empty shape."""
    return float(sum(solid.Volume() for solid in shape.Solids()))


def solids(pieces: Sequence[Tuple[str, object]]) -> List[cq.Shape]:
    """Return the CAD shapes of a list of pieces."""
    return [shape for _, shape in pieces if isinstance(shape, cq.Shape)]


def gold_count(parts: Sequence[Tuple[str, object]]) -> int:
    """Return how many of the parts are gold: a connector's contacts."""
    return sum(1 for material, _ in parts if material == 'gold')


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

    def test_stack_has_the_contacts_of_both_receptacles(self) -> None:
        """Count the stack's contacts: HDMI's 19 and DisplayPort's 20."""
        self.assertEqual(gold_count(jetson.hdmi_dp_stack()), 19 + 20)

    def test_contact_count_stays_when_the_tongue_moves_sideways(self) -> None:
        """Build each receptacle's inside with its tongue centred and off the centre.

        The number of contacts belongs to the receptacle: HDMI has 19, DisplayPort 20,
        wherever the tongue lies in the opening.
        """
        width, height, bottom = jetson.HDMI_OPENING
        hdmi_outline = board_parts.chamfered_rectangle(width, height, 1.3, bottom)
        width, height, bottom = jetson.DP_OPENING
        displayport_outline = board_parts.chamfered_rectangle(width, height, 1.0, bottom,
                                                              'lower_left')
        tongue_width, thickness, tip, centre = jetson.DP_TONGUE

        for offset in (0.0, jetson.DP_TONGUE_OFFSET):
            hdmi = jetson.hdmi_inside(hdmi_outline, jetson.HDMI_INSULATOR, jetson.HDMI_TONGUE,
                                      jetson.HDMI_OPENING[2] + jetson.HDMI_OPENING[1] / 2,
                                      jetson.HDMI_CONTACT_ROWS, offset)
            displayport = jetson.hdmi_inside(displayport_outline, jetson.DP_INSULATOR,
                                             (tongue_width, thickness, tip), bottom + centre,
                                             jetson.DP_CONTACT_ROWS, offset)

            self.assertEqual(gold_count(hdmi), 19)
            self.assertEqual(gold_count(displayport), 20)


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

    def test_module_screws_share_no_volume_with_the_module_and_its_pads(self) -> None:
        """Intersect the screws from above with the module's board and its gold pads: nothing."""
        module = [jetson.module_plate()] + solids(jetson.module_pads())

        shared = [volume(screw.intersect(part))
                  for screw in solids(jetson.module_mounting()['module_screws'])
                  for part in module]

        self.assertEqual(shared, [0.0] * len(shared))
        self.assertEqual(len(shared), 2 * 3)

    def test_carrier_screws_share_no_volume_with_the_carrier_and_its_pads(self) -> None:
        """Intersect the screws from below with the carrier's board and its gold pads: nothing."""
        carrier = [jetson.carrier_plate()] + solids(jetson.carrier_pads())

        shared = [volume(screw.intersect(part)) for screw in solids(jetson.carrier_screws())
                  for part in carrier]

        self.assertEqual(shared, [0.0] * len(shared))
        self.assertEqual(len(shared), 2 * (1 + 2 + 2 * len(jetson.J50_PINS)))

    def test_module_screws_heads_sit_on_the_gold_pads(self) -> None:
        """Probe above and below each pad's top face: the head above, the pad below."""
        pad_top = jetson.MODULE_Z[1] + jetson.PAD_THICKNESS
        screws = solids(jetson.module_mounting()['module_screws'])
        pads = solids(jetson.module_pads())

        for (u, v), screw, pad in zip(jetson.MODULE_STANDOFFS, screws, pads):
            on_the_seat = cq.Vector(u + ON_THE_HEADS_SEAT, v, pad_top)
            self.assertTrue(screw.isInside(on_the_seat + cq.Vector(0, 0, BESIDE_A_FACE), 1e-7))
            self.assertTrue(pad.isInside(on_the_seat - cq.Vector(0, 0, BESIDE_A_FACE), 1e-7))

    def test_carrier_screws_heads_sit_on_the_gold_pads(self) -> None:
        """Probe below and above each pad's bottom face: the head below, the pad above."""
        pad_bottom = -jetson.PAD_THICKNESS
        screws = solids(jetson.carrier_screws())
        pads = solids(jetson.carrier_pads())[:len(jetson.MODULE_STANDOFFS)]

        for (u, v), screw, pad in zip(jetson.MODULE_STANDOFFS, screws, pads):
            on_the_seat = cq.Vector(u + ON_THE_HEADS_SEAT, v, pad_bottom)
            self.assertTrue(screw.isInside(on_the_seat - cq.Vector(0, 0, BESIDE_A_FACE), 1e-7))
            self.assertTrue(pad.isInside(on_the_seat + cq.Vector(0, 0, BESIDE_A_FACE), 1e-7))


if __name__ == '__main__':
    unittest.main()
