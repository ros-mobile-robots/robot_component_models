# SPDX-FileCopyrightText: 2026 ROS Mobile Robots
# SPDX-License-Identifier: BSD-3-Clause
"""Tests of how deep the single-board computers' ports are.

A receptacle's drawing gives how far behind the front the insulator's face lies. A plug goes
in that far. Each test looks into an assembled connector along a line beside the tongue and
measures where the first solid begins: at the drawing's depth. A line through the tongue
meets the tongue's tip first. Run from the repository's root, with the packages of
generators/requirements.txt:

  python3 -m unittest discover -s tests
"""
from typing import List
from typing import Sequence
from typing import Tuple
import unittest

import cadquery as cq

from generators import board_parts
from generators import jetson_nano_dev_kit_b01 as jetson
from generators import raspberry_pi_4b as pi

IN_FRONT = 0.5  # mm: a line of sight starts this far in front of a connector's front face
REACH = 20.0    # mm: how far a line of sight goes
STEP = 0.02     # mm: the first solid is searched in steps of this size, then narrowed down
INTO_A_CONNECTOR = cq.Vector(-1.0, 0.0, 0.0)  # in a connector's own frame, which opens to +x
TOWARDS_PLUS_Y = cq.Vector(0.0, 1.0, 0.0)     # into a port at a board's -y or -v edge


def solids(pieces: Sequence[Tuple[str, object]]) -> List[cq.Shape]:
    """Return the CAD shapes of a connector's pieces."""
    return [shape for _, shape in pieces if isinstance(shape, cq.Shape)]


def first_solid(shapes: Sequence[cq.Shape], start: cq.Vector, direction: cq.Vector) -> float:
    """Return how far from the start, along the direction, the first solid begins (mm).

    1. Walk along the line in steps until a point lies in one of the shapes.
    2. Halve the last step twelve times, to 0.00001 mm.
    Fail if the line meets no shape within its reach.
    """
    boxes = [(shape, shape.BoundingBox()) for shape in shapes]

    def is_solid(distance: float) -> bool:
        point = start + direction * distance
        return any(box.xmin <= point.x <= box.xmax and box.ymin <= point.y <= box.ymax
                   and box.zmin <= point.z <= box.zmax and shape.isInside(point, 1e-7)
                   for shape, box in boxes)

    distance = 0.0
    while not is_solid(distance):
        distance += STEP
        if distance > REACH:
            raise AssertionError(f'no solid within {REACH} mm of {start.toTuple()}')
    free, solid = distance - STEP, distance
    for _ in range(12):
        middle = (free + solid) / 2
        free, solid = (free, middle) if is_solid(middle) else (middle, solid)
    return solid


class PortDepthTest(unittest.TestCase):
    """The depth of each port, from its front face to its insulator's face."""

    def assert_depth(self, shapes: Sequence[cq.Shape], start: cq.Vector, direction: cq.Vector,
                     depth: float) -> None:
        """Fail unless the first solid on the line lies the depth behind the front face."""
        self.assertAlmostEqual(first_solid(shapes, start, direction), IN_FRONT + depth,
                               places=3)

    def test_jetson_hdmi_port_is_as_deep_as_its_drawing_says(self) -> None:
        """Look into J6's HDMI port beside the tongue: the insulator is 6.80 mm in."""
        width, height, bottom = jetson.HDMI_OPENING
        tongue_width = jetson.HDMI_TONGUE[0]
        beside_the_tongue = (tongue_width / 2 + width / 2) / 2
        start = cq.Vector(IN_FRONT, beside_the_tongue, bottom + height / 2)

        self.assert_depth(solids(jetson.hdmi_dp_stack()), start, INTO_A_CONNECTOR,
                          jetson.HDMI_INSULATOR)

    def test_jetson_displayport_is_as_deep_as_its_drawing_says(self) -> None:
        """Look into J6's DisplayPort beside the tongue: the insulator is 8.88 mm in."""
        width, _, bottom = jetson.DP_OPENING
        tongue_width, _, _, centre = jetson.DP_TONGUE
        # The tongue lies off the middle towards +y, so the wider gap is at -y
        beside_the_tongue = (jetson.DP_TONGUE_OFFSET - tongue_width / 2 - width / 2) / 2
        start = cq.Vector(IN_FRONT, beside_the_tongue, bottom + centre)

        self.assert_depth(solids(jetson.hdmi_dp_stack()), start, INTO_A_CONNECTOR,
                          jetson.DP_INSULATOR)

    def test_jetson_hdmi_and_displayport_tongues_are_as_long_as_their_drawings_say(self) -> None:
        """Look at each of J6's tongues: its tip lies 4.80 mm in front of the insulator."""
        _, height, bottom = jetson.HDMI_OPENING
        hdmi = cq.Vector(IN_FRONT, 0.0, bottom + height / 2)
        _, _, _, centre = jetson.DP_TONGUE
        displayport = cq.Vector(IN_FRONT, jetson.DP_TONGUE_OFFSET,
                                jetson.DP_OPENING[2] + centre)
        shapes = solids(jetson.hdmi_dp_stack())

        hdmi_tip = first_solid(shapes, hdmi, INTO_A_CONNECTOR) - IN_FRONT
        displayport_tip = first_solid(shapes, displayport, INTO_A_CONNECTOR) - IN_FRONT

        self.assertAlmostEqual(jetson.HDMI_INSULATOR - hdmi_tip, 4.8, places=3)
        self.assertAlmostEqual(jetson.DP_INSULATOR - displayport_tip, 4.8, places=3)

    def test_usb_a_ports_are_as_deep_as_their_drawing_says(self) -> None:
        """Look into both ports of a USB stack beside the tongue: the rear is 10.25 mm in."""
        u0, u1, front, back, height = jetson.USB_STACKS[0]
        shapes = solids(board_parts.usb_a_stack(u1 - u0, back - front, height, 'usb3_blue'))
        opening_width = board_parts.USB_A_OPENING[0]
        tongue_width = board_parts.USB_A_TONGUE[0]
        beside_the_tongue = (tongue_width / 2 + opening_width / 2) / 2

        for bottom in board_parts.USB_A_PORT_BOTTOMS:
            start = cq.Vector(IN_FRONT, beside_the_tongue, bottom + 1.0)
            self.assert_depth(shapes, start, INTO_A_CONNECTOR, board_parts.USB_A_PORT_DEPTH)

    def test_jetson_micro_usb_port_is_as_deep_as_its_drawing_says(self) -> None:
        """Look into the micro-USB port beside the tongue: the insulator is 3.50 mm in."""
        u0, u1, front, _, _ = jetson.MICRO_USB
        width, height, _, bottom = jetson.MICRO_USB_OPENING
        tongue_half_width = jetson.MICRO_USB_TONGUE[0]
        beside_the_tongue = (tongue_half_width + width / 2) / 2
        start = cq.Vector((u0 + u1) / 2 + beside_the_tongue, front - IN_FRONT,
                          jetson.TOP + bottom + 0.75 * height)

        self.assert_depth(solids(jetson.micro_usb()), start, TOWARDS_PLUS_Y,
                          jetson.MICRO_USB_DEPTH)

    def test_raspberry_pi_micro_hdmi_port_is_as_deep_as_the_generator_says(self) -> None:
        """Look into a micro-HDMI port beside the tongue: the insulator is 5.0 mm in."""
        centre_x, front, back = pi.MICRO_HDMI[0]
        width, height, bottom = pi.MICRO_HDMI_OPENING
        tongue_half_width = pi.MICRO_HDMI_TONGUE[0]
        beside_the_tongue = (tongue_half_width + width / 2) / 2
        start = cq.Vector(centre_x + beside_the_tongue, front - IN_FRONT,
                          pi.TOP + bottom + 0.75 * height)

        self.assert_depth(solids(pi.micro_hdmi(centre_x, front, back)), start, TOWARDS_PLUS_Y,
                          pi.MICRO_HDMI_DEPTH)

    def test_raspberry_pi_usb_c_port_is_as_deep_as_its_drawing_says(self) -> None:
        """Look into the USB-C port beside the tongue: the insulator is 4.65 mm in."""
        centre_x, front, _, _, height = pi.USB_C
        opening_width = pi.USB_C_OPENING[0]
        tongue_width = pi.USB_C_TONGUE[0]
        # Between the metal plate at the tongue's edge and the opening's rounded end
        beside_the_tongue = (tongue_width / 2 + 0.05 + opening_width / 2) / 2
        start = cq.Vector(centre_x + beside_the_tongue, front - IN_FRONT, pi.TOP + height / 2)

        self.assert_depth(solids(pi.usb_c()), start, TOWARDS_PLUS_Y, pi.USB_C_INSULATOR)


if __name__ == '__main__':
    unittest.main()
