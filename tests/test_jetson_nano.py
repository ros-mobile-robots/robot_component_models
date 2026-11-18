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


if __name__ == '__main__':
    unittest.main()
