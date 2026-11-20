# SPDX-FileCopyrightText: 2026 ROS Mobile Robots
# SPDX-License-Identifier: BSD-3-Clause
"""Tests of the room between the caster's fork and its wheel.

The fork's two arms hold the wheel's hub between them, and the axle runs through the wheel's
bore. The generator has to refuse a fork whose arms would stand inside the hub, and the fork
and the wheel that it builds must not share any volume where DiffBot's joints place them. Run
from the repository's root, with the packages of generators/requirements.txt:

  python3 -m unittest discover -s tests
"""
from typing import Dict
import unittest
from unittest import mock

import cadquery as cq

from generators import swivel_caster_30mm as caster

# An inner half width of the fork below the hub's half width of 7.8 mm
HALF_WIDTH_INSIDE_THE_HUB = 7.5
# The wheel's bore is 0.1 mm wider in radius than the axle (WHEEL_BORE)
AXLE_CLEARANCE = 0.1
# The hub's side faces stand this far from the flat arms' inner faces
ARM_CLEARANCE = caster.FORK_INNER_HALF_WIDTH - caster.WHEEL_HALF_WIDTH
DISTANCE_TOLERANCE = 0.001  # mm


class ForkAndWheelTest(unittest.TestCase):
    """The fork and the wheel, each in its own mesh, fit together on the axle."""

    fork: Dict[str, cq.Workplane]
    wheel: cq.Workplane

    @classmethod
    def setUpClass(cls) -> None:
        """Build the fork and the wheel once, the wheel placed on the fork's axle.

        DiffBot's wheel joint puts the wheel's frame at the axle, which is at (AXLE_X, 0,
        AXLE_Z) in the fork's frame.
        """
        cls.fork = caster.make_fork()
        cls.wheel = caster.make_wheel()['wheel'].translate((caster.AXLE_X, 0.0, caster.AXLE_Z))

    def distance(self, material: str) -> float:
        """Return how far the fork's solid of the material is from the wheel, in millimetres."""
        fork_shape = cq.Compound.makeCompound(self.fork[material].solids().vals())
        wheel_shape = cq.Compound.makeCompound(self.wheel.solids().vals())
        return float(fork_shape.distance(wheel_shape))

    def test_make_fork_raises_a_value_error_for_arms_inside_the_wheels_hub(self) -> None:
        """Make the fork narrower than the hub: an error names both widths."""
        with mock.patch.object(caster, 'FORK_INNER_HALF_WIDTH', HALF_WIDTH_INSIDE_THE_HUB):
            with self.assertRaisesRegex(ValueError, "no room for the wheel's hub"):
                caster.make_fork()

    def test_forks_steel_and_the_wheel_share_no_volume(self) -> None:
        """Intersect the fork's steel with the placed wheel: nothing, the arms clear the hub."""
        self.assertEqual(caster.volume(self.fork['steel'].intersect(self.wheel)), 0.0)
        self.assertAlmostEqual(self.distance('steel'), ARM_CLEARANCE, delta=DISTANCE_TOLERANCE)

    def test_axle_and_the_wheel_share_no_volume(self) -> None:
        """Intersect the axle with the placed wheel: nothing, the bore is wider than the axle."""
        self.assertEqual(caster.volume(self.fork['axle'].intersect(self.wheel)), 0.0)
        self.assertAlmostEqual(self.distance('axle'), AXLE_CLEARANCE, delta=DISTANCE_TOLERANCE)


if __name__ == '__main__':
    unittest.main()
