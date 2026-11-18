# SPDX-FileCopyrightText: 2026 ROS Mobile Robots
# SPDX-License-Identifier: BSD-3-Clause
"""Tests of the connector shapes that the single-board computers share.

generators/board_parts.py draws a connector in its own frame: the opening faces +x, z is up,
so someone who looks into the opening has -y on the left. Run from the repository's root,
with the packages of generators/requirements.txt:

  python3 -m unittest discover -s tests
"""
from typing import List
from typing import Tuple
import unittest

import cadquery as cq

from generators import board_parts
from generators import jetson_nano_dev_kit_b01 as jetson
from generators import raspberry_pi_4b

# A tongue 12 mm wide, 1 mm thick, its tip 1 mm behind the front: (half width, z0, z1, tip)
TONGUE = (6.0, 1.0, 2.0, 1.0)
CONTACT_LENGTH = 3.2
# Ten contacts of 0.5 mm at 1 mm are 9.5 mm wide: they fit the 12 mm tongue
FITTING_ROW = (10, 1.0, 0.5, True)
# Thirteen such contacts are 12.5 mm wide: wider than the tongue
ROW_WIDER_THAN_THE_TONGUE = (13, 1.0, 0.5, True)
TONGUE_SHIFT = 3.0  # mm: a tongue and its contacts off the opening's centre
# An opening 16 mm wide and 5 mm high with its bottom at z = 10 and a 1 mm chamfer
WIDTH, HEIGHT, CHAMFER, BOTTOM = 16.0, 5.0, 1.0, 10.0
WALL = 0.2
DEPTH = 6.0
INSIDE_THE_CHAMFER = 0.3  # mm from the corner, along y and z: a point that the chamfer cuts off
SECTION_X = -0.1  # mm behind the front face


def contact_centres(parts: List[board_parts.Part]) -> List[float]:
    """Return the y of each contact's centre, sorted."""
    boxes = [shape.BoundingBox() for _, shape in parts]
    return sorted(round((box.ymin + box.ymax) / 2, 6) for box in boxes)


def is_solid(shapes: List[cq.Shape], y: float, z: float) -> bool:
    """Return whether one of the shapes holds the point (y, z) just behind the front face."""
    return any(shape.isInside(cq.Vector(SECTION_X, y, z), 1e-7) for shape in shapes)


class ContactRowsTest(unittest.TestCase):
    """contact_rows() puts a row of contacts on a tongue, centred with the tongue."""

    def test_contact_rows_keeps_every_contact_on_a_tongue_off_the_centre(self) -> None:
        """Shift a 12 mm tongue and its ten contacts by 3 mm: all ten, centred on 3 mm."""
        parts = board_parts.contact_rows((FITTING_ROW,), TONGUE, CONTACT_LENGTH, TONGUE_SHIFT)

        centres = contact_centres(parts)

        self.assertEqual(len(centres), FITTING_ROW[0])
        self.assertAlmostEqual(sum(centres) / len(centres), TONGUE_SHIFT)

    def test_contact_rows_raises_a_value_error_for_a_row_wider_than_the_tongue(self) -> None:
        """Ask for thirteen contacts on the 12 mm tongue: an error with both widths."""
        with self.assertRaisesRegex(ValueError, "12.5 mm wide: more than the tongue's 12 mm"):
            board_parts.contact_rows((ROW_WIDER_THAN_THE_TONGUE,), TONGUE, CONTACT_LENGTH)


class OpeningTest(unittest.TestCase):
    """An opening's outline is the room for the plug, and its liner lies round it."""

    def corner(self, side: int) -> Tuple[float, float]:
        """Return a point near the lower corner on the side (-1 left, +1 right) of the opening."""
        return (side * (WIDTH / 2 - INSIDE_THE_CHAMFER), BOTTOM + INSIDE_THE_CHAMFER)

    def test_lower_left_chamfer_is_on_the_viewers_left(self) -> None:
        """Key the opening at its lower left: metal near the -y corner, room near the +y one."""
        outline = board_parts.chamfered_rectangle(WIDTH, HEIGHT, CHAMFER, BOTTOM, 'lower_left')
        _, liner = board_parts.opening_with_liner(outline, DEPTH, WALL)
        body = board_parts.box(-DEPTH - 1, -WIDTH, BOTTOM - 2, 0.0, WIDTH, BOTTOM + HEIGHT + 2)
        seat, _ = board_parts.opening_with_liner(outline, DEPTH, WALL)
        shapes = [body.cut(seat), liner]

        self.assertTrue(is_solid(shapes, *self.corner(-1)))
        self.assertFalse(is_solid(shapes, *self.corner(+1)))

    def test_chamfered_rectangle_raises_a_value_error_for_an_unknown_corner(self) -> None:
        """Ask for the corners 'one': an error names the word."""
        with self.assertRaisesRegex(ValueError, "'one'"):
            board_parts.chamfered_rectangle(WIDTH, HEIGHT, CHAMFER, BOTTOM, 'one')

    def test_liner_leaves_the_outline_clear(self) -> None:
        """Line a plain opening: room just inside the outline, the liner's wall just outside."""
        outline = [(-WIDTH / 2, BOTTOM), (WIDTH / 2, BOTTOM), (WIDTH / 2, BOTTOM + HEIGHT),
                   (-WIDTH / 2, BOTTOM + HEIGHT)]
        _, liner = board_parts.opening_with_liner(outline, DEPTH, WALL)
        middle = BOTTOM + HEIGHT / 2
        half_wall = WALL / 2

        self.assertFalse(is_solid([liner], WIDTH / 2 - half_wall, middle))
        self.assertTrue(is_solid([liner], WIDTH / 2 + half_wall, middle))
        self.assertFalse(is_solid([liner], 0.0, BOTTOM + half_wall))
        self.assertTrue(is_solid([liner], 0.0, BOTTOM - half_wall))


class UnknownMaterialTest(unittest.TestCase):
    """The boards' sub_part() refuses a piece of a material that the board doesn't have."""

    def test_jetson_sub_part_raises_a_value_error_for_a_misspelled_material(self) -> None:
        """Give the Jetson a piece of "g0ld": an error names the sub-part and the name."""
        cube = board_parts.box(0.0, 0.0, 0.0, 1.0, 1.0, 1.0)

        with self.assertRaisesRegex(ValueError, "sub-part 'fixture'.*g0ld"):
            jetson.sub_part('fixture', [('gold', cube), ('g0ld', cube)])

    def test_raspberry_pi_sub_part_raises_a_value_error_for_a_misspelled_material(self) -> None:
        """Give the Raspberry Pi a piece of "g0ld": an error names the sub-part and the name."""
        cube = board_parts.box(0.0, 0.0, 0.0, 1.0, 1.0, 1.0)

        with self.assertRaisesRegex(ValueError, "sub-part 'fixture'.*g0ld"):
            raspberry_pi_4b.sub_part('fixture', [('gold', cube), ('g0ld', cube)])


if __name__ == '__main__':
    unittest.main()
