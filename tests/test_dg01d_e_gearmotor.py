# SPDX-FileCopyrightText: 2026 ROS Mobile Robots
# SPDX-License-Identifier: BSD-3-Clause
"""Tests of generators/dg01d_e_gearmotor.py: its parameter checks and the built mesh's sizes.

The checks of the parameters have to refuse values that don't fit together, such as a shaft
whose flats are wider than the shaft. The built mesh reaches from the measured encoder cover's
end to the drawing's tab end, has its output shaft where the parameters put it, and the frame
that the robots' joints use. The motor housing, the neck and the encoder cover have the round
shapes of the motor. The expected values come from the generator's parameter block, never from
the generated files.
Run from the repository's root, with the packages of generators/requirements.txt:

  python3 -m unittest discover -s tests
"""
from typing import Dict
import unittest
from unittest import mock

import cadquery as cq
import numpy as np

from generators import dg01d_e_gearmotor as motor
from generators.mesh_export import Model

CUBE_TRIANGLES = 12  # two per face
# A neck 0.5 mm thinner than the one that the housing's chamfer runs down to
THINNER_NECK = motor.NECK_THICKNESS - 0.5
# Flats wider apart than the shaft's diameter
FLATS_WIDER_THAN_THE_SHAFT = motor.SHAFT_DIAMETER + 0.1
# A connector centre that puts the connector's back half past the window's end at x = -34.475
CONNECTOR_PAST_THE_WINDOW = motor.WINDOW_X.low + motor.CONNECTOR_DEPTH / 4
# A fin whose foot reaches past the neck's start, onto the motor housing's chamfer
FIN_ON_THE_CHAMFER = motor.HOUSING_START_X + motor.HOUSING_LENGTH + motor.FIN_BASE_LENGTH - 0.5
SIZE_TOLERANCE = 1e-4  # mm: the vertices are snapped to 10 nm, the files keep float32
# A screw centre 1 mm past the gearbox's end at x = 35.175 mm
SCREW_PAST_THE_END = motor.GEARBOX_END_X + 1.0
# A small hole in the corner outside the gearbox's R5 round: inside the block's rectangle,
# but 5.2 mm from the round's centre at (30.175, 6.2) mm
HOLE_OUTSIDE_THE_ROUND = motor.Hole(34.0, 9.5, 1.4, 2.0)
# The -y recess reaching to x = 40 mm, past the gearbox's end
RECESS_PAST_THE_END = motor.Recess(motor.Span(motor.BACK_RECESS.x.low, 40.0),
                                   motor.BACK_RECESS.half_height, motor.BACK_RECESS.depth,
                                   motor.BACK_RECESS.corner)
# How far beside a hole's edge a point lies, inside the material around the hole
BESIDE_THE_EDGE = 0.3  # mm
# How far below and above a fin's top a point lies: inside the fin, and outside it
AROUND_THE_TOP = 0.2  # mm
# Half the width of the motor housing's and the neck's flats
HOUSING_FLAT = motor.flat_half_width(motor.HOUSING_HEIGHT, motor.HOUSING_THICKNESS)
NECK_FLAT = motor.flat_half_width(motor.NECK_HEIGHT, motor.NECK_THICKNESS)
# A connector's box 0.2 mm wider than the motor housing's -y flat
BOX_WIDER_THAN_THE_FLAT = 2 * HOUSING_FLAT + 0.2
# Encoder cover corners larger than half the round part's height
CORNERS_PAST_HALF_THE_HEIGHT = motor.ENCODER_HEIGHT / 2 + 0.5
# Corners that leave a flat strip of half the shortest straight piece between them
CORNERS_WITH_A_SLIVER = (motor.ENCODER_HEIGHT - motor.SHORTEST_STRAIGHT / 2) / 2
# The outline's values that leave exactly the shortest straight piece: the round part's +-z
# faces past the corners, the flat strip between the corners, the box's faces. Rounded to six
# decimal places in millimetres, as one writes them (8.01, 9.595 and 14.98 mm), so that
# subtracting them in the checks leaves a little less than SHORTEST_STRAIGHT
SHORTEST_PIECES = {
    'ENCODER_ROUND_LENGTH': round(motor.ENCODER_TOP_EDGE_ROUND + motor.SHORTEST_STRAIGHT, 6),
    'ENCODER_TOP_EDGE_ROUND': round((motor.ENCODER_HEIGHT - motor.SHORTEST_STRAIGHT) / 2, 6),
    'ENCODER_STEP_LENGTH': round(motor.ENCODER_DEPTH - motor.ENCODER_ROUND_LENGTH
                                 - motor.ENCODER_BOTTOM_EDGE_ROUND - motor.SHORTEST_STRAIGHT, 6),
}
# An S-curve that ends where the rounds of the box's -y end start: no straight box faces
S_CURVE_TO_THE_END_ROUNDS = (motor.ENCODER_DEPTH - motor.ENCODER_ROUND_LENGTH
                             - motor.ENCODER_BOTTOM_EDGE_ROUND)
# A window as wide as the connector's box
WINDOW_AS_WIDE_AS_THE_BOX = motor.CONNECTOR_BOX_HEIGHT / 2
# A window that starts 0.1 mm from the cover's end, where the board around it, OVERLAP wider,
# would stand out of the end face
WINDOW_AT_THE_END = motor.Span(motor.ENCODER_END_X + motor.OVERLAP / 3, motor.WINDOW_X.high)
# A window whose board starts 0.01 mm inside the cover's end face and reaches 0.11 mm short of
# the box's sides (a half height of 8.00 mm): the board's corner lies 0.35 mm from the centre
# of the end face's 0.3 mm round, outside the cover
WINDOW_IN_THE_END_ROUND = motor.Span(round(motor.ENCODER_END_X + motor.OVERLAP + 0.01, 6),
                                     motor.WINDOW_X.high)
WINDOW_NEAR_THE_SIDES = round(motor.CONNECTOR_BOX_HEIGHT / 2 - motor.OVERLAP - 0.11, 6)
# A fin 0.5 mm taller (z) than the neck's flat
FIN_TALLER_THAN_THE_FLAT = 2 * NECK_FLAT + 0.5
# Places along x in the middle of the motor housing, of the neck between its fins and the
# gearbox, and of the encoder cover's part at the motor's end
HOUSING_MIDDLE_X = motor.HOUSING_START_X + motor.HOUSING_LENGTH / 2
NECK_MIDDLE_X = (motor.FIN_FRONT_X + motor.GEARBOX_BACK_X) / 2
COVER_END_MIDDLE_X = (motor.ENCODER_END_X + motor.HOUSING_START_X) / 2
# How far inside or outside a round outline a point lies
OFF_THE_OUTLINE = 0.3  # mm


def cube() -> cq.Workplane:
    """Return a cube with an edge of 1 mm."""
    return cq.Workplane('XY').box(1.0, 1.0, 1.0)


class ParameterChecksTest(unittest.TestCase):
    """The parameter checks accept the generator's values and refuse values that don't fit."""

    def test_check_parameters_accepts_the_generators_values(self) -> None:
        """Run all checks with the parameter block as it is: no error."""
        motor.check_parameters()

    def test_check_housing_chamfer_meets_neck_raises_a_value_error_for_a_thinner_neck(
            self) -> None:
        """Make the neck thinner than the chamfer's end: an error names both sizes."""
        with mock.patch.object(motor, 'NECK_THICKNESS', THINNER_NECK):
            with self.assertRaisesRegex(ValueError,
                                        f'the chamfer is {motor.HOUSING_CHAMFER:.3f} mm'):
                motor.check_housing_chamfer_meets_neck()

    def test_check_shaft_raises_a_value_error_for_flats_wider_than_the_shaft(self) -> None:
        """Put the flats further apart than the shaft's diameter: an error."""
        with mock.patch.object(motor, 'SHAFT_FLATS', FLATS_WIDER_THAN_THE_SHAFT):
            with self.assertRaisesRegex(ValueError, 'bore < flats < diameter < collar'):
                motor.check_shaft()

    def test_check_connector_inside_window_raises_a_value_error_past_the_window(self) -> None:
        """Move the connector so that its back half leaves the window: an error."""
        with mock.patch.object(motor, 'CONNECTOR_CENTRE_X', CONNECTOR_PAST_THE_WINDOW):
            with self.assertRaisesRegex(ValueError, "doesn't fit into the encoder cover"):
                motor.check_connector_inside_window()

    def test_check_screw_recesses_inside_face_raises_a_value_error_past_the_end(self) -> None:
        """Put the screws' centre past the gearbox's end: an error."""
        with mock.patch.object(motor, 'SCREW_X', SCREW_PAST_THE_END):
            with self.assertRaisesRegex(ValueError, "a screw's recess reaches past"):
                motor.check_screw_recesses_inside_face()

    def test_check_holes_inside_face_raises_a_value_error_outside_the_round_end(self) -> None:
        """Put a small hole into the corner that the R5 round takes away: an error."""
        with mock.patch.object(motor, 'SHAFT_SIDE_HOLES', (HOLE_OUTSIDE_THE_ROUND,)):
            with self.assertRaisesRegex(ValueError, r'the hole at \(x, z\) = \(34.0, 9.5\) mm'):
                motor.check_holes_inside_face()

    def test_check_holes_inside_face_raises_a_value_error_for_a_recess_past_the_end(
            self) -> None:
        """Let the -y recess reach past the gearbox's end: an error."""
        with mock.patch.object(motor, 'BACK_RECESS', RECESS_PAST_THE_END):
            with self.assertRaisesRegex(ValueError, 'the -y recess reaches past'):
                motor.check_holes_inside_face()

    def test_check_fins_on_neck_raises_a_value_error_for_a_foot_on_the_chamfer(self) -> None:
        """Move the fins' front face so close to the housing that their foot leaves the neck."""
        with mock.patch.object(motor, 'FIN_FRONT_X', FIN_ON_THE_CHAMFER):
            with self.assertRaisesRegex(ValueError, "a fin's foot reaches past the neck"):
                motor.check_fins_on_neck()

    def test_check_fins_on_neck_raises_a_value_error_for_a_fin_taller_than_the_flat(
            self) -> None:
        """Make the fins taller than the neck's flat, onto its round sides: an error."""
        with mock.patch.object(motor, 'FIN_HEIGHT', FIN_TALLER_THAN_THE_FLAT):
            with self.assertRaisesRegex(ValueError, "a fin's foot reaches past the neck's flat"):
                motor.check_fins_on_neck()

    def test_check_connector_box_under_flat_raises_a_value_error_for_a_wider_box(self) -> None:
        """Make the connector's box wider than the housing's flat: an error names both widths."""
        with mock.patch.object(motor, 'CONNECTOR_BOX_HEIGHT', BOX_WIDER_THAN_THE_FLAT):
            with self.assertRaisesRegex(ValueError, f'{BOX_WIDER_THAN_THE_FLAT:.2f} mm wide, '
                                        f"the motor housing's flat {2 * HOUSING_FLAT:.2f} mm"):
                motor.check_connector_box_under_flat()

    def test_check_encoder_cover_outline_raises_a_value_error_for_too_large_corners(
            self) -> None:
        """Make the cover's corners larger than half its height: an error."""
        with mock.patch.object(motor, 'ENCODER_TOP_EDGE_ROUND', CORNERS_PAST_HALF_THE_HEIGHT):
            with self.assertRaisesRegex(ValueError, 'corners are larger than half its height'):
                motor.check_encoder_cover_outline()

    def test_check_encoder_cover_outline_raises_a_value_error_for_a_sliver_of_a_flat_strip(
            self) -> None:
        """Leave a flat strip shorter than the shortest straight piece: an error."""
        with mock.patch.object(motor, 'ENCODER_TOP_EDGE_ROUND', CORNERS_WITH_A_SLIVER):
            with self.assertRaisesRegex(ValueError, 'flat strip between its corners is too short'):
                motor.check_encoder_cover_outline()

    def test_check_encoder_cover_outline_accepts_straight_pieces_of_the_shortest_length(
            self) -> None:
        """Leave exactly the shortest straight piece, one piece at a time: no error, a solid."""
        for name, value in SHORTEST_PIECES.items():
            with self.subTest(name=name), mock.patch.object(motor, name, value):
                motor.check_encoder_cover_outline()
                self.assertTrue(motor.cover_end_block().isValid())

    def test_check_encoder_cover_outline_raises_a_value_error_for_faces_as_long_as_corners(
            self) -> None:
        """End the round part's +-z faces where its corners end, no straight piece: an error."""
        with mock.patch.object(motor, 'ENCODER_ROUND_LENGTH', motor.ENCODER_TOP_EDGE_ROUND):
            with self.assertRaisesRegex(ValueError, 'needs straight \\+-z faces past its'):
                motor.check_encoder_cover_outline()

    def test_check_encoder_cover_outline_raises_a_value_error_for_an_s_curve_to_the_rounds(
            self) -> None:
        """End the S-curve where the rounds of the box's -y end start: an error."""
        with mock.patch.object(motor, 'ENCODER_STEP_LENGTH', S_CURVE_TO_THE_END_ROUNDS):
            with self.assertRaisesRegex(ValueError, 'box needs straight faces between'):
                motor.check_encoder_cover_outline()

    def test_check_window_inside_box_raises_a_value_error_for_a_window_as_wide_as_the_box(
            self) -> None:
        """Make the connector's window as wide as its box: an error."""
        with mock.patch.object(motor, 'WINDOW_HALF_HEIGHT', WINDOW_AS_WIDE_AS_THE_BOX):
            with self.assertRaisesRegex(ValueError, "the connector's window cuts through"):
                motor.check_window_inside_box()

    def test_check_window_inside_box_raises_a_value_error_for_a_board_in_the_end_round(
            self) -> None:
        """Let the board's corner stand out of the end face's round: an error."""
        with mock.patch.object(motor, 'WINDOW_X', WINDOW_IN_THE_END_ROUND), \
                mock.patch.object(motor, 'WINDOW_HALF_HEIGHT', WINDOW_NEAR_THE_SIDES):
            with self.assertRaisesRegex(ValueError, "the connector's window cuts through"):
                motor.check_window_inside_box()

    def test_check_window_inside_box_raises_a_value_error_for_a_board_past_the_end(
            self) -> None:
        """Start the window 0.1 mm from the cover's end, so the board stands out: an error."""
        with mock.patch.object(motor, 'WINDOW_X', WINDOW_AT_THE_END):
            with self.assertRaisesRegex(ValueError, "the connector's window cuts through"):
                motor.check_window_inside_box()


class SubPartTest(unittest.TestCase):
    """sub_part() accepts the motor's material names and no others."""

    def test_sub_part_has_a_surface_for_each_material_of_its_solids(self) -> None:
        """Give the body and the contacts under their names: both surfaces, in order."""
        part = motor.sub_part('connector', {'tin': [cube().val()],
                                            'white_plastic': [cube().val()]})

        self.assertEqual(list(part.surfaces), ['white_plastic', 'tin'])
        self.assertEqual([len(surface.triangles) for surface in part.surfaces.values()],
                         [CUBE_TRIANGLES, CUBE_TRIANGLES])

    def test_sub_part_raises_a_value_error_for_a_misspelled_material(self) -> None:
        """Give the contacts as "tins": an error names the sub-part and the name."""
        with self.assertRaisesRegex(ValueError, "sub-part 'connector'.*tins"):
            motor.sub_part('connector', {'white_plastic': [cube().val()],
                                         'tins': [cube().val()]})


class RoundFaceEdgesTest(unittest.TestCase):
    """round_face_edges() rounds the edges of the selected faces, an edge they share once."""

    @staticmethod
    def difference(first: cq.Shape, second: cq.Shape) -> float:
        """Return the volume that lies in one of the two shapes but not in the other (mm3)."""
        return float(first.cut(second).Volume() + second.cut(first).Volume())

    def test_round_face_edges_rounds_the_same_edges_as_cadquerys_or_selector(self) -> None:
        """Round a box's +y and -x faces' edges: the same solid as with ">Y or <X"."""
        block = cq.Workplane('XY').box(10.0, 10.0, 10.0)
        expected = block.faces('>Y or <X').edges().fillet(1.0).val()

        rounded = motor.round_face_edges(block, ('>Y', '<X'), 1.0)

        self.assertAlmostEqual(self.difference(rounded, expected), 0.0, places=6)

    def test_round_face_edges_rounds_only_the_selected_faces_edges(self) -> None:
        """Round the +y and -x faces' edges: another solid than with the -y and +x faces'."""
        block = cq.Workplane('XY').box(10.0, 10.0, 10.0)
        opposite = motor.round_face_edges(block, ('<Y', '>X'), 1.0)

        rounded = motor.round_face_edges(block, ('>Y', '<X'), 1.0)

        self.assertGreater(self.difference(rounded, opposite), 1.0)


class BuiltMeshTest(unittest.TestCase):
    """The built mesh has the drawing's sizes and its output shaft where the frame says."""

    model: Model
    corners: Dict[str, np.ndarray]

    @classmethod
    def setUpClass(cls) -> None:
        """Build the motor once and keep each sub-part's vertices (mm)."""
        cls.model = motor.build()[0]
        cls.corners = {part.name: np.vstack([surface.vertices
                                             for surface in part.surfaces.values()])
                       for part in cls.model.sub_parts}

    def bounds(self, *names: str) -> np.ndarray:
        """Return the lowest and highest corner of the named sub-parts, or of all of them."""
        chosen = names or tuple(self.corners)
        points = np.vstack([self.corners[name] for name in chosen])
        return np.array([points.min(axis=0), points.max(axis=0)])

    def test_mesh_reaches_from_the_encoder_covers_end_to_the_tabs_end_along_x(self) -> None:
        """Measure the mesh along x: from ENCODER_END_X to TAB_END_X, half the drawing's length."""
        low, high = self.bounds()

        self.assertAlmostEqual(low[0], motor.ENCODER_END_X, delta=SIZE_TOLERANCE)
        self.assertAlmostEqual(high[0], motor.TAB_END_X, delta=SIZE_TOLERANCE)
        self.assertAlmostEqual(high[0], motor.DRAWING_LENGTH / 2, delta=SIZE_TOLERANCE)

    def test_mesh_reaches_from_the_encoder_covers_face_to_the_shafts_end_along_y(self) -> None:
        """Measure the mesh along y: from the cover's -y face to the shaft's end."""
        low, high = self.bounds()
        cover_face = motor.ENCODER_TOP_Y - motor.ENCODER_DEPTH
        shaft_end = motor.GEARBOX_THICKNESS / 2 + motor.SHAFT_LENGTH

        self.assertAlmostEqual(low[1], cover_face, delta=SIZE_TOLERANCE)
        self.assertAlmostEqual(high[1], shaft_end, delta=SIZE_TOLERANCE)

    def test_mesh_is_as_high_as_the_motor_housing_about_the_middle_plane(self) -> None:
        """Measure the mesh along z: the housing's height, half above and half below z = 0.

        The housing's round sides are tessellated, so their extremes may lie up to the linear
        deflection inside the diameter.
        """
        low, high = self.bounds()

        self.assertAlmostEqual(low[2], -motor.HOUSING_HEIGHT / 2, delta=motor.LINEAR_DEFLECTION)
        self.assertAlmostEqual(high[2], motor.HOUSING_HEIGHT / 2, delta=motor.LINEAR_DEFLECTION)

    def test_shaft_has_its_axis_at_shaft_x_and_its_diameter_and_flats(self) -> None:
        """Measure the shaft's sub-part: centred on SHAFT_X and z = 0, as wide and as thick.

        Along x the round sides are tessellated, so their extremes may lie up to the linear
        deflection inside the diameter.
        """
        low, high = self.bounds('shaft')

        self.assertAlmostEqual((low[0] + high[0]) / 2, motor.SHAFT_X,
                               delta=motor.LINEAR_DEFLECTION)
        self.assertAlmostEqual(high[0] - low[0], motor.SHAFT_DIAMETER,
                               delta=motor.LINEAR_DEFLECTION)
        self.assertAlmostEqual((low[2] + high[2]) / 2, 0.0, delta=SIZE_TOLERANCE)
        self.assertAlmostEqual(high[2] - low[2], motor.SHAFT_FLATS, delta=SIZE_TOLERANCE)

    def test_gearbox_has_the_drawings_thickness_and_height_about_the_middle_planes(self) -> None:
        """Measure the gearbox's sub-part without the boss: 18.75 by 22.4 mm about y = z = 0."""
        points = self.corners['gearbox']
        body = points[points[:, 1] <= motor.GEARBOX_THICKNESS / 2 + SIZE_TOLERANCE]

        self.assertAlmostEqual(body[:, 1].min(), -motor.GEARBOX_THICKNESS / 2,
                               delta=SIZE_TOLERANCE)
        self.assertAlmostEqual(body[:, 1].max(), motor.GEARBOX_THICKNESS / 2,
                               delta=SIZE_TOLERANCE)
        self.assertAlmostEqual(body[:, 2].min(), -motor.GEARBOX_HEIGHT / 2, delta=SIZE_TOLERANCE)
        self.assertAlmostEqual(body[:, 2].max(), motor.GEARBOX_HEIGHT / 2, delta=SIZE_TOLERANCE)


class CoverOutlineTest(unittest.TestCase):
    """cover_end_block() builds the encoder cover's D shape, also with a full semicircle."""

    def test_cover_end_block_builds_a_full_semicircle_without_a_flat_strip(self) -> None:
        """Make the corners half as large as the round part's height: a semicircle.

        The solid is as wide and as high as with the generator's corners. A point just outside
        the semicircle, on its diagonal, lies inside the generator's smaller corners only.
        """
        half = motor.ENCODER_HEIGHT / 2
        with mock.patch.object(motor, 'ENCODER_TOP_EDGE_ROUND', half):
            motor.check_encoder_cover_outline()
            semicircle = motor.cover_end_block()
        reach = (half + OFF_THE_OUTLINE) * 2 ** -0.5
        diagonal = cq.Vector(COVER_END_MIDDLE_X, motor.ENCODER_TOP_Y - half + reach, reach)

        bounds = semicircle.BoundingBox()
        self.assertTrue(semicircle.isValid())
        self.assertAlmostEqual(bounds.ymax, motor.ENCODER_TOP_Y, delta=SIZE_TOLERANCE)
        self.assertAlmostEqual(bounds.zmax, half, delta=SIZE_TOLERANCE)
        self.assertFalse(semicircle.isInside(diagonal))
        self.assertTrue(motor.cover_end_block().isInside(diagonal))


class BuiltFeaturesTest(unittest.TestCase):
    """The built solids have the holes and the fins that set where the motor sits.

    The motor housing and the neck are round between their flats, and the encoder cover is
    shaped like a D, as on the motor.
    """

    gearbox: cq.Shape
    housing: cq.Shape
    cover: cq.Shape

    @classmethod
    def setUpClass(cls) -> None:
        """Build the gearbox, the motor housing and the encoder cover once."""
        cls.gearbox = motor.make_gearbox()
        cls.housing = motor.make_motor_housing()
        cls.cover = motor.make_encoder_cover()

    def assert_round_between_flats(self, x: float, diameter: float, thickness: float) -> None:
        """Look 2 mm inside each flat's edge: inside the round side, then just outside it."""
        y = thickness / 2 - 2.0
        for radius, expected in ((diameter / 2 - OFF_THE_OUTLINE, True),
                                 (diameter / 2 + OFF_THE_OUTLINE, False)):
            z = (radius ** 2 - y ** 2) ** 0.5
            for side_y, side_z in ((1, 1), (1, -1), (-1, 1), (-1, -1)):
                point = cq.Vector(x, side_y * y, side_z * z)
                self.assertEqual(self.housing.isInside(point), expected, point)

    def test_mounting_holes_are_open_through_the_gearbox(self) -> None:
        """Look at each mounting hole's axis on both faces and in the middle: no material."""
        face = motor.GEARBOX_THICKNESS / 2 - SIZE_TOLERANCE
        for z in (motor.MOUNT_HOLE_Z, -motor.MOUNT_HOLE_Z):
            for y in (-face, 0.0, face):
                self.assertFalse(self.gearbox.isInside(cq.Vector(motor.MOUNT_HOLE_X, y, z)))

    def test_mounting_holes_have_material_beside_their_edge(self) -> None:
        """Look just beside each mounting hole's edge, towards the boss: material."""
        beside = motor.MOUNT_HOLE_X + motor.MOUNT_HOLE_DIAMETER / 2 + BESIDE_THE_EDGE
        for z in (motor.MOUNT_HOLE_Z, -motor.MOUNT_HOLE_Z):
            self.assertTrue(self.gearbox.isInside(cq.Vector(beside, 0.0, z)))

    def test_tabs_hole_is_open_through_the_tab(self) -> None:
        """Look at the tab's hole's axis on both faces and in the middle: no material."""
        face = motor.TAB_THICKNESS / 2 - SIZE_TOLERANCE
        for y in (-face, 0.0, face):
            self.assertFalse(self.gearbox.isInside(cq.Vector(motor.TAB_HOLE_X, y, 0.0)))

    def test_tabs_hole_has_material_beside_its_edge(self) -> None:
        """Look just beside the tab's hole's edge, towards the gearbox: material."""
        beside = motor.TAB_HOLE_X - motor.TAB_HOLE_DIAMETER / 2 - BESIDE_THE_EDGE
        self.assertTrue(self.gearbox.isInside(cq.Vector(beside, 0.0, 0.0)))

    def test_motor_housing_is_round_between_its_flats(self) -> None:
        """Look at the housing's middle near its four corners: material only inside the round."""
        self.assert_round_between_flats(HOUSING_MIDDLE_X, motor.HOUSING_HEIGHT,
                                        motor.HOUSING_THICKNESS)

    def test_neck_is_round_between_its_flats(self) -> None:
        """Look at the neck near its four corners, between the fins and the gearbox."""
        self.assert_round_between_flats(NECK_MIDDLE_X, motor.NECK_HEIGHT, motor.NECK_THICKNESS)

    def test_encoder_cover_has_round_corners_and_a_narrower_box(self) -> None:
        """Look across the cover's part at the motor's end: its round corners, then its box.

        Points just inside the corners' rounds on the +y side are material, points just
        outside them are not. Beside the box that holds the connector, at a z that the round
        part still reaches, there is no material.
        """
        corner = motor.ENCODER_TOP_EDGE_ROUND
        centre_y = motor.ENCODER_TOP_Y - corner
        centre_z = motor.ENCODER_HEIGHT / 2 - corner
        diagonal = 2 ** -0.5
        for side in (1, -1):
            for reach, expected in ((corner - OFF_THE_OUTLINE, True),
                                    (corner + OFF_THE_OUTLINE, False)):
                point = cq.Vector(COVER_END_MIDDLE_X, centre_y + reach * diagonal,
                                  side * (centre_z + reach * diagonal))
                self.assertEqual(self.cover.isInside(point), expected, point)
        box_end = motor.ENCODER_TOP_Y - motor.ENCODER_DEPTH
        step_end = motor.ENCODER_TOP_Y - motor.ENCODER_ROUND_LENGTH - motor.ENCODER_STEP_LENGTH
        box_y = (box_end + step_end) / 2
        for z, expected in ((motor.CONNECTOR_BOX_HEIGHT / 2 - OFF_THE_OUTLINE, True),
                            (motor.CONNECTOR_BOX_HEIGHT / 2 + OFF_THE_OUTLINE, False)):
            point = cq.Vector(COVER_END_MIDDLE_X, box_y, z)
            self.assertEqual(self.cover.isInside(point), expected, point)

    def test_connector_box_reaches_its_measured_end_and_no_further(self) -> None:
        """Look just before and just past the box's end, below the housing: inside, outside."""
        below_flat = (motor.ENCODER_TOP_Y - motor.ENCODER_DEPTH - motor.HOUSING_THICKNESS / 2) / 2
        before = cq.Vector(motor.CONNECTOR_BOX_END_X - OFF_THE_OUTLINE, below_flat, 0.0)
        past = cq.Vector(motor.CONNECTOR_BOX_END_X + OFF_THE_OUTLINE, below_flat, 0.0)
        self.assertTrue(self.cover.isInside(before))
        self.assertFalse(self.cover.isInside(past))

    def test_each_fin_reaches_its_top_and_no_further(self) -> None:
        """Look just below and just above each fin's top, in its middle: inside, then outside."""
        middle_x = motor.FIN_FRONT_X - motor.FIN_TOP_LENGTH / 2
        for side in (1, -1):
            below = cq.Vector(middle_x, side * (motor.FIN_TOP_Y - AROUND_THE_TOP), 0.0)
            above = cq.Vector(middle_x, side * (motor.FIN_TOP_Y + AROUND_THE_TOP), 0.0)
            self.assertTrue(self.housing.isInside(below))
            self.assertFalse(self.housing.isInside(above))


if __name__ == '__main__':
    unittest.main()
