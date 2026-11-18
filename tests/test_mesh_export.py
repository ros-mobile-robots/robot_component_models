# SPDX-FileCopyrightText: 2026 ROS Mobile Robots
# SPDX-License-Identifier: BSD-3-Clause
"""Tests of how generators/mesh_export.py groups a sub-part's pieces by material.

A piece whose material name is not one of the model's materials must stop the generation with
an error that names the sub-part and the name. Run from the repository's root, with the
packages of generators/requirements.txt:

  python3 -m unittest discover -s tests
"""
import unittest
from unittest import mock

import cadquery as cq
import numpy as np

from generators import breadboard_400_point
from generators.mesh_export import classified_sub_part
from generators.mesh_export import Model
from generators.mesh_export import phong
from generators.mesh_export import require_model_materials
from generators.mesh_export import SubPart
from generators.mesh_export import Surface
from generators.mesh_export import surfaces_by_material

MATERIALS = ('gold', 'steel')  # a model's material names, in the order of its files
MISSPELLED_MATERIAL = 'gould'
SUB_PART = 'connector'
LINEAR_DEFLECTION = 0.05  # mm
ANGULAR_DEFLECTION = 0.6  # rad
CUBE_TRIANGLES = 12  # two per face
# The error has to name the sub-part and the unknown name, in this order
ERROR_WITH_SUB_PART_AND_NAME = f"sub-part '{SUB_PART}'.*{MISSPELLED_MATERIAL}"


def one_triangle() -> Surface:
    """Return a surface of one triangle in the xy plane."""
    return Surface(np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]),
                   np.array([[0.0, 0.0, 1.0]] * 3), np.array([[0, 1, 2]]))


def cube() -> cq.Workplane:
    """Return a cube with an edge of 1 mm."""
    return cq.Workplane('XY').box(1.0, 1.0, 1.0)


def triangle_count(surface: Surface) -> int:
    """Return the number of triangles of a surface."""
    return len(surface.triangles)


class SurfacesByMaterialTest(unittest.TestCase):
    """surfaces_by_material() merges (material, surface) pieces into one surface per material."""

    def test_surfaces_by_material_returns_the_materials_in_the_given_order(self) -> None:
        """Give pieces as steel, gold, steel: gold comes back first, steel with both pieces."""
        pieces = [('steel', one_triangle()), ('gold', one_triangle()), ('steel', one_triangle())]

        surfaces = surfaces_by_material(SUB_PART, pieces, MATERIALS)

        self.assertEqual(list(surfaces), ['gold', 'steel'])
        self.assertEqual(triangle_count(surfaces['gold']), 1)
        self.assertEqual(triangle_count(surfaces['steel']), 2)

    def test_surfaces_by_material_raises_a_value_error_for_a_misspelled_material(self) -> None:
        """Give a piece a material outside the order: an error names the sub-part and it."""
        pieces = [('gold', one_triangle()), (MISSPELLED_MATERIAL, one_triangle())]

        with self.assertRaisesRegex(ValueError, ERROR_WITH_SUB_PART_AND_NAME):
            surfaces_by_material(SUB_PART, pieces, MATERIALS)


class ClassifiedSubPartTest(unittest.TestCase):
    """classified_sub_part() tessellates (material, workplane) pieces into a sub-part."""

    def test_classified_sub_part_keeps_every_triangle_of_known_materials(self) -> None:
        """Give two cubes of known materials: the sub-part has the triangles of two cubes."""
        pieces = [('steel', cube()), ('gold', cube())]

        sub_part = classified_sub_part(SUB_PART, pieces, MATERIALS, LINEAR_DEFLECTION,
                                       ANGULAR_DEFLECTION)

        self.assertEqual(list(sub_part.surfaces), ['gold', 'steel'])
        self.assertEqual(triangle_count(sub_part.surfaces['gold']), CUBE_TRIANGLES)
        self.assertEqual(triangle_count(sub_part.surfaces['steel']), CUBE_TRIANGLES)

    def test_classified_sub_part_raises_a_value_error_for_a_misspelled_material(self) -> None:
        """Give a piece a material outside the order: an error names the sub-part and it."""
        pieces = [('gold', cube()), (MISSPELLED_MATERIAL, cube())]

        with self.assertRaisesRegex(ValueError, ERROR_WITH_SUB_PART_AND_NAME):
            classified_sub_part(SUB_PART, pieces, MATERIALS, LINEAR_DEFLECTION,
                                ANGULAR_DEFLECTION)

    def test_classified_sub_part_raises_a_value_error_for_a_misspelled_classifier_label(
            self) -> None:
        """Let a classifier return a material outside the order: an error names it."""
        def misspelling_classifier(triangles: np.ndarray) -> np.ndarray:
            """Return the misspelled material for the first triangle, gold for the others."""
            labels = np.full(len(triangles), 'gold', dtype=object)
            labels[0] = MISSPELLED_MATERIAL
            return labels

        with self.assertRaisesRegex(ValueError, ERROR_WITH_SUB_PART_AND_NAME):
            classified_sub_part(SUB_PART, [('gold', cube())], MATERIALS, LINEAR_DEFLECTION,
                                ANGULAR_DEFLECTION, {'gold': misspelling_classifier})

    def test_classified_sub_part_raises_a_value_error_for_a_misspelled_classifier_key(
            self) -> None:
        """Register a classifier under a material outside the order: an error names it."""
        def gold_classifier(triangles: np.ndarray) -> np.ndarray:
            """Return gold for every triangle."""
            return np.full(len(triangles), 'gold', dtype=object)

        with self.assertRaisesRegex(ValueError, ERROR_WITH_SUB_PART_AND_NAME):
            classified_sub_part(SUB_PART, [('gold', cube())], MATERIALS, LINEAR_DEFLECTION,
                                ANGULAR_DEFLECTION, {MISSPELLED_MATERIAL: gold_classifier})


class RequireModelMaterialsTest(unittest.TestCase):
    """require_model_materials() checks a finished model before its files are written."""

    def model(self, surface_material: str) -> Model:
        """Return a model of gold and steel whose one sub-part has a surface of the material."""
        palette = {name: phong((0.5, 0.5, 0.5), (0.1, 0.1, 0.1), 10.0) for name in MATERIALS}
        return Model('part', 'A part', 'The origin at a corner.', palette,
                     [SubPart(SUB_PART, {surface_material: one_triangle()})])

    def test_require_model_materials_accepts_a_surface_of_a_known_material(self) -> None:
        """Give the sub-part a gold surface: no error."""
        require_model_materials(self.model('gold'))

    def test_require_model_materials_raises_a_value_error_for_an_unknown_material(self) -> None:
        """Give the sub-part a surface of a material outside the model's: an error names it."""
        with self.assertRaisesRegex(ValueError, ERROR_WITH_SUB_PART_AND_NAME):
            require_model_materials(self.model(MISSPELLED_MATERIAL))


class BreadboardRailLinesTest(unittest.TestCase):
    """The breadboard's rail lines take their material names from STRIPE_OFFSETS' keys."""

    def rail_lines(self) -> SubPart:
        """Return the breadboard's sub-part of the rail lines, as build() makes it."""
        return classified_sub_part(
            'rail_lines', breadboard_400_point.stripes(), list(breadboard_400_point.MATERIALS),
            breadboard_400_point.TOL, breadboard_400_point.ANG_TOL,
            breadboard_400_point.CLASSIFIERS)

    def test_rail_lines_have_red_and_blue_surfaces(self) -> None:
        """Build the rail lines from the generator's table: both colours have triangles."""
        surfaces = self.rail_lines().surfaces

        self.assertEqual(list(surfaces), ['red', 'blue'])
        self.assertEqual(triangle_count(surfaces['red']), triangle_count(surfaces['blue']))

    def test_rail_lines_raise_a_value_error_for_a_misspelled_colour(self) -> None:
        """Misspell the red lines' key: an error names the rail lines and the key.

        The blue lines are in the same sub-part. So the sub-part would still exist without the
        red lines, and the files would only have fewer triangles.
        """
        misspelled = {'rde': breadboard_400_point.STRIPE_OFFSETS['red'],
                      'blue': breadboard_400_point.STRIPE_OFFSETS['blue']}

        with mock.patch.object(breadboard_400_point, 'STRIPE_OFFSETS', misspelled):
            with self.assertRaisesRegex(ValueError, "sub-part 'rail_lines'.*rde"):
                self.rail_lines()


if __name__ == '__main__':
    unittest.main()
