# SPDX-FileCopyrightText: 2026 ROS Mobile Robots
# SPDX-License-Identifier: BSD-3-Clause
"""Tests of how scripts/check_meshes.py compares a COLLADA or glTF file with the STL file.

The three files of a mesh must hold the same triangles with the same winding. The files may
list the triangles in another order, start a triangle at another corner, and round the
coordinates differently. Run from the repository's root, with the packages of
generators/requirements.txt:

  python3 -m unittest discover -s tests
"""
import os
import shutil
import tempfile
from typing import Callable
from typing import Dict
from typing import List
import unittest

import numpy as np

from generators.mesh_export import Model
from generators.mesh_export import phong
from generators.mesh_export import SubPart
from generators.mesh_export import Surface
from generators.mesh_export import write_dae
from generators.mesh_export import write_glb
from generators.mesh_export import write_stl
from scripts import check_meshes

# A tetrahedron's corner points in metres, with digits beyond the files' precision
POINTS = np.array([[0.0012345678, 0.0023456789, 0.0034567891],
                   [0.0123456789, 0.0023456789, 0.0034567891],
                   [0.0012345678, 0.0234567891, 0.0034567891],
                   [0.0012345678, 0.0023456789, 0.0345678912]])
# Its four faces as point numbers, counter-clockwise seen from outside
FACES = np.array([[0, 2, 1], [0, 1, 3], [1, 2, 3], [0, 3, 2]])
REVERSED_FACE = 2
MILLIMETRES_PER_METRE = 1000.0
COLLADA_DIGITS = 7  # significant digits of a position in a COLLADA file
OPPOSITE_WINDING = "1 triangles have the opposite winding of the STL file's"


def tetrahedron() -> np.ndarray:
    """Return the tetrahedron's triangle corners (4, 3, 3) in metres."""
    corners: np.ndarray = POINTS[FACES]
    return corners


def with_reversed_face(corners: np.ndarray) -> np.ndarray:
    """Return the triangle corners with the corners of one face in the opposite order."""
    changed = corners.copy()
    changed[REVERSED_FACE] = changed[REVERSED_FACE][::-1]
    return changed


def rounded_like_collada(corners: np.ndarray) -> np.ndarray:
    """Return the corners rounded to the significant digits of a COLLADA file."""
    rounded = [float(f'{value:.{COLLADA_DIGITS}g}') for value in corners.ravel()]
    return np.array(rounded).reshape(corners.shape)


def differences(corners: np.ndarray) -> List[str]:
    """Return how the corners differ from the tetrahedron as an STL file holds it."""
    stl = tetrahedron().astype(np.float32).astype(float)
    return check_meshes.triangle_differences(stl, corners, check_meshes.VERTEX_TOLERANCE)


class TriangleDifferencesTest(unittest.TestCase):
    """triangle_differences() compares oriented triangles, whatever their order."""

    def test_triangle_differences_finds_none_for_another_order_and_first_corner(self) -> None:
        """List the faces backwards and start each at its second corner: no difference."""
        reordered = np.roll(tetrahedron()[::-1], 1, axis=1)

        self.assertEqual(differences(reordered), [])

    def test_triangle_differences_finds_none_for_collada_rounding(self) -> None:
        """Round the corners to a COLLADA file's seven digits: no difference."""
        self.assertEqual(differences(rounded_like_collada(tetrahedron())), [])

    def test_triangle_differences_reports_a_reversed_face(self) -> None:
        """Reverse one face's corners: one triangle with the opposite winding."""
        self.assertEqual(differences(with_reversed_face(tetrahedron())), [OPPOSITE_WINDING])

    def test_triangle_differences_reports_a_missing_triangle(self) -> None:
        """Leave out one face: one triangle of the STL file is missing."""
        self.assertEqual(differences(tetrahedron()[1:]),
                         ['1 triangles of the STL file are missing'])

    def test_triangle_differences_reports_a_triangle_that_the_stl_file_lacks(self) -> None:
        """Add a second copy of one face: one triangle isn't in the STL file."""
        doubled = np.concatenate([tetrahedron(), tetrahedron()[:1]])

        self.assertEqual(differences(doubled), ['1 triangles are not in the STL file'])

    def test_triangle_differences_reports_a_corner_away_from_every_vertex(self) -> None:
        """Move one corner by ten times the tolerance: that corner has no STL vertex."""
        moved = tetrahedron()
        moved[0, 0, 0] += 10 * check_meshes.VERTEX_TOLERANCE

        problems = differences(moved)

        self.assertEqual(len(problems), 1)
        self.assertIn('1 corners lie more than', problems[0])


class WrittenFilesTest(unittest.TestCase):
    """The files that mesh_export writes, read back by the checker's readers."""

    def setUp(self) -> None:
        """Make a folder for the test's files."""
        self.folder = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.folder)

    def model(self, corners: np.ndarray) -> Model:
        """Return a model of one sub-part with the triangle corners, given in metres."""
        points = corners.reshape(-1, 3) * MILLIMETRES_PER_METRE
        triangles = np.arange(len(points)).reshape(-1, 3)
        edges = corners[:, 1] - corners[:, 0], corners[:, 2] - corners[:, 0]
        normals = np.repeat(np.cross(*edges), 3, axis=0)
        normals /= np.linalg.norm(normals, axis=1, keepdims=True)
        surface = Surface(points, normals, triangles)
        return Model('tetrahedron', 'A tetrahedron', 'The origin outside the tetrahedron.',
                     {'grey': phong((0.5, 0.5, 0.5), (0.1, 0.1, 0.1), 10.0)},
                     [SubPart('body', {'grey': surface})])

    def written(self, corners: np.ndarray, extension: str) -> np.ndarray:
        """Return the corners that the checker reads from the model's file of the format."""
        path = os.path.join(self.folder, 'tetrahedron' + extension)
        writers: Dict[str, Callable[[Model, str], object]] = {
            '.stl': write_stl,
            '.dae': lambda model, path: write_dae(model, path, 'tests/test_check_meshes.py'),
            '.glb': write_glb}
        writers[extension](self.model(corners), path)
        read: np.ndarray = check_meshes.READERS[extension](path)
        return read

    def differences_from_stl(self, corners: np.ndarray, extension: str) -> List[str]:
        """Return how the written file of the corners differs from the tetrahedron's STL."""
        return check_meshes.triangle_differences(self.written(tetrahedron(), '.stl'),
                                                 self.written(corners, extension),
                                                 check_meshes.VERTEX_TOLERANCE)

    def test_written_collada_file_has_the_stl_files_triangles(self) -> None:
        """Write the tetrahedron as STL and COLLADA: the checker finds no difference."""
        self.assertEqual(self.differences_from_stl(tetrahedron(), '.dae'), [])

    def test_written_gltf_file_has_the_stl_files_triangles(self) -> None:
        """Write the tetrahedron as STL and glTF: no difference after the root node's turn."""
        self.assertEqual(self.differences_from_stl(tetrahedron(), '.glb'), [])

    def test_written_collada_file_with_a_reversed_face_differs_from_the_stl_file(self) -> None:
        """Write a COLLADA file with one face reversed: the checker reports its winding."""
        self.assertEqual(self.differences_from_stl(with_reversed_face(tetrahedron()), '.dae'),
                         [OPPOSITE_WINDING])

    def test_written_gltf_file_with_a_reversed_face_differs_from_the_stl_file(self) -> None:
        """Write a glTF file with one face reversed: the checker reports its winding."""
        self.assertEqual(self.differences_from_stl(with_reversed_face(tetrahedron()), '.glb'),
                         [OPPOSITE_WINDING])


if __name__ == '__main__':
    unittest.main()
