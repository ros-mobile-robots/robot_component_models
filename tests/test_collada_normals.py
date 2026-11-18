# SPDX-FileCopyrightText: 2026 ROS Mobile Robots
# SPDX-License-Identifier: BSD-3-Clause
"""Tests of how scripts/check_meshes.py checks the normals of a COLLADA file.

mesh_export writes a COLLADA file's normals as an input of its triangle lists: every corner is
its position's index followed by its normal's index, and the two are the same number. A file
can declare these inputs and still give a corner another vertex's normal. The tests damage the
indices of a file that mesh_export wrote, and the checker has to report each damage. Run from
the repository's root, with the packages of generators/requirements.txt:

  python3 -m unittest discover -s tests
"""
import os
import shutil
import tempfile
from typing import Any
from typing import List
import unittest

from lxml import etree
import numpy as np

from generators.mesh_export import Model
from generators.mesh_export import phong
from generators.mesh_export import SubPart
from generators.mesh_export import Surface
from generators.mesh_export import write_dae
from scripts import check_meshes

# A tetrahedron in millimetres: four points, four faces counter-clockwise seen from outside
POINTS = np.array([[0.0, 0.0, 0.0], [10.0, 0.0, 0.0], [0.0, 10.0, 0.0], [0.0, 0.0, 10.0]])
FACES = np.array([[0, 2, 1], [0, 1, 3], [1, 2, 3], [0, 3, 2]])
CORNERS = 3 * len(FACES)  # every corner is a vertex of its own, with its face's normal
NAMESPACE = {'c': check_meshes.COLLADA_NAMESPACE}


def tetrahedron() -> Model:
    """Return the tetrahedron as a model of one sub-part and one material."""
    corners = POINTS[FACES]
    normals = np.repeat(np.cross(corners[:, 1] - corners[:, 0], corners[:, 2] - corners[:, 0]),
                        3, axis=0)
    normals /= np.linalg.norm(normals, axis=1, keepdims=True)
    surface = Surface(corners.reshape(-1, 3), normals, np.arange(CORNERS).reshape(-1, 3))
    return Model('tetrahedron', 'A tetrahedron', 'The origin at a corner.',
                 {'grey': phong((0.5, 0.5, 0.5), (0.1, 0.1, 0.1), 10.0)},
                 [SubPart('body', {'grey': surface})])


class ColladaNormalsTest(unittest.TestCase):
    """collada_normal_problems() reads the index pairs of every corner."""

    def setUp(self) -> None:
        """Write the tetrahedron's COLLADA file and parse it."""
        folder = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, folder)
        path = os.path.join(folder, 'tetrahedron.dae')
        write_dae(tetrahedron(), path, 'tests/test_collada_normals.py')
        self.document = etree.parse(path)

    def index_list(self) -> Any:
        """Return the <p> element of the file's one triangle list."""
        lists = self.document.xpath('//c:triangles/c:p', namespaces=NAMESPACE)
        self.assertEqual(len(lists), 1)
        return lists[0]

    def indices(self) -> List[str]:
        """Return the numbers of the triangle list, as the file's text."""
        numbers: List[str] = self.index_list().text.split()
        return numbers

    def problems_with_indices(self, numbers: List[str]) -> List[str]:
        """Return the checker's problems after the triangle list's numbers are replaced."""
        self.index_list().text = ' '.join(numbers)
        return check_meshes.collada_normal_problems(self.document)

    def test_written_collada_file_has_no_normal_problems(self) -> None:
        """Check the file as mesh_export wrote it: no problem, a pair for every corner."""
        self.assertEqual(len(self.indices()), 2 * CORNERS)
        self.assertEqual(check_meshes.collada_normal_problems(self.document), [])

    def test_normal_indices_of_zero_are_reported(self) -> None:
        """Set every normal index to 0: all corners but the first have another normal."""
        numbers = self.indices()
        numbers[1::2] = ['0'] * CORNERS

        self.assertEqual(self.problems_with_indices(numbers),
                         [f'<triangles material="grey"> has {CORNERS - 1} corners whose normal '
                          'index differs from their position index'])

    def test_a_missing_index_pair_is_reported(self) -> None:
        """Remove the last corner's pair: the list no longer has a pair for every corner."""
        numbers = self.indices()[:-2]

        self.assertEqual(self.problems_with_indices(numbers),
                         [f'<triangles material="grey"> has {2 * CORNERS - 2} indices, not a '
                          f'position and a normal index for each of its {CORNERS} corners'])

    def test_an_index_beyond_the_normals_is_reported(self) -> None:
        """Point the last corner at a position and a normal that the mesh doesn't have."""
        numbers = self.indices()
        numbers[-2:] = [str(CORNERS), str(CORNERS)]

        self.assertEqual(self.problems_with_indices(numbers),
                         [f'<triangles material="grey"> has 2 indices outside the {CORNERS} '
                          'positions and normals of its mesh'])

    def test_normals_as_an_input_of_the_vertices_are_reported(self) -> None:
        """Move the normals' input from the triangle list to <vertices>: reported."""
        normal_input = self.document.xpath('//c:triangles/c:input[@semantic="NORMAL"]',
                                           namespaces=NAMESPACE)[0]
        vertices = self.document.xpath('//c:mesh/c:vertices', namespaces=NAMESPACE)[0]
        normal_input.getparent().remove(normal_input)
        del normal_input.attrib['offset']
        vertices.append(normal_input)

        problems = check_meshes.collada_normal_problems(self.document)

        self.assertEqual(problems[0], "<vertices> has the inputs ['NORMAL', 'POSITION'], not "
                                      'only POSITION')
        self.assertEqual(len(problems), 2)


if __name__ == '__main__':
    unittest.main()
