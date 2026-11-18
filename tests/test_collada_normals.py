# SPDX-FileCopyrightText: 2026 ROS Mobile Robots
# SPDX-License-Identifier: BSD-3-Clause
"""Tests of how scripts/check_meshes.py checks the normals of a COLLADA file.

mesh_export writes a COLLADA file's normals as an input of its triangle lists: every corner is
its position's index followed by its normal's index, and the two are the same number. A file
can declare these inputs and still give a corner another vertex's normal, or read the
positions as normals. The tests damage the indices or the inputs' sources of a file that
mesh_export wrote, and the checker has to report each damage. Run from the repository's root,
with the packages of generators/requirements.txt:

  python3 -m unittest discover -s tests
"""
import copy
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
GEOMETRY = 'tetrahedron-body'  # the id that mesh_export gives the one sub-part's geometry
POSITION_INPUT = f'POSITION from #{GEOMETRY}-positions'
VERTEX_INPUT = f'VERTEX from #{GEOMETRY}-vertices at offset 0'
NORMAL_INPUT = f'NORMAL from #{GEOMETRY}-normals at offset 1'


def input_semantics(element: Any) -> List[str]:
    """Return the semantics of an element's inputs, in the file's order."""
    return [bound.get('semantic') for bound in element.findall('c:input', NAMESPACE)]


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

    def test_indices_that_are_not_numbers_are_reported(self) -> None:
        """Replace the last normal index with 'x': reported as a problem, not an error."""
        numbers = self.indices()
        numbers[-1] = 'x'

        self.assertEqual(self.problems_with_indices(numbers),
                         ['<triangles material="grey"> has a count or indices that aren\'t '
                          'whole numbers'])

    def test_normals_as_an_input_of_the_vertices_are_reported(self) -> None:
        """Move the normals' input from the triangle list to <vertices>: reported."""
        normal_input = self.document.xpath('//c:triangles/c:input[@semantic="NORMAL"]',
                                           namespaces=NAMESPACE)[0]
        vertices = self.document.xpath('//c:mesh/c:vertices', namespaces=NAMESPACE)[0]
        normal_input.getparent().remove(normal_input)
        del normal_input.attrib['offset']
        vertices.append(normal_input)

        problems = check_meshes.collada_normal_problems(self.document)

        self.assertEqual(problems, [
            f'<vertices id="{GEOMETRY}-vertices"> has the inputs '
            f"{[POSITION_INPUT, f'NORMAL from #{GEOMETRY}-normals']}, not {[POSITION_INPUT]} "
            f'with the id {GEOMETRY}-vertices',
            f'<triangles material="grey"> has the inputs {[VERTEX_INPUT]}, not '
            f'{[VERTEX_INPUT, NORMAL_INPUT]}'])


class ColladaSourceBindingTest(unittest.TestCase):
    """collada_normal_problems() follows every input to the source that it names.

    In each test the arrays of positions and normals and every index pair stay as mesh_export
    wrote them. Only one reference changes, so a check of the inputs' semantics, offsets and
    indices alone would accept the file.
    """

    def setUp(self) -> None:
        """Write the tetrahedron's COLLADA file and parse it."""
        folder = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, folder)
        path = os.path.join(folder, 'tetrahedron.dae')
        write_dae(tetrahedron(), path, 'tests/test_collada_normals.py')
        self.document = etree.parse(path)

    def element(self, path: str) -> Any:
        """Return the file's one element at the XPath."""
        elements = self.document.xpath(path, namespaces=NAMESPACE)
        self.assertEqual(len(elements), 1)
        return elements[0]

    def test_geometry_has_the_expected_id(self) -> None:
        """Read the geometry's id: the name that the tests' expected inputs are built from."""
        self.assertEqual(self.element('//c:geometry').get('id'), GEOMETRY)

    def test_normals_read_from_the_positions_are_reported(self) -> None:
        """Bind the triangle list's NORMAL input to the positions' source: reported."""
        self.element('//c:triangles/c:input[@semantic="NORMAL"]').set(
            'source', f'#{GEOMETRY}-positions')

        self.assertEqual(
            check_meshes.collada_normal_problems(self.document),
            [f'<triangles material="grey"> has the inputs '
             f"{[VERTEX_INPUT, f'NORMAL from #{GEOMETRY}-positions at offset 1']}, not "
             f'{[VERTEX_INPUT, NORMAL_INPUT]}'])

    def test_positions_read_from_the_normals_are_reported(self) -> None:
        """Bind the POSITION input of <vertices> to the normals' source: reported."""
        self.element('//c:vertices/c:input').set('source', f'#{GEOMETRY}-normals')

        self.assertEqual(
            check_meshes.collada_normal_problems(self.document),
            [f'<vertices id="{GEOMETRY}-vertices"> has the inputs '
             f"{[f'POSITION from #{GEOMETRY}-normals']}, not {[POSITION_INPUT]} with the id "
             f'{GEOMETRY}-vertices'])

    def test_corners_read_from_a_source_instead_of_the_vertices_are_reported(self) -> None:
        """Bind the triangle list's VERTEX input to the positions' source: reported."""
        self.element('//c:triangles/c:input[@semantic="VERTEX"]').set(
            'source', f'#{GEOMETRY}-positions')

        self.assertEqual(
            check_meshes.collada_normal_problems(self.document),
            [f'<triangles material="grey"> has the inputs '
             f"{[f'VERTEX from #{GEOMETRY}-positions at offset 0', NORMAL_INPUT]}, not "
             f'{[VERTEX_INPUT, NORMAL_INPUT]}'])

    def test_a_normals_accessor_that_reads_the_positions_array_is_reported(self) -> None:
        """Let the normals' accessor read the positions' array: reported."""
        self.element(f'//c:source[@id="{GEOMETRY}-normals"]//c:accessor').set(
            'source', f'#{GEOMETRY}-positions-array')

        self.assertEqual(
            check_meshes.collada_normal_problems(self.document),
            [f"the source {GEOMETRY}-normals's accessor reads #{GEOMETRY}-positions-array, "
             f"not the source's own array #{GEOMETRY}-normals-array"])

    def test_a_normals_accessor_that_starts_behind_the_first_number_is_reported(self) -> None:
        """Give the normals' accessor an offset of 3: its last vector lies past the array."""
        self.element(f'//c:source[@id="{GEOMETRY}-normals"]//c:accessor').set('offset', '3')

        self.assertEqual(
            check_meshes.collada_normal_problems(self.document),
            [f"the source {GEOMETRY}-normals's accessor starts at the array's number 3, not at "
             'its first number'])

    def test_an_accessor_with_an_offset_of_zero_has_no_problems(self) -> None:
        """Write the offset that the accessor has by default, 0: no problem."""
        self.element(f'//c:source[@id="{GEOMETRY}-normals"]//c:accessor').set('offset', '0')

        self.assertEqual(check_meshes.collada_normal_problems(self.document), [])

    def test_an_accessor_offset_that_is_not_a_number_is_reported(self) -> None:
        """Give the normals' accessor the offset 'x': reported as a problem, not an error."""
        self.element(f'//c:source[@id="{GEOMETRY}-normals"]//c:accessor').set('offset', 'x')

        self.assertEqual(
            check_meshes.collada_normal_problems(self.document),
            [f"the source {GEOMETRY}-normals's accessor count '{CORNERS}', accessor offset "
             f"'x' and array count '{3 * CORNERS}' aren't all whole numbers"])

    def test_inputs_in_the_other_order_have_no_problems(self) -> None:
        """Put the NORMAL input before the VERTEX input: the offsets still pair the indices."""
        triangles = self.element('//c:triangles')
        vertex_input = self.element('//c:triangles/c:input[@semantic="VERTEX"]')
        normal_input = self.element('//c:triangles/c:input[@semantic="NORMAL"]')
        triangles.remove(normal_input)
        vertex_input.addprevious(normal_input)

        self.assertEqual(input_semantics(triangles), ['NORMAL', 'VERTEX'])
        self.assertEqual(check_meshes.collada_normal_problems(self.document), [])

    def test_a_second_normal_input_is_reported(self) -> None:
        """Copy the NORMAL input: a triangle list has each of its two inputs once."""
        normal_input = self.element('//c:triangles/c:input[@semantic="NORMAL"]')
        normal_input.addnext(copy.deepcopy(normal_input))

        self.assertEqual(
            check_meshes.collada_normal_problems(self.document),
            [f'<triangles material="grey"> has the inputs '
             f'{[VERTEX_INPUT, NORMAL_INPUT, NORMAL_INPUT]}, not '
             f'{[VERTEX_INPUT, NORMAL_INPUT]}'])

    def test_a_missing_normals_source_is_reported(self) -> None:
        """Rename the normals' source: the mesh has no source with the expected id."""
        self.element(f'//c:source[@id="{GEOMETRY}-normals"]').set('id', 'other-normals')

        self.assertEqual(
            check_meshes.collada_normal_problems(self.document),
            [f'the mesh has 0 sources with the id {GEOMETRY}-normals, not one'])

    def test_fewer_normals_than_positions_are_reported(self) -> None:
        """Take the last normal out of its source: not one normal for each position."""
        array = self.element(f'//c:source[@id="{GEOMETRY}-normals"]/c:float_array')
        array.text = ' '.join(array.text.split()[:-3])
        array.set('count', str(3 * (CORNERS - 1)))
        self.element(f'//c:source[@id="{GEOMETRY}-normals"]//c:accessor').set(
            'count', str(CORNERS - 1))

        self.assertEqual(
            check_meshes.collada_normal_problems(self.document),
            [f'the mesh has {CORNERS} positions and {CORNERS - 1} normals, not one normal for '
             'each position'])


if __name__ == '__main__':
    unittest.main()
