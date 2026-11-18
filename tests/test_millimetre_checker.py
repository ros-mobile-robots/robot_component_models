# SPDX-FileCopyrightText: 2026 ROS Mobile Robots
# SPDX-License-Identifier: BSD-3-Clause
"""Tests of the parts that scripts/check_millimetre_collada.py checks.

`python3 -m generators --millimetre-collada DIR` writes every part's COLLADA files in
millimetres, or only the files of the parts named on the command line. The checker compares
DIR with meshes/: without --parts it expects every part's files, with --parts the files of the
named parts. Each test runs the real script in a small copy of the repository's layout, with
two parts of one tetrahedron each that mesh_export wrote in metres and in millimetres. Run from
the repository's root, with the packages of generators/requirements.txt:

  python3 -m unittest discover -s tests
"""
import os
import shutil
import subprocess
import sys
import tempfile
from typing import Sequence
import unittest

import numpy as np

from generators.mesh_export import METRE
from generators.mesh_export import MILLIMETRE
from generators.mesh_export import Model
from generators.mesh_export import phong
from generators.mesh_export import SubPart
from generators.mesh_export import Surface
from generators.mesh_export import write_dae

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CHECKER = os.path.join('scripts', 'check_millimetre_collada.py')
FIRST_PART, SECOND_PART = 'first_part', 'second_part'
PASSED, FAILED, USAGE_ERROR = 0, 1, 2
# A tetrahedron in millimetres: four points, four faces counter-clockwise seen from outside
POINTS = np.array([[1.25, 2.5, 3.75], [11.25, 2.5, 3.75], [1.25, 12.5, 3.75], [1.25, 2.5, 13.75]])
FACES = np.array([[0, 2, 1], [0, 1, 3], [1, 2, 3], [0, 3, 2]])


def tetrahedron(name: str) -> Model:
    """Return the tetrahedron as a model with the name, one sub-part and one material."""
    corners = POINTS[FACES]
    normals = np.repeat(np.cross(corners[:, 1] - corners[:, 0], corners[:, 2] - corners[:, 0]),
                        3, axis=0)
    normals /= np.linalg.norm(normals, axis=1, keepdims=True)
    surface = Surface(corners.reshape(-1, 3), normals, np.arange(3 * len(FACES)).reshape(-1, 3))
    return Model(name, 'A tetrahedron', 'The origin outside the tetrahedron.',
                 {'grey': phong((0.5, 0.5, 0.5), (0.1, 0.1, 0.1), 10.0)},
                 [SubPart('body', {'grey': surface})])


class MillimetreCheckerTest(unittest.TestCase):
    """check_millimetre_collada.py compares the parts it is told to, by default all."""

    def setUp(self) -> None:
        """Make a repository layout with the checker and two parts' files in metres."""
        self.repository = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.repository)
        os.mkdir(os.path.join(self.repository, 'scripts'))
        shutil.copyfile(os.path.join(ROOT, CHECKER), os.path.join(self.repository, CHECKER))
        self.millimetre_folder = os.path.join(self.repository, 'millimetre')
        for part in (FIRST_PART, SECOND_PART):
            os.makedirs(os.path.join(self.repository, 'meshes', part))
            write_dae(tetrahedron(part), os.path.join(self.repository, 'meshes', part,
                                                      part + '.dae'), 'a test', METRE)

    def write_millimetre_files(self, parts: Sequence[str]) -> None:
        """Write the parts' files in millimetres, as the generators' option lays them out."""
        for part in parts:
            os.makedirs(os.path.join(self.millimetre_folder, part))
            write_dae(tetrahedron(part), os.path.join(self.millimetre_folder, part,
                                                      part + '.dae'), 'a test', MILLIMETRE)

    def run_checker(self, *arguments: str) -> 'subprocess.CompletedProcess[str]':
        """Run the checker on the millimetre folder with the arguments."""
        return subprocess.run([sys.executable, CHECKER, self.millimetre_folder] + list(arguments),
                              cwd=self.repository, check=False, capture_output=True, text=True)

    def test_checker_passes_for_every_parts_files_without_a_selection(self) -> None:
        """Write both parts and check without --parts: two files, no problem."""
        self.write_millimetre_files([FIRST_PART, SECOND_PART])

        result = self.run_checker()

        self.assertEqual(result.returncode, PASSED, result.stdout + result.stderr)
        self.assertIn('2 millimetre files checked, 0 problems', result.stdout)

    def test_checker_reports_the_other_parts_as_missing_without_a_selection(self) -> None:
        """Write one part and check without --parts: the other part's file is missing."""
        self.write_millimetre_files([FIRST_PART])

        result = self.run_checker()

        self.assertEqual(result.returncode, FAILED, result.stdout + result.stderr)
        self.assertIn(f'{SECOND_PART}/{SECOND_PART}.dae: missing', result.stdout)

    def test_checker_passes_for_one_part_that_is_selected(self) -> None:
        """Write one part and check with --parts for it: one file, no problem."""
        self.write_millimetre_files([FIRST_PART])

        result = self.run_checker('--parts', FIRST_PART)

        self.assertEqual(result.returncode, PASSED, result.stdout + result.stderr)
        self.assertIn('1 millimetre files checked, 0 problems', result.stdout)

    def test_checker_checks_a_part_once_that_is_selected_twice(self) -> None:
        """Write one part and name it twice with --parts: one file is checked."""
        self.write_millimetre_files([FIRST_PART])

        result = self.run_checker('--parts', FIRST_PART, FIRST_PART)

        self.assertEqual(result.returncode, PASSED, result.stdout + result.stderr)
        self.assertIn('1 millimetre files checked, 0 problems', result.stdout)

    def test_checker_reports_the_file_of_a_part_that_is_not_selected(self) -> None:
        """Write both parts and select one: the other part's file doesn't belong there."""
        self.write_millimetre_files([FIRST_PART, SECOND_PART])

        result = self.run_checker('--parts', FIRST_PART)

        self.assertEqual(result.returncode, FAILED, result.stdout + result.stderr)
        self.assertIn(f'{SECOND_PART}/{SECOND_PART}.dae: no COLLADA file of the checked parts',
                      result.stdout)

    def test_checker_refuses_an_unknown_part(self) -> None:
        """Select a part that meshes/ doesn't hold: a usage error that names it."""
        self.write_millimetre_files([FIRST_PART])

        result = self.run_checker('--parts', 'third_part')

        self.assertEqual(result.returncode, USAGE_ERROR, result.stdout + result.stderr)
        self.assertIn('unknown parts: third_part', result.stderr)


if __name__ == '__main__':
    unittest.main()
