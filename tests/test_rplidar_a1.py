# SPDX-FileCopyrightText: 2026 ROS Mobile Robots
# SPDX-License-Identifier: BSD-3-Clause
"""Tests of how generators/rplidar_a1.py groups a sub-part's solids by material.

The scanner's sub_part() takes a sub-part's solids by material name and writes one surface per
material, in the order of the scanner's materials. A name that isn't one of them, such as a
misspelled one, has to stop the generation: its solids would be missing from the files. Run
from the repository's root, with the packages of generators/requirements.txt:

  python3 -m unittest discover -s tests
"""
import unittest

import cadquery as cq

from generators import rplidar_a1

CUBE_TRIANGLES = 12  # two per face


def cube() -> cq.Workplane:
    """Return a cube with an edge of 1 mm."""
    return cq.Workplane('XY').box(1.0, 1.0, 1.0)


class SubPartTest(unittest.TestCase):
    """sub_part() accepts the scanner's material names and no others."""

    def test_sub_part_has_a_surface_for_each_material_of_its_solids(self) -> None:
        """Give housings and contacts under their names: both surfaces, in the materials' order."""
        part = rplidar_a1.sub_part('connectors', {'contact': [cube()], 'connector': [cube()]})

        self.assertEqual(list(part.surfaces), ['connector', 'contact'])
        self.assertEqual([len(surface.triangles) for surface in part.surfaces.values()],
                         [CUBE_TRIANGLES, CUBE_TRIANGLES])

    def test_sub_part_raises_a_value_error_for_a_misspelled_material(self) -> None:
        """Give the contacts as "contacts": an error names the sub-part and the name."""
        with self.assertRaisesRegex(ValueError, "sub-part 'connectors'.*contacts"):
            rplidar_a1.sub_part('connectors', {'connector': [cube()], 'contacts': [cube()]})


if __name__ == '__main__':
    unittest.main()
