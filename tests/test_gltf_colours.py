# SPDX-FileCopyrightText: 2026 ROS Mobile Robots
# SPDX-License-Identifier: BSD-3-Clause
"""Tests of the base colours that generators/mesh_export.py writes into the glTF files.

A model's colours are sRGB values, as the COLLADA files hold them. glTF defines a material's
base colour factor as linear values, so the exporter converts red, green and blue with the
sRGB transfer function and leaves alpha as it is. The expected numbers are written out here.
They don't come from the exporter or from the released files, so the tests still fail if the
conversion is lost and the files and the manifest are regenerated without it. Run from the
repository's root, with the packages of generators/requirements.txt:

  python3 -m unittest discover -s tests
"""
import os
import shutil
import tempfile
from typing import List
import unittest

import numpy as np

from generators.mesh_export import gltf_material
from generators.mesh_export import Model
from generators.mesh_export import phong
from generators.mesh_export import srgb_to_linear
from generators.mesh_export import SubPart
from generators.mesh_export import Surface
from generators.mesh_export import write_glb
from scripts import check_meshes

# sRGB components and their linear values (IEC 61966-2-1), at the files' six decimal places.
# Below 0.04045 the transfer function is a division by 12.92: 0.03 / 12.92 = 0.002322.
# Above it: ((0.5 + 0.055) / 1.055) ** 2.4 = 0.214041.
DARK, DARK_LINEAR = 0.03, 0.002322
MIDTONE, MIDTONE_LINEAR = 0.5, 0.214041
WHITE = 1.0
HALF_TRANSPARENT = 0.5
SRGB_COLOUR = (DARK, MIDTONE, WHITE, HALF_TRANSPARENT)
LINEAR_BASE_COLOUR = [DARK_LINEAR, MIDTONE_LINEAR, WHITE, HALF_TRANSPARENT]
FILE_DECIMALS = 6


def one_triangle() -> Surface:
    """Return a surface of one triangle in the xy plane, in millimetres."""
    return Surface(np.array([[0.0, 0.0, 0.0], [10.0, 0.0, 0.0], [0.0, 10.0, 0.0]]),
                   np.array([[0.0, 0.0, 1.0]] * 3), np.array([[0, 1, 2]]))


class SrgbToLinearTest(unittest.TestCase):
    """srgb_to_linear() is the sRGB transfer function for one colour component."""

    def test_srgb_to_linear_divides_a_dark_component_by_12_92(self) -> None:
        """Convert 0.03, below the function's threshold: 0.002322."""
        self.assertAlmostEqual(srgb_to_linear(DARK), DARK_LINEAR, places=FILE_DECIMALS)

    def test_srgb_to_linear_follows_the_power_curve_for_a_midtone(self) -> None:
        """Convert 0.5: 0.214041."""
        self.assertAlmostEqual(srgb_to_linear(MIDTONE), MIDTONE_LINEAR, places=FILE_DECIMALS)

    def test_srgb_to_linear_keeps_black_and_white(self) -> None:
        """Convert 0 and 1: both stay."""
        self.assertEqual([srgb_to_linear(0.0), srgb_to_linear(WHITE)], [0.0, WHITE])


class GltfMaterialTest(unittest.TestCase):
    """gltf_material() and write_glb() write a material's colour as linear values."""

    def material_in_written_file(self) -> List[float]:
        """Return the base colour factor in the glTF file of a model with the sRGB colour."""
        folder = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, folder)
        path = os.path.join(folder, 'triangle.glb')
        model = Model('triangle', 'A triangle', 'The origin at a corner.',
                      {'paint': phong(SRGB_COLOUR, (0.1, 0.1, 0.1), 10.0)},
                      [SubPart('face', {'paint': one_triangle()})])
        write_glb(model, path)
        materials = check_meshes.gltf_document(path)['materials']
        self.assertEqual([material['name'] for material in materials], ['paint'])
        factor: List[float] = materials[0]['pbrMetallicRoughness']['baseColorFactor']
        return factor

    def test_gltf_material_converts_red_green_and_blue_and_keeps_alpha(self) -> None:
        """Convert a half-transparent colour: linear red, green and blue, alpha 0.5."""
        material = gltf_material('paint', phong(SRGB_COLOUR, (0.1, 0.1, 0.1), 10.0))

        self.assertEqual(material['pbrMetallicRoughness'],
                         {'baseColorFactor': LINEAR_BASE_COLOUR, 'metallicFactor': 0.0,
                          'roughnessFactor': 0.408})

    def test_written_gltf_file_has_the_linear_base_colour(self) -> None:
        """Write a model with the sRGB colour: the file's material has the linear values."""
        self.assertEqual(self.material_in_written_file(), LINEAR_BASE_COLOUR)


if __name__ == '__main__':
    unittest.main()
