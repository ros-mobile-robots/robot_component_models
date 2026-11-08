# SPDX-FileCopyrightText: 2026 ROS Mobile Robots
# SPDX-License-Identifier: BSD-3-Clause
"""Generators of the package's display models, one module per part.

Each part module has a TITLE and a build() function that returns the part's meshes as
mesh_export.Model objects. `python3 -m generators` writes all meshes and meshes/manifest.yaml.
"""

# The parts in the order of the manifest and the README. Each name is a module in this
# package, a folder in meshes/ and a page in provenance/.
PARTS = (
    'swivel_caster_30mm',
    'rplidar_a2',
    'raspberry_pi_4b',
    'jetson_nano_dev_kit_b01',
    'teensy_4_0',
    'breadboard_400_point',
    'raspberry_pi_camera_v2',
    'tt_motor_wheel_65mm',
)
