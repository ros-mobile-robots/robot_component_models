# SPDX-FileCopyrightText: 2026 ROS Mobile Robots
# SPDX-License-Identifier: BSD-3-Clause
"""Generate the display model of the Raspberry Pi Camera Module v2.

The model is built with CadQuery from public facts only: Raspberry Pi's mechanical drawing of
the Camera Module v2.1 (board 25 x 23.862 mm, four 2.2 mm holes, 2 mm corner radius, the 8.5 x
8.5 mm camera module, its flex connector and the 15-way ribbon connector, 20.8 x 5.5 mm) and
the documented size of about 25 x 24 x 9 mm. Heights that the drawing doesn't give are
estimates from the product photo. No logos, no printed text. Sources in
provenance/raspberry_pi_camera_v2.md.

Frame (positions in mm): the board in the yz plane with its back face at x = -1.645, the lens
looking along +x, the edge with the mounting holes 2 mm away at -z (the ribbon connector's
edge at +z, on the back). The STL and glTF files are in metres, the COLLADA file in
millimetres.
"""
from typing import List

import cadquery as cq
import numpy as np

from generators.mesh_export import classified_sub_part
from generators.mesh_export import Model
from generators.mesh_export import phong

TITLE = 'Raspberry Pi Camera Module v2'

# --- Parameters (mm) ---------------------------------------------------------------------------
NAME = 'raspberry_pi_camera_v2'
# Board (drawing): 25 mm along y, 23.862 mm along z, 1 mm thick, corners R 2
BOARD_Y, BOARD_Z, BOARD_T, CORNER_R = 25.0, 23.862, 1.0, 2.0
# Placement: the board's -y and -z edges and back face. With Remo's camera_joint, the back face
# lies 0.8 mm in front of the printed holder's plate (provenance/raspberry_pi_camera_v2.md)
Y0, Z0, X_BACK = -12.16, -12.283, -1.645
X_FRONT = X_BACK + BOARD_T
# Drawing coordinates: a from the edge next to the holes (along +z), b from the drawing's top
# edge (along +y)
HOLE_D = 2.2
HOLES_AB = [(2.0, 2.0), (2.0, 23.0), (14.5, 2.0), (14.5, 23.0)]
LENS_AB = (23.862 - 9.462, 12.5)  # the camera module's centre: 9.462 mm from the connector edge
# Camera module (8.5 x 8.5 mm in the drawing). Heights estimated, about 5.2 mm on the board
MODULE = 8.5
MODULE_TAPE, MODULE_BASE_H = 0.3, 2.4
HOLDER_D, HOLDER_H = 7.0, 1.4
BARREL_D, BARREL_H = 5.2, 1.1
GLASS_D = 2.4
# Board-to-board connector of the module's flex (drawing): a 3.6-5.8, b 7.05-15.2
FLEX_CONN = (3.6, 5.8, 7.05, 15.2, 1.0)
FLEX_T = 0.12
# Gold rings around the mounting holes and two round test pads on the front (product photo)
HOLE_PAD_D, PAD_T = 4.0, 0.035
TEST_PADS_AB = [(20.3, 3.2), (10.8, 22.6)]
TEST_PAD_D = 2.0
# 15-way ribbon connector on the back (drawing): 5.5 mm from the edge, 20.8 mm long
RIBBON_BODY = (23.862 - 5.5, 22.74, 2.1, 22.9, 2.0)    # a0, a1, b0, b1, height
RIBBON_LATCH = (22.74, 23.75, 1.6, 23.4, 2.5)
# Small parts, from the drawing. Raspberry Pi's product photo shows the front without them, so
# they sit on the back: (material, a, b, size_a, size_b, height)
BACK_PARTS = [
    ('ic', 7.8, 2.55, 2.2, 1.3, 0.9),
    ('cap', 10.0, 2.2, 0.8, 1.6, 0.5),
    ('ic', 9.75, 6.65, 2.0, 2.4, 0.8),
    ('cap', 7.25, 6.7, 1.25, 2.0, 0.6),
    ('cap', 14.3, 5.4, 1.0, 0.5, 0.35),
    ('cap', 14.3, 6.4, 1.0, 0.5, 0.35),
    ('cap', 14.3, 7.3, 1.0, 0.5, 0.35),
    ('res', 1.8, 9.7, 0.5, 1.0, 0.35),
    ('cap', 8.15, 9.85, 0.5, 1.0, 0.35),
    ('ic', 8.35, 15.6, 1.4, 2.6, 1.0),
    ('ic', 8.35, 20.4, 1.4, 2.6, 1.0),
    ('ic', 3.6, 18.6, 2.3, 2.0, 0.8),
    ('cap', 7.4, 17.6, 1.0, 0.5, 0.35),
    ('cap', 9.4, 17.6, 1.0, 0.5, 0.35),
    ('cap', 7.4, 22.6, 1.0, 0.5, 0.35),
    ('cap', 9.4, 22.6, 1.0, 0.5, 0.35),
    ('res', 1.5, 18.7, 0.5, 1.0, 0.35),
    ('cap', 3.6, 16.6, 1.0, 0.5, 0.35),
    ('cap', 15.4, 17.9, 1.1, 0.9, 0.5),
]
# Their gull-wing leads: (a, b, size_a, size_b, leads along a or b, leads per side (one, other))
LEADS = [(7.8, 2.55, 2.2, 1.3, 'b', (3, 2)),
         (8.35, 15.6, 1.4, 2.6, 'a', (3, 3)),
         (8.35, 20.4, 1.4, 2.6, 'a', (3, 3))]
TOL, ANG_TOL = 0.05, 0.6  # tessellation: absolute linear (mm) and angular (rad) limits

COLOURS = {
    'pcb': (0.10, 0.42, 0.22),  # the YAML's rgba for raspi-cam
    'fr4': (0.55, 0.56, 0.36),
    'module': (0.08, 0.08, 0.09),
    'glass': (0.05, 0.08, 0.14),
    'flex': (0.80, 0.50, 0.12),
    'connector_black': (0.10, 0.10, 0.10),
    'gold': (0.85, 0.68, 0.25),
    'connector_dark': (0.20, 0.15, 0.11),
    'silver': (0.80, 0.80, 0.82),
    'ic': (0.07, 0.07, 0.08),
    'cap': (0.72, 0.60, 0.42),
    'res': (0.10, 0.10, 0.10),
}
METALS = {'silver', 'glass', 'gold'}
MATERIALS = {name: phong(rgb, (0.35, 0.35, 0.35) if name in METALS else (0.04, 0.04, 0.04),
                         30.0 if name in METALS else 10.0, name in METALS)
             for name, rgb in COLOURS.items()}


def to_yz(a, b):
    """Map drawing coordinates (a, b) to the frame's (y, z)."""
    return Y0 + b, Z0 + a


# --- Geometry helpers --------------------------------------------------------------------------
def box(x0, x1, y0, y1, z0, z1):
    """Return an axis-aligned box from its bounds."""
    return cq.Workplane('XY').box(x1 - x0, y1 - y0, z1 - z0, centered=False) \
        .translate((x0, y0, z0))


def ab_box(a0, a1, b0, b1, x0, x1):
    """Return a box between drawing coordinates a0-a1 and b0-b1, from x0 to x1."""
    (y0, z0), (y1, z1) = to_yz(a0, b0), to_yz(a1, b1)
    return box(min(x0, x1), max(x0, x1), y0, y1, z0, z1)


def ab_centred(a, b, sa, sb, x0, x1):
    """Return a box centred on (a, b) with sizes sa (along z) and sb (along y)."""
    return ab_box(a - sa / 2, a + sa / 2, b - sb / 2, b + sb / 2, x0, x1)


def x_cylinder(a, b, d, x0, x1):
    """Return a cylinder along x through drawing position (a, b)."""
    y, z = to_yz(a, b)
    return cq.Workplane('YZ').circle(d / 2).extrude(x1 - x0).translate((x0, y, z))


# --- The parts ---------------------------------------------------------------------------------
def board():
    """Return the board with its four mounting holes."""
    plate = (cq.Workplane('YZ').rect(BOARD_Y, BOARD_Z, centered=False).extrude(BOARD_T)
             .edges('|X').fillet(CORNER_R).translate((X_BACK, Y0, Z0)))
    for a, b in HOLES_AB:
        plate = plate.cut(x_cylinder(a, b, HOLE_D, X_BACK - 1, X_FRONT + 1))
    return plate


def pads():
    """Return the gold rings around the mounting holes (both faces) and the test pads."""
    parts = []
    for a, b in HOLES_AB:
        y, z = to_yz(a, b)
        for x0, x1 in ((X_FRONT - 0.005, X_FRONT + PAD_T), (X_BACK - PAD_T, X_BACK + 0.005)):
            parts.append(('gold', cq.Workplane('YZ').circle(HOLE_PAD_D / 2).circle(HOLE_D / 2)
                          .extrude(x1 - x0).translate((x0, y, z))))
    for a, b in TEST_PADS_AB:
        parts.append(('gold', x_cylinder(a, b, TEST_PAD_D, X_FRONT - 0.005, X_FRONT + PAD_T)))
    return parts


def camera_module():
    """Return the camera module on the front: foam tape, body, holder, barrel and glass."""
    a, b = LENS_AB
    x = X_FRONT - 0.01
    parts = [('module', ab_centred(a, b, MODULE - 0.2, MODULE - 0.2, x, X_FRONT + MODULE_TAPE))]
    x = X_FRONT + MODULE_TAPE
    parts.append(('module', ab_centred(a, b, MODULE, MODULE, x - 0.01, x + MODULE_BASE_H)
                  .edges('|X').chamfer(0.3).faces('>X').edges().chamfer(0.2)))
    x += MODULE_BASE_H
    parts.append(('module', x_cylinder(a, b, HOLDER_D, x - 0.01, x + HOLDER_H)
                  .faces('>X').edges().chamfer(0.25)))
    x += HOLDER_H
    parts.append(('module', x_cylinder(a, b, BARREL_D, x - 0.01, x + BARREL_H)
                  .faces('>X').edges().chamfer(0.2)))
    x += BARREL_H
    parts.append(('glass', x_cylinder(a, b, GLASS_D, x - 0.05, x + 0.02)))
    return parts


def flex():
    """Return the module's flex: its connector, the stiffener over it and the strip."""
    a0, a1, b0, b1, h = FLEX_CONN
    parts = [('connector_dark', ab_box(a0, a1, b0, b1, X_FRONT - 0.01, X_FRONT + h))]
    parts.append(('module', ab_box(a0 - 0.3, a1 + 0.3, b0 + 0.25, b1 - 0.25, X_FRONT + h - 0.01,
                                   X_FRONT + h + 0.2)))
    # The flex itself, black like its cover in the product photo, with its orange edge showing
    # next to the connector
    lens_a = LENS_AB[0]
    parts.append(('module', ab_box(a1 + 0.1, lens_a - 2.0, 10.5, 14.0, X_FRONT - 0.01,
                                   X_FRONT + FLEX_T)))
    parts.append(('module', ab_box(a1 + 0.1, a1 + 0.3, 10.5, 14.0, X_FRONT - 0.01,
                                   X_FRONT + h + 0.2)))
    parts.append(('flex', ab_box(a1 + 0.3, a1 + 0.55, b0 + 0.25, b1 - 0.25, X_FRONT - 0.01,
                                 X_FRONT + h + 0.1)))
    return parts


def ribbon_connector():
    """Return the 15-way ribbon connector on the back: body, latch and solder tabs."""
    a0, a1, b0, b1, h = RIBBON_BODY
    parts = [('connector_black', ab_box(a0, a1, b0, b1, X_BACK + 0.01, X_BACK - h))]
    a0, a1, b0, b1, h = RIBBON_LATCH
    parts.append(('connector_dark', ab_box(a0, a1, b0, b1, X_BACK + 0.01, X_BACK - h)))
    for bt in ((1.3, 2.1), (22.9, 23.7)):
        parts.append(('silver', ab_box(19.0, 21.0, bt[0], bt[1], X_BACK + 0.01, X_BACK - 0.6)))
    return parts


def back_parts():
    """Return the small parts on the back, with the leads of the SOT packages."""
    parts = []
    for mat, a, b, sa, sb, h in BACK_PARTS:
        part = ab_centred(a, b, sa, sb, X_BACK + 0.01, X_BACK - h)
        if min(sa, sb) >= 1.2:
            part = part.faces('<X').edges().chamfer(0.08)
        parts.append((mat, part))
    for a, b, sa, sb, axis, counts in LEADS:
        for side, n in zip((-1, 1), counts):
            for k in range(n):
                along = (k - (n - 1) / 2) * 0.95
                if axis == 'b':
                    la, lb = a + along, b + side * (sb / 2 + 0.25)
                    size = (0.4, 0.55)
                else:
                    la, lb = a + side * (sa / 2 + 0.25), b + along
                    size = (0.55, 0.4)
                parts.append(('silver', ab_centred(la, lb, *size, X_BACK + 0.01,
                                                   X_BACK - 0.15)))
    return parts


def pcb_material(tri):
    """Return the material of each board triangle: solder mask on the faces, FR4 on the edges."""
    normal = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    normal /= np.linalg.norm(normal, axis=1, keepdims=True)
    mats = np.full(len(tri), 'pcb', dtype=object)
    mats[np.abs(normal[:, 0]) < 0.5] = 'fr4'
    return mats


# Parts whose triangles get more than one material
CLASSIFIERS = {'pcb': pcb_material}


# --- Mesh --------------------------------------------------------------------------------------
FRAME = ('The board in the yz plane, its back face at x = -1.645 mm, its edges at y = -12.16 '
         'and z = -12.283 mm, the lens looking along +x, the mounting holes at (y, z) = '
         '(-10.16, -10.283), (-10.16, 2.217), (10.84, -10.283) and (10.84, 2.217) mm.')


def build() -> List[Model]:
    """Return the camera's mesh, one sub-part per group of parts."""
    groups = [('board', [('pcb', board())]), ('pads', pads()),
              ('camera_module', camera_module()), ('flex', flex()),
              ('ribbon_connector', ribbon_connector()), ('back_parts', back_parts())]
    sub_parts = [classified_sub_part(name, pieces, list(MATERIALS), TOL, ANG_TOL, CLASSIFIERS)
                 for name, pieces in groups]
    return [Model(NAME, TITLE, FRAME, MATERIALS, sub_parts)]
