# SPDX-FileCopyrightText: 2026 ROS Mobile Robots
# SPDX-License-Identifier: BSD-3-Clause
"""Generate the display model of a 400-point solderless breadboard (half size).

The model is built with CadQuery from public facts only: BusBoard's BB400 datasheet (layout of
the 300-point terminal area and the four 25-point power rails, 0.1 in hole pitch, 8.5 mm body),
the 0.1 in grid of the boards that plug into it on Remo (PJRC's Teensy 4.0 drawing, Adafruit's
BNO055 breakout layout) and their positions in remo_description's URDF. No logos, no printed
numbers or letters. Sources in provenance/breadboard_400_point.md.

Frame: millimetres in the STL, metres in the COLLADA and glTF files, the bottom at z = 0, the
centre channel along y, the dovetail tabs on the -x and -y sides.
"""
from typing import List

import cadquery as cq
import numpy as np

from generators.mesh_export import classified_sub_part
from generators.mesh_export import Model
from generators.mesh_export import phong

TITLE = '400-point solderless breadboard'

# --- Parameters (mm) ---------------------------------------------------------------------------
NAME = 'breadboard_400_point'
PITCH = 2.54
# The hole grid. CHANNEL_X: the centre channel's middle. COLUMN_Y0: the y of one hole column.
# The others follow at PITCH. Both put the Teensy's and the BNO055's header pins, at their
# URDF positions, into holes (provenance/breadboard_400_point.md, "Hole grid").
CHANNEL_X = -0.16
COLUMN_Y0 = -0.335
FIRST_COLUMN = -16  # the 30 columns are COLUMN_Y0 + PITCH * n, n = FIRST_COLUMN ... + 29
COLUMNS = 30
# Terminal rows (a-e, f-j) at 0.15, 0.25 ... 0.55 in from the channel's middle, power rails at
# 0.8 and 0.9 in (BusBoard BB400 drawing), each rail 25 holes in 5 groups of 5
ROW_OFFSETS = [PITCH * (1.5 + k) for k in range(5)]
RAIL_OFFSETS = [PITCH * 8, PITCH * 9]
RAIL_GROUPS, RAIL_GROUP_HOLES, RAIL_GROUP_STEP = 5, 5, 6
# Body: 82.5 x 54.5 x 8.5 mm on a 1.0 mm adhesive foam backing, 9.5 mm in all
LENGTH, WIDTH, BODY_H, TAPE_H = 82.5, 54.5, 8.5, 1.0
TAPE_INSET = 0.3
TOP_CHAMFER = 0.4
HOLE_W, HOLE_DEPTH = 1.1, 1.6  # square openings, below them the contact (dark)
CHANNEL_W, CHANNEL_DEPTH = 2.6, 4.5
STRIP_X = 0.68 * 25.4  # the rail strips' seams, 1.36 in apart (BB400 drawing)
SEAM_W, SEAM_DEPTH = 0.4, 1.0
STRIPE_W, STRIPE_T, STRIPE_LEN = 0.6, 0.03, 74.0
STRIPE_OFFSETS = {'red': (-24.8, 18.5), 'blue': (-18.5, 24.8)}  # from CHANNEL_X
# Dovetails: tabs on -x and -y, matching slots on +x and +y
TAB_DEPTH, TAB_NECK, TAB_TIP = 1.4, 2.6, 3.6
TAB_Z = (TAPE_H, TAPE_H + 7.5)
SLOT_CLEARANCE = 0.2
LONG_SIDE_TABS = (-25.4, 25.4)        # y offsets from the board's centre
SHORT_SIDE_TABS = (-22.0, -8.9, 8.9, 22.0)  # x offsets from CHANNEL_X
TOL, ANG_TOL = 0.05, 0.6  # tessellation: absolute linear (mm) and angular (rad) limits


COLOURS = {
    'white': (0.93, 0.93, 0.91),  # the YAML's rgba for the breadboard
    'hole': (0.30, 0.30, 0.31),
    'tape': (0.84, 0.84, 0.80),
    'red': (0.85, 0.12, 0.10),
    'blue': (0.12, 0.28, 0.72),
}

MATERIALS = {name: phong(rgb, (0.04, 0.04, 0.04), 10.0) for name, rgb in COLOURS.items()}

TOP_Z = TAPE_H + BODY_H
CENTRE_Y = COLUMN_Y0 + PITCH * (FIRST_COLUMN + (COLUMNS - 1) / 2)


def column_ys():
    """Return the y of the 30 terminal columns."""
    return [COLUMN_Y0 + PITCH * (FIRST_COLUMN + n) for n in range(COLUMNS)]


def rail_ys():
    """Return the y of the 25 holes of a power rail, centred between the columns."""
    first = -(RAIL_GROUP_STEP * (RAIL_GROUPS - 1) + RAIL_GROUP_HOLES - 1) / 2
    return [CENTRE_Y + PITCH * (first + RAIL_GROUP_STEP * g + h)
            for g in range(RAIL_GROUPS) for h in range(RAIL_GROUP_HOLES)]


def hole_centres():
    """Return the (x, y) of all 400 holes."""
    pts = [(CHANNEL_X + s * off, y) for s in (-1, 1) for off in ROW_OFFSETS
           for y in column_ys()]
    pts += [(CHANNEL_X + s * off, y) for s in (-1, 1) for off in RAIL_OFFSETS for y in rail_ys()]
    return pts


# --- Geometry helpers --------------------------------------------------------------------------
def box(x0, x1, y0, y1, z0, z1):
    """Return an axis-aligned box from its bounds."""
    return cq.Workplane('XY').box(x1 - x0, y1 - y0, z1 - z0, centered=False) \
        .translate((x0, y0, z0))


def dovetail(neck, tip, depth, z0, z1):
    """Return a dovetail prism: neck on the y = 0 line, widening to the tip at y = -depth."""
    pts = [(-neck / 2, 0.0), (neck / 2, 0.0), (tip / 2, -depth), (-tip / 2, -depth)]
    return cq.Workplane('XY').polyline(pts).close().extrude(z1 - z0).translate((0, 0, z0))


# --- The parts ---------------------------------------------------------------------------------
def body():
    """Return the white body with holes, channel, seams, tabs and slots."""
    x0, x1 = CHANNEL_X - WIDTH / 2, CHANNEL_X + WIDTH / 2
    y0, y1 = CENTRE_Y - LENGTH / 2, CENTRE_Y + LENGTH / 2
    solid = box(x0, x1, y0, y1, TAPE_H, TOP_Z).faces('>Z').edges().chamfer(TOP_CHAMFER)
    # Tabs (-x and -y) and slots (+x and +y)
    for dy in LONG_SIDE_TABS:
        tab = dovetail(TAB_NECK, TAB_TIP, TAB_DEPTH + 0.01, *TAB_Z)
        solid = solid.union(tab.rotate((0, 0, 0), (0, 0, 1), -90)
                            .translate((x0 + 0.01, CENTRE_Y + dy, 0)))
        slot = dovetail(TAB_NECK + SLOT_CLEARANCE, TAB_TIP + SLOT_CLEARANCE,
                        TAB_DEPTH + SLOT_CLEARANCE, TAPE_H - 0.1, TOP_Z + 0.1)
        solid = solid.cut(slot.rotate((0, 0, 0), (0, 0, 1), -90)
                          .translate((x1 + 0.01, CENTRE_Y + dy, 0)))
    for dx in SHORT_SIDE_TABS:
        tab = dovetail(TAB_NECK, TAB_TIP, TAB_DEPTH + 0.01, *TAB_Z)
        solid = solid.union(tab.translate((CHANNEL_X + dx, y0 + 0.01, 0)))
        slot = dovetail(TAB_NECK + SLOT_CLEARANCE, TAB_TIP + SLOT_CLEARANCE,
                        TAB_DEPTH + SLOT_CLEARANCE, TAPE_H - 0.1, TOP_Z + 0.1)
        solid = solid.cut(slot.translate((CHANNEL_X + dx, y1 + 0.01, 0)))
    # Centre channel and the seams between the terminal block and the rail strips
    solid = solid.cut(box(CHANNEL_X - CHANNEL_W / 2, CHANNEL_X + CHANNEL_W / 2, y0 - 1, y1 + 1,
                          TOP_Z - CHANNEL_DEPTH, TOP_Z + 1))
    for s in (-1, 1):
        xs = CHANNEL_X + s * STRIP_X
        solid = solid.cut(box(xs - SEAM_W / 2, xs + SEAM_W / 2, y0 - 1, y1 + 1,
                              TOP_Z - SEAM_DEPTH, TOP_Z + 1))
    # The 400 holes
    holes = (cq.Workplane('XY').workplane(offset=TOP_Z - HOLE_DEPTH)
             .pushPoints(hole_centres()).rect(HOLE_W, HOLE_W).extrude(HOLE_DEPTH + 1))
    return solid.cut(holes)


def tape():
    """Return the adhesive foam backing under the body."""
    return box(CHANNEL_X - WIDTH / 2 + TAPE_INSET, CHANNEL_X + WIDTH / 2 - TAPE_INSET,
               CENTRE_Y - LENGTH / 2 + TAPE_INSET, CENTRE_Y + LENGTH / 2 - TAPE_INSET,
               0.0, TAPE_H + 0.01)


def stripes():
    """Return the red and blue lines along the power rails, by colour."""
    out = []
    for colour, offsets in STRIPE_OFFSETS.items():
        for off in offsets:
            x = CHANNEL_X + off
            out.append((colour, box(x - STRIPE_W / 2, x + STRIPE_W / 2,
                                    CENTRE_Y - STRIPE_LEN / 2, CENTRE_Y + STRIPE_LEN / 2,
                                    TOP_Z - 0.01, TOP_Z + STRIPE_T)))
    return out


def body_material(tri):
    """Return the material of each body triangle: white, the holes' walls and floors dark."""
    normal = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    normal /= np.linalg.norm(normal, axis=1, keepdims=True)
    centre = tri.mean(axis=1)
    holes = np.array(hole_centres())
    near = np.zeros(len(tri), dtype=bool)
    for start in range(0, len(tri), 2000):  # in chunks, to keep the distance matrix small
        d = np.linalg.norm(centre[start:start + 2000, None, :2] - holes[None], axis=2)
        near[start:start + 2000] = d.min(axis=1) < HOLE_W
    floor = (normal[:, 2] > 0.9) & (np.abs(centre[:, 2] - (TOP_Z - HOLE_DEPTH)) < 1e-3)
    wall = (np.abs(normal[:, 2]) < 0.5) & (centre[:, 2] > TOP_Z - HOLE_DEPTH - 1e-3)
    mats = np.full(len(tri), 'white', dtype=object)
    mats[near & (floor | wall)] = 'hole'
    return mats


# Parts whose triangles get more than one material
CLASSIFIERS = {'white': body_material}


# --- Mesh --------------------------------------------------------------------------------------
FRAME = ('The bottom of the foam backing at z = 0, the top at z = 9.5 mm, the centre channel '
         'along y at x = -0.16 mm, the hole columns at y = -0.335 mm + n * 2.54 mm, the dovetail '
         'tabs on -x and -y.')


def build() -> List[Model]:
    """Return the breadboard's mesh: body, foam backing and the rails' coloured lines."""
    groups = [('body', [('white', body())]), ('backing', [('tape', tape())]),
              ('rail_lines', stripes())]
    sub_parts = [classified_sub_part(name, pieces, list(MATERIALS), TOL, ANG_TOL, CLASSIFIERS)
                 for name, pieces in groups]
    return [Model(NAME, TITLE, FRAME, MATERIALS, sub_parts)]
