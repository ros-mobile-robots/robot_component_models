# SPDX-FileCopyrightText: 2026 ROS Mobile Robots
# SPDX-License-Identifier: BSD-3-Clause
"""Generate the display model of a Teensy 4.0 with two 14-pin headers, as on Remo's breadboard.

The model is built with CadQuery from public facts only: PJRC's Teensy 4.0 dimension drawing,
parts placement and product photos, NXP's package drawing of the i.MX RT1062 (10 x 10 x 1.3 mm
MAPBGA) and Winbond's W25Q16JV datasheet (USON 2 x 3 x 0.6 mm). No logos, no printed text.
Sources and measurements in provenance/teensy_4_0.md.

Frame: millimetres in the STL, metres in the COLLADA and glTF files, the board's bottom face at
z = 0, the long axis along x with the USB connector at -x, the row with the inner pin hole at +y.
"""
from typing import List
from typing import Tuple

import cadquery as cq
import numpy as np

from generators.mesh_export import classified_sub_part
from generators.mesh_export import Model
from generators.mesh_export import phong

TITLE = 'Teensy 4.0 with two 14-pin headers'

Piece = Tuple[str, cq.Workplane]

# --- Parameters (mm) ---------------------------------------------------------------------------
NAME = 'teensy_4-0'
PITCH = 2.54
BOARD_L, BOARD_W, BOARD_T = 35.56, 17.78, 1.57  # PJRC drawing: 1.4 x 0.7 in, 1.57 mm thick
CORNER_R = 0.4
# Placement in the frame: the far end (opposite the USB connector) at x = FAR_END_X and the
# board's centre line at y = CENTRE_Y (frame alignment, see provenance/teensy_4_0.md)
FAR_END_X, CENTRE_Y = 21.631, -0.032
HOLE_D = 0.965  # PJRC: finished hole diameter 0.965 mm, 34 locations
PAD_D, PAD_T = 1.9, 0.035  # plated ring around each hole, top and bottom
# Pin header: plastic spacer under the board, pins 0.64 mm square (0.025 in posts)
SPACER_H = 2.5
PIN_W = 0.64
PIN_BELOW = 6.0   # pin length below the spacer, into the breadboard
PIN_ABOVE = 1.43  # pin length above the board's top face (3.0 mm short end minus the board)
SOLDER_D0, SOLDER_D1, SOLDER_H = 1.7, 0.9, 0.6
SOLDER_T = 0.1  # the solder on the pads of the small parts
WITH_HEADER = True  # False: the bare board, without the two headers
TOL, ANG_TOL = 0.05, 0.6  # tessellation: absolute linear (mm) and angular (rad) limits

# Colours (r, g, b), close to the real board. 'pcb' is the YAML's rgba for the Teensy.
COLOURS = {
    'pcb': (0.10, 0.42, 0.22),
    'fr4': (0.55, 0.56, 0.36),
    'gold': (0.85, 0.68, 0.25),
    'solder': (0.78, 0.78, 0.80),
    'silver': (0.80, 0.80, 0.82),
    'ic': (0.07, 0.07, 0.08),
    'substrate': (0.20, 0.22, 0.12),
    'plastic_black': (0.06, 0.06, 0.06),
    'cap': (0.72, 0.60, 0.42),
    'res': (0.10, 0.10, 0.10),
    'ceramic': (0.82, 0.80, 0.74),
    'ceramic_grey': (0.62, 0.62, 0.62),
    'copper': (0.55, 0.30, 0.16),
    'usb_slot': (0.45, 0.43, 0.41),
    'ic_mark': (0.20, 0.20, 0.22),
    'button_frame': (0.72, 0.72, 0.74),
    'button_cap': (0.93, 0.86, 0.66),
    'inductor': (0.40, 0.40, 0.42),
    'led_orange': (0.98, 0.80, 0.55),
    'led_red': (0.95, 0.68, 0.62),
}
METALS = {'gold', 'solder', 'silver', 'button_frame', 'copper'}

MATERIALS = {name: phong(rgb, (0.35, 0.35, 0.35) if name in METALS else (0.04, 0.04, 0.04),
                         30.0 if name in METALS else 10.0, name in METALS)
             for name, rgb in COLOURS.items()}


def to_xy(u, v):
    """Map board coordinates to the frame.

    u: distance from the USB end (mm), v: distance from the +y edge (mm).
    """
    return FAR_END_X - BOARD_L + u, CENTRE_Y + BOARD_W / 2 - v


# --- Positions on the board (u from the USB end, v from the +y edge), from PJRC's drawing ----
ROW_V = (PITCH / 2, BOARD_W - PITCH / 2)
HEADER_PINS = [(PITCH / 2 + PITCH * k, v) for v in ROW_V for k in range(14)]
END_PINS = [(BOARD_L - PITCH / 2, PITCH / 2 + PITCH * m) for m in range(1, 6)]
INNER_PIN = [(PITCH / 2 + PITCH, PITCH / 2 + PITCH)]
ALL_HOLES = HEADER_PINS + END_PINS + INNER_PIN
# Copper test pads (a ring with a gold centre), from PJRC's front and back photos
TEST_PADS_TOP = [(2.55, 2.5), (2.55, 15.2), (33.0, 15.3)]
TEST_PADS_BOTTOM = [(2.55, 15.4), (33.0, 15.4), (33.0, 2.7)]

# All positions and orientations below were measured on PJRC's photos of the front
# (teensy40_front.jpg, 12.2 px/mm) and the back (dimensions_teensy40_photo2.jpg, 39.7 px/mm
# after 5x), checked against the parts placement drawings, about +-0.2 mm. Sizes from the
# datasheets or standard packages. u runs from the USB end, v from the +y edge (the photo's top).

# Two-terminal parts: (name, material, u, v, body_u, body_v, height, outline_u, outline_v).
# The body is the package (0402 1.0 x 0.5, 0603 1.6 x 0.8 mm) with metal ends on its long
# axis. The outline adds the solder on the pads around it, as far as the part stands out from
# the board on PJRC's photos. Centres and outlines measured on the photos
# (provenance/teensy_4_0.md, "How the parts were measured"). Names from PJRC's parts
# placement drawings.
TOP_PASSIVES = [
    ('D4', 'led_red', 5.82, 5.08, 0.8, 1.6, 0.6, 0.70, 1.66),
    ('C25', 'cap', 5.84, 8.90, 0.8, 1.6, 0.8, 0.92, 2.31),
    ('C8', 'cap', 5.87, 11.19, 0.5, 1.0, 0.35, 0.74, 1.49),
    ('C7', 'cap', 8.39, 7.46, 1.0, 0.5, 0.35, 1.55, 0.74),
    ('R4', 'res', 8.85, 12.07, 1.0, 0.5, 0.35, 1.55, 0.71),
    ('C26', 'cap', 8.82, 13.37, 1.6, 0.8, 0.6, 2.35, 0.98),
    ('C1', 'cap', 22.94, 4.54, 0.5, 1.0, 0.35, 0.49, 1.47),
    ('C2', 'cap', 22.91, 6.14, 0.5, 1.0, 0.35, 0.53, 1.43),
    ('R5', 'res', 26.97, 5.08, 0.5, 1.0, 0.35, 0.74, 1.51),
    ('C3', 'cap', 28.61, 4.04, 1.0, 0.5, 0.35, 1.51, 0.74),
    ('C4', 'cap', 30.41, 4.04, 1.0, 0.5, 0.35, 1.51, 0.74),
    ('C27', 'cap', 26.28, 8.99, 1.6, 0.8, 0.8, 2.19, 0.92),
    ('C28', 'cap', 29.31, 12.54, 0.8, 1.6, 0.6, 0.98, 2.08),
    ('D3', 'led_orange', 32.29, 5.07, 0.8, 1.6, 0.6, 0.98, 2.35),  # the pin 13 LED
]
BOTTOM_PASSIVES = [
    ('L2', 'cap', 12.74, 7.82, 0.8, 1.6, 0.8, 0.89, 2.38),
    ('F1', 'ceramic', 13.19, 5.24, 1.0, 2.0, 0.6, 1.08, 2.54),
    ('R2', 'res', 14.81, 5.52, 1.0, 0.5, 0.35, 1.71, 0.67),
    ('R1', 'res', 15.78, 4.44, 1.0, 0.5, 0.35, 1.62, 0.67),
    ('R3', 'res', 16.32, 12.89, 1.0, 0.5, 0.35, 1.49, 0.62),
    ('X1', 'ceramic', 17.53, 12.92, 0.5, 0.5, 0.25, 0.56, 0.50),  # no label on the drawing
    ('L3', 'inductor', 22.62, 13.83, 1.6, 0.8, 0.8, 2.44, 0.76),
    ('C29', 'inductor', 22.60, 12.10, 1.6, 0.8, 0.8, 2.41, 0.92),
    ('C17', 'cap', 23.62, 10.74, 0.5, 1.0, 0.35, 0.64, 1.49),
] + [(name, 'cap', u, v, 1.0, 0.5, 0.35, ou, ov) for name, u, v, ou, ov in (
    ('C6', 15.99, 9.47, 1.55, 0.63), ('C5', 17.77, 9.47, 1.55, 0.63),
    ('C16', 21.28, 9.48, 1.59, 0.63), ('C23', 14.31, 8.24, 1.51, 0.70),
    ('C12', 16.00, 8.17, 1.55, 0.57), ('C13', 17.77, 8.19, 1.55, 0.60),
    ('C14', 19.52, 8.16, 1.55, 0.67), ('C15', 21.29, 8.22, 1.56, 0.62),
    ('C22', 15.71, 6.87, 1.62, 0.76), ('C11', 19.14, 6.83, 1.62, 0.67),
    ('C10', 21.03, 6.87, 1.59, 0.60), ('C9', 22.97, 6.04, 1.59, 0.63),
    ('C20', 22.99, 4.72, 1.55, 0.73), ('C21', 20.91, 4.91, 1.52, 0.67),
    ('C24', 20.51, 13.35, 1.52, 0.70), ('C18', 20.54, 12.18, 1.59, 0.57))]
# Plain bodies: (name, material, u, v, body_u, body_v, height, outline_u, outline_v)
TOP_BLOCKS = [
    ('L1', 'inductor', 26.89, 12.30, 2.0, 2.6, 1.0, 2.17, 3.04),  # 4.7 uH, moulded
]
# Bodies with leads on their -v and +v edges: (name, u, v, size_u, size_v, height, leads on
# -v, leads on +v, lead pitch, lead length beyond the body). Leads as on the photo.
LEADED = [
    ('Q1', 8.5, 9.49, 2.9, 1.3, 1.0, 1, 2, 1.9, 0.55),     # SOT-23: single lead towards -v
    ('D1', 31.36, 12.44, 1.3, 1.1, 0.6, 3, 3, 0.5, 0.45),    # 6 leads
    ('U3', 24.08, 12.03, 2.0, 3.0, 0.55, 4, 4, 0.5, 0.15),  # flash W25Q16JV, USON-8 2 x 3 mm
    ('U4', 5.84, 13.54, 2.0, 2.0, 0.8, 3, 3, 0.62, 0.15),  # regulator, 6 pads, 2 x 2 mm
]
# U2, the bootloader chip: QFN 3 x 3 mm, 5 pads per side at 0.5 mm (photo), pin-1 dimple
QFN = (8.8, 5.1, 3.0, 0.75, 5, 0.5, (7.75, 5.9))
# U1, i.MX RT1062, 10 x 10 x 1.3 mm MAPBGA (NXP), pin-1 dimple at the -u/+v corner (photo)
MCU = (17.15, 8.9, 10.0, 1.3, (13.3, 12.3))
# Crystals: (u, v, body_u, body_v, height, outline_u, outline_v), outline with the solder
CRYSTAL_24M = (24.88, 5.56, 2.5, 3.2, 0.8, 2.88, 3.60)  # Y1 24 MHz: ceramic base, metal lid
CRYSTAL_32K = (29.49, 5.57, 3.2, 1.5, 0.9, 3.58, 1.84)  # Y2 32.768 kHz: metal ends along u
# S1 on the photo: metal frame 4.3 x 3.68 mm (with its legs 5.03 mm), plunger 3.3 x 2.7 mm.
# The PJRC drawing puts its centre 29.21 mm from the first pin column (u = 30.48).
BUTTON = (30.59, 8.87)
BUTTON_BODY = (4.3, 3.68, 1.8)
BUTTON_CAP = (3.3, 2.7, 2.5)  # plunger: size_u, size_v, top above the board
BUTTON_OPENING = (4.0, 3.0)   # opening of the metal frame around the plunger
BUTTON_LEGS = [(28.24, 7.9, 0.53, 0.8), (28.24, 9.66, 0.53, 0.65),  # (u, v, size_u, size_v)
               (32.84, 7.9, 0.33, 0.8), (32.84, 9.66, 0.33, 0.65)]
# Micro-USB B receptacle (PJRC drawing): 0.7 mm beyond the board's edge, 7.5 mm wide, 2.5 mm
# high, flange 8.06 x 3.07 mm, 0.6 mm thick. The shell ends at u = 4.3 (photo), the five
# signal leads behind it.
USB_FRONT, USB_FLANGE_T, USB_LEN = -0.7, 0.6, 4.4
USB_W, USB_H, USB_FLANGE_W, USB_FLANGE_H = 7.5, 2.5, 8.06, 3.07
USB_OPENING = (6.8, 1.8, 3.0)  # width, height, depth of the plug opening
USB_SIDE_TABS = [(1.4, 4.6), (1.4, 12.95)]    # shell tabs beside the shell, 1.6 x 0.8 mm
USB_REAR_LEGS = [(4.05, 6.5), (4.05, 11.1)]   # rear legs, 1.4 x 1.4 mm, dark on the photo
USB_LATCH_SLOTS = [(0.55, 6.47), (0.55, 11.37)]  # the two slots in the shell's top
USB_LATCH_SLOT = (2.2, 1.0)  # their size along u and v
# Bottom pads (PJRC): pins 24-33 2.794 x 0.889, USB host 3.048 x 1.016, SD 2.0 x 0.6 mm.
# Positions and the other pads from the back photo
BOTTOM_PADS = ([(25.86 + c * 4.17, 8.89 + PITCH * r, 2.794, 0.889)
                for c in range(2) for r in range(-2, 3)]
               + [(1.97, v, 3.048, 1.016) for v in (7.87, 10.2)]
               + [(9.15, 5.53 + k, 2.0, 0.6) for k in range(8)]
               + [(7.07, v, 4.2, 0.55) for v in (4.42, 13.5)]
               + [(1.42, v, 1.9, 1.0) for v in (2.98, 4.25)])
BOTTOM_SLOTS = [(1.5, 5.6), (1.5, 12.45)]  # oval pads of the USB shell's tabs, 1.7 x 0.7 mm
BOTTOM_VIAS = [(2.3, 13.75), (4.0, 11.5), (5.45, 12.3), (5.6, 7.9), (4.1, 6.5), (5.5, 5.6),
               (27.95, 12.7), (27.95, 10.2), (27.95, 7.6), (27.95, 5.1), (16.9, 7.15),
               (18.1, 5.0), (22.6, 8.3)]


# --- Geometry helpers --------------------------------------------------------------------------
def box(x0, x1, y0, y1, z0, z1):
    """Return an axis-aligned box from its bounds."""
    return cq.Workplane('XY').box(x1 - x0, y1 - y0, z1 - z0, centered=False) \
        .translate((x0, y0, z0))


def board_box(u, v, su, sv, z0, z1):
    """Return a box centred on board position (u, v), sizes along u and v."""
    x, y = to_xy(u, v)
    return box(x - su / 2, x + su / 2, y - sv / 2, y + sv / 2, z0, z1)


def disc(x, y, d, z0, z1):
    """Return a vertical cylinder."""
    return cq.Workplane('XY').circle(d / 2).extrude(z1 - z0).translate((x, y, z0))


def ring(x, y, d_out, d_in, z0, z1):
    """Return an annulus (flat ring) between z0 and z1."""
    return (cq.Workplane('XY').circle(d_out / 2).circle(d_in / 2).extrude(z1 - z0)
            .translate((x, y, z0)))


# --- The parts ---------------------------------------------------------------------------------
def pcb():
    """Return the board with its 34 plated holes."""
    x0, y0 = to_xy(0, BOARD_W)
    body = (cq.Workplane('XY').rect(BOARD_L, BOARD_W, centered=False).extrude(BOARD_T)
            .edges('|Z').fillet(CORNER_R).translate((x0, y0, 0)))
    pts = [to_xy(u, v) for u, v in ALL_HOLES]
    return body.faces('>Z').workplane(origin=(0, 0, BOARD_T)).pushPoints(pts).hole(HOLE_D)


def pads():
    """Return the plated rings on both faces and the bottom SMT pads."""
    shapes = []
    for u, v in ALL_HOLES:
        x, y = to_xy(u, v)
        shapes.append(ring(x, y, PAD_D, HOLE_D, BOARD_T - 0.005, BOARD_T + PAD_T))
        shapes.append(ring(x, y, PAD_D, HOLE_D, -PAD_T, 0.005))
    for u, v, su, sv in BOTTOM_PADS:
        shapes.append(board_box(u, v, su, sv, -PAD_T, 0.005))
    return shapes


def header():
    """Return the two 14-pin headers: spacers, pins and solder joints, by material."""
    spacers, pins, solder = [], [], []
    for u, v in HEADER_PINS:
        x, y = to_xy(u, v)
        spacers.append(box(x - PITCH / 2 + 0.03, x + PITCH / 2 - 0.03, y - PITCH / 2 + 0.03,
                           y + PITCH / 2 - 0.03, -SPACER_H, 0.0)
                       .edges('|Z').chamfer(0.25))
        z_bottom = -SPACER_H - PIN_BELOW
        z_top = BOARD_T + PIN_ABOVE
        pins.append(box(x - PIN_W / 2, x + PIN_W / 2, y - PIN_W / 2, y + PIN_W / 2,
                        z_bottom, z_top).faces('>Z or <Z').chamfer(0.15))
        cone = cq.Solid.makeCone(SOLDER_D0 / 2, SOLDER_D1 / 2, SOLDER_H,
                                 cq.Vector(x, y, BOARD_T + PAD_T - 0.01), cq.Vector(0, 0, 1))
        solder.append(cq.Workplane('XY').add(cone))
    return spacers, pins, solder


def dimple(u, v, d, z):
    """Return a pin-1 marker: a shallow, slightly lighter disc on top of a package."""
    return disc(*to_xy(u, v), d, z - 0.01, z + 0.005)


def mcu():
    """Return the MCU: ball layer, substrate, mould cap and pin-1 dimple."""
    u, v, size, height, pin1 = MCU
    z = BOARD_T - 0.01
    return [('ic', board_box(u, v, size - 0.8, size - 0.8, z, BOARD_T + 0.25)),
            ('substrate', board_box(u, v, size, size, BOARD_T + 0.25, BOARD_T + 0.55)),
            ('ic', board_box(u, v, size, size, BOARD_T + 0.55, BOARD_T + height)
             .edges('>Z').chamfer(0.08)),
            ('ic_mark', dimple(*pin1, 0.5, BOARD_T + height))]


def solder(u, v, ou, ov, bottom=False):
    """Return the solder on a part's pads: a flat slab as large as the part's outline."""
    z0, z1 = (BOARD_T - 0.01, BOARD_T + SOLDER_T) if not bottom else (-SOLDER_T, 0.01)
    return ('solder', board_box(u, v, ou, ov, z0, z1))


def two_terminal(name, mat, u, v, su, sv, h, ou, ov, bottom=False):
    """Return a chip resistor, capacitor or LED: body, metal ends on its long axis, solder."""
    z0, z1 = (BOARD_T - 0.01, BOARD_T + h) if not bottom else (0.01, -h)
    lo, hi = min(z0, z1), max(z0, z1)
    end = 0.22 * max(su, sv)
    if su >= sv:
        ends = [(u + d * (su / 2 - end / 2), v, end, sv + 0.02) for d in (-1, 1)]
        body = (u, v, su - 2 * end + 0.02, sv)
    else:
        ends = [(u, v + d * (sv / 2 - end / 2), su + 0.02, end) for d in (-1, 1)]
        body = (u, v, su, sv - 2 * end + 0.02)
    parts = [(mat, board_box(*body, lo, hi))]
    parts += [('silver', board_box(*e, lo, hi)) for e in ends]
    parts.append(solder(u, v, max(ou, su), max(ov, sv), bottom))
    return parts


def leaded(name, u, v, su, sv, h, n_minus, n_plus, pitch, length):
    """Return a package with leads on its -v and +v edges (SOT, USON, WSON)."""
    z = BOARD_T - 0.01
    parts = [('ic', board_box(u, v, su, sv, z + 0.05, BOARD_T + h).edges('>Z').chamfer(0.05))]
    width = min(0.4, 0.6 * pitch)
    for side, n in ((-1, n_minus), (1, n_plus)):
        for k in range(n):
            du = (k - (n - 1) / 2) * pitch
            dv = side * (sv / 2 + length / 2 - 0.1)
            parts.append(('silver', board_box(u + du, v + dv, width, length + 0.2, z,
                                              BOARD_T + 0.15)))
    return parts


def qfn():
    """Return U2: a QFN body, the tin pads on its four edges and its pin-1 dimple."""
    u, v, size, h, per_side, pitch, pin1 = QFN
    z = BOARD_T - 0.01
    parts = [('ic', board_box(u, v, size, size, z, BOARD_T + h).edges('>Z').chamfer(0.05)),
             ('ic_mark', dimple(*pin1, 0.35, BOARD_T + h))]
    for k in range(per_side):
        off = (k - (per_side - 1) / 2) * pitch
        for du, dv, su, sv in ((off, -size / 2, 0.25, 0.35), (off, size / 2, 0.25, 0.35),
                               (-size / 2, off, 0.35, 0.25), (size / 2, off, 0.35, 0.25)):
            parts.append(('silver', board_box(u + du, v + dv, su, sv, z, BOARD_T + 0.2)))
    return parts


def usb():
    """Return the micro-USB B receptacle: shell, opening, tongue, tabs, legs and leads."""
    x0, _ = to_xy(0, 0)
    _, yc = to_xy(0, BOARD_W / 2)
    z_mid = BOARD_T + USB_H / 2
    shell = box(x0 + USB_FRONT + USB_FLANGE_T, x0 + USB_FRONT + USB_LEN + USB_FLANGE_T,
                yc - USB_W / 2, yc + USB_W / 2, BOARD_T, BOARD_T + USB_H)
    flange = box(x0 + USB_FRONT, x0 + USB_FRONT + USB_FLANGE_T, yc - USB_FLANGE_W / 2,
                 yc + USB_FLANGE_W / 2, z_mid - USB_FLANGE_H / 2, z_mid + USB_FLANGE_H / 2)
    ow, oh, depth = USB_OPENING
    opening = box(x0 + USB_FRONT - 0.1, x0 + USB_FRONT + depth, yc - ow / 2, yc + ow / 2,
                  z_mid - oh / 2, z_mid + oh / 2).edges('|X').chamfer(0.3)
    metal = shell.union(flange).cut(opening)
    tongue = box(x0 + USB_FRONT + 0.6, x0 + USB_FRONT + depth + 0.01, yc - 2.6, yc + 2.6,
                 z_mid + 0.05, z_mid + 0.6)
    parts = [('silver', metal), ('plastic_black', tongue)]
    parts += [('silver', board_box(u, v, 1.6, 0.8, BOARD_T - 0.01, BOARD_T + 0.6))
              for u, v in USB_SIDE_TABS]
    parts += [('plastic_black', board_box(u, v, 1.4, 1.4, BOARD_T - 0.01, BOARD_T + 1.0))
              for u, v in USB_REAR_LEGS]
    parts += [('silver', board_box(4.6, BOARD_W / 2 + 0.65 * (k - 2), 0.6, 0.3,
                                   BOARD_T - 0.01, BOARD_T + 0.15)) for k in range(5)]
    parts += [('usb_slot', board_box(u, v, *USB_LATCH_SLOT, BOARD_T + USB_H - 0.01,
                                     BOARD_T + USB_H + 0.01)) for u, v in USB_LATCH_SLOTS]
    return parts


def button():
    """Return the push button: dark body, metal frame, plunger."""
    u, v = BUTTON
    bu, bv, bh = BUTTON_BODY
    cu, cv, ch = BUTTON_CAP
    ou, ov = BUTTON_OPENING
    x, y = to_xy(u, v)
    body = board_box(u, v, bu - 0.1, bv - 0.1, BOARD_T - 0.01, BOARD_T + bh - 0.2)
    frame = (board_box(u, v, bu, bv, BOARD_T + bh - 0.2, BOARD_T + bh).edges('|Z').chamfer(0.3)
             .cut(cq.Workplane('XY').rect(ou, ov).extrude(1.0).edges('|Z').fillet(1.0)
                  .translate((x, y, BOARD_T + bh - 0.5))))
    cap = (cq.Workplane('XY').rect(cu, cv).extrude(ch - bh + 0.25).edges('|Z').fillet(1.1)
           .translate((x, y, BOARD_T + bh - 0.25)))
    legs = [('button_frame', board_box(lu, lv, su, sv, BOARD_T - 0.01, BOARD_T + 0.5))
            for lu, lv, su, sv in BUTTON_LEGS]
    return [('ic', body), ('button_frame', frame), ('button_cap', cap)] + legs


def crystals():
    """Return Y1 (ceramic base, metal lid) and Y2 (ceramic body, metal ends), with solder."""
    z = BOARD_T - 0.01
    u, v, su, sv, h, ou, ov = CRYSTAL_24M
    parts = [('ceramic', board_box(u, v, su, sv, z, BOARD_T + 0.3)),
             ('silver', board_box(u, v, su - 0.25, sv - 0.25, BOARD_T + 0.3, BOARD_T + h)),
             solder(u, v, ou, ov)]
    u, v, su, sv, h, ou, ov = CRYSTAL_32K
    parts.append(('ceramic_grey', board_box(u, v, su - 0.7, sv, z, BOARD_T + h)))
    for du in (-(su - 0.4) / 2, (su - 0.4) / 2):
        parts.append(('silver', board_box(u + du, v, 0.4, sv + 0.02, z, BOARD_T + h)))
    parts.append(solder(u, v, ou, ov))
    return parts


def test_pads():
    """Return the copper test pads (copper ring, gold centre) on both faces."""
    parts = []
    for pads, z0, z1 in ((TEST_PADS_TOP, BOARD_T - 0.005, BOARD_T + PAD_T),
                         (TEST_PADS_BOTTOM, -PAD_T, 0.005)):
        for u, v in pads:
            x, y = to_xy(u, v)
            parts.append(('copper', ring(x, y, 1.4, 0.8, z0, z1)))
            parts.append(('gold', disc(x, y, 0.8, z0 - 0.002, z1 + 0.002)))
    return parts


def bottom_features():
    """Return the bottom's slot pads (with their dark slots) and via pads."""
    parts = []
    for u, v in BOTTOM_SLOTS:
        x, y = to_xy(u, v)
        parts.append(('gold', cq.Workplane('XY').rect(1.7, 0.7).extrude(PAD_T + 0.005)
                      .edges('|Z').fillet(0.34).translate((x, y, -PAD_T))))
        parts.append(('plastic_black', cq.Workplane('XY').rect(0.9, 0.3).extrude(PAD_T + 0.01)
                      .edges('|Z').fillet(0.14).translate((x, y, -PAD_T - 0.003))))
    parts += [('gold', disc(*to_xy(u, v), 0.9, -PAD_T, 0.005)) for u, v in BOTTOM_VIAS]
    return parts


def pcb_material(tri):
    """Return the material of each board triangle: mask on the faces, FR4 edges, plated holes."""
    normal = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    normal /= np.linalg.norm(normal, axis=1, keepdims=True)
    mats = np.full(len(tri), 'pcb', dtype=object)
    side = np.abs(normal[:, 2]) < 0.5
    centre = tri.mean(axis=1)[:, :2]
    holes = np.array([to_xy(u, v) for u, v in ALL_HOLES])
    near = np.min(np.linalg.norm(centre[:, None, :] - holes[None], axis=2), axis=1) < 0.6
    mats[side & ~near] = 'fr4'
    mats[side & near] = 'gold'
    return mats


# Parts whose triangles get more than one material
CLASSIFIERS = {'pcb': pcb_material}


# --- Mesh --------------------------------------------------------------------------------------
FRAME = ("The board's bottom face at z = 0, the board from x = -13.929 to 21.631 mm and "
         'centred at y = -0.032 mm, the USB connector at -x, the row with the inner pin hole at '
         '+y. The header spacer ends at z = -2.5 mm, the pins at z = -8.5 mm.')


def build() -> List[Model]:
    """Return the board's mesh, one sub-part per group of parts."""
    groups: List[Tuple[str, List[Piece]]] = [('board', [('pcb', pcb())]),
                                             ('pads', [('gold', s) for s in pads()])]
    if WITH_HEADER:
        spacers, pins, joints = header()
        groups.append(('header', [('plastic_black', s) for s in spacers]
                       + [('gold', s) for s in pins] + [('solder', s) for s in joints]))
    leaded_parts: List[Piece] = []
    for leaded_spec in LEADED:
        leaded_parts += leaded(*leaded_spec)
    blocks: List[Piece] = []
    for _, mat, u, v, su, sv, h, ou, ov in TOP_BLOCKS:
        blocks.append((mat, board_box(u, v, su, sv, BOARD_T - 0.01, BOARD_T + h)
                       .edges('>Z').chamfer(0.08)))
        blocks.append(solder(u, v, ou, ov))
    top_passives: List[Piece] = []
    for top_spec in TOP_PASSIVES:
        top_passives += two_terminal(*top_spec)
    bottom_passives: List[Piece] = []
    for bottom_spec in BOTTOM_PASSIVES:
        bottom_passives += two_terminal(*bottom_spec, bottom=True)
    groups += [('mcu', mcu()), ('bootloader_chip', qfn()), ('leaded_parts', leaded_parts),
               ('blocks', blocks), ('top_passives', top_passives),
               ('bottom_passives', bottom_passives), ('crystals', crystals()), ('usb', usb()),
               ('button', button()), ('test_pads', test_pads()),
               ('bottom_features', bottom_features())]
    sub_parts = [classified_sub_part(name, pieces, list(MATERIALS), TOL, ANG_TOL, CLASSIFIERS)
                 for name, pieces in groups]
    return [Model(NAME, TITLE, FRAME, MATERIALS, sub_parts)]
