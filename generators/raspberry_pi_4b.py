# SPDX-FileCopyrightText: 2026 ROS Mobile Robots
# SPDX-License-Identifier: BSD-3-Clause
"""Generate the display model of a Raspberry Pi 4 Model B.

The model is drawn from scratch from Raspberry Pi's public mechanical drawing of the Pi 4 Model B
(raspberry-pi-4-mechanical-drawing.pdf and .dxf: board outline, holes, part outlines and part
heights "Z"). No geometry comes from another 3D model. Sources in provenance/raspberry_pi_4b.md.

Coordinates while drawing: the drawing's frame in mm, x along the 85 mm edge (USB and Ethernet at
x = 85), y along the 56 mm edge (GPIO header at y = 56), z up with the board's mid-plane at z = 0.
The meshes are turned and moved into Remo's frame for the board (see BOARD_TO_MESH).
"""
from typing import List
from typing import Sequence
from typing import Tuple
from typing import Union

import cadquery as cq
import numpy as np

from generators.board_parts import box
from generators.board_parts import chamfered_rectangle
from generators.board_parts import cylinder
from generators.board_parts import drop_degenerate
from generators.board_parts import materials
from generators.board_parts import Part
from generators.board_parts import pin_header
from generators.board_parts import place
from generators.board_parts import prism
from generators.board_parts import quad_package
from generators.board_parts import ring
from generators.board_parts import rj45_jack
from generators.board_parts import rounded_box
from generators.board_parts import shell_with_opening
from generators.board_parts import split_board
from generators.board_parts import two_terminal
from generators.board_parts import usb_a_stack
from generators.mesh_export import Model
from generators.mesh_export import SubPart
from generators.mesh_export import Surface
from generators.mesh_export import surfaces_by_material
from generators.mesh_export import tessellate_shape
from generators.mesh_export import transform_surface

TITLE = 'Raspberry Pi 4 Model B'


# ---------------------------------------------------------------------------------------------
# Parameters (mm). Sources: Raspberry Pi 4 Model B mechanical drawing (PDF and DXF), see
# provenance/raspberry_pi_4b.md. Heights "Z" are the drawing's heights above the board's top face.
# ---------------------------------------------------------------------------------------------
NAME = 'raspberry_pi_4b'
BOARD_LENGTH = 85.0
BOARD_WIDTH = 56.0
BOARD_THICKNESS = 1.6  # Not in the drawing: the common FR-4 thickness
CORNER_RADIUS = 3.0
HOLE_DIAMETER = 2.7
HOLE_PAD_DIAMETER = 6.0
HOLES = ((3.5, 3.5), (61.5, 3.5), (3.5, 52.5), (61.5, 52.5))
TOP = BOARD_THICKNESS / 2  # z of the board's top face
BOTTOM = -BOARD_THICKNESS / 2

HEADER_PIN_HEIGHT = 8.5  # Drawing: GPIO header Z = 8.5
PIN_TAIL = 2.0  # Solder tails below the board
GPIO_PINS_X = [8.37 + 2.54 * k for k in range(20)]
GPIO_PINS_Y = (51.23, 53.77)
POE_PINS = ((60.23, 45.09), (62.77, 45.09), (60.23, 47.63), (62.77, 47.63))
J2_PADS = ((9.46, 20.5), (12.0, 20.5), (14.54, 20.5))  # RUN and GLOBAL_EN, not fitted

ETHERNET = (66.65, 37.99, 88.0, 53.51, 13.5)  # x0, y0, x1, y1, Z
USB2 = (70.3, 1.65, 88.0, 15.57, 16.0)
USB3 = (70.5, 20.43, 88.0, 34.25, 16.0)
USB_C = (11.2, -1.25, 6.15, 8.94, 3.2)  # centre x, front y, back y, width, Z
MICRO_HDMI = ((26.0, -1.43, 6.52), (39.5, -1.43, 6.52))  # centre x, front y, back y
MICRO_HDMI_SIZE = (6.5, 3.0)  # width, Z
AUDIO = (50.5, 0.0, 57.5, 12.5, 6.0)  # body x0, y0, x1, y1, Z. The barrel reaches y = -2.5
AUDIO_BARREL = (54.0, -2.5, 6.0)  # centre x, front y, diameter
CSI = (45.05, 0.3, 49.0, 22.7, 5.5)  # camera connector: x0, y0, x1, y1, Z
DSI = (1.5, 16.8, 5.45, 39.2, 5.5)  # display connector
SOC = (21.75, 25.0, 36.75, 40.0, 2.4)
RAM = (39.05, 25.0, 50.55, 40.0, 1.0)
WIRELESS_CAN = (6.7, 36.2, 17.3, 49.0, 1.5)
MICRO_SD = (1.69, 22.11, 13.13, 34.10, 1.4)  # on the bottom, height below the board
LEDS = (((0.75, 7.10, 1.45, 8.70), 'led_red'), ((0.75, 10.70, 1.45, 12.30), 'led_green'))
INDUCTOR_BOX = (12.9, 14.1, 16.9, 18.1)
TALL_PARTS = {(65.55, 29.75, 69.85, 37.05): 2.2, (18.20, 6.30, 21.80, 10.70): 2.0}

# Pins of the Ethernet and USB connectors that show as solder joints on the bottom (centres).
THROUGH_HOLE_PINS = (
    (72.38, 5.5), (75.0, 5.5), (72.38, 8.0), (75.0, 8.0), (72.38, 10.0), (75.0, 10.0),
    (72.38, 12.5), (75.0, 12.5), (71.66, 23.0), (74.86, 23.0), (73.16, 23.5), (76.36, 23.5),
    (71.66, 25.0), (74.86, 25.0), (73.16, 26.0), (76.36, 26.0), (71.66, 27.0), (74.86, 27.0),
    (73.16, 28.0), (76.36, 28.0), (71.66, 29.0), (74.86, 29.0), (73.16, 30.5), (76.36, 30.5),
    (71.66, 31.0), (74.86, 31.0), (68.36, 40.035), (73.42, 40.035), (70.9, 41.305),
    (68.36, 42.575), (74.69, 42.575), (70.9, 43.845), (68.36, 45.115), (70.9, 46.385),
    (68.36, 47.655), (70.9, 48.925), (74.69, 48.925), (68.36, 50.195), (70.9, 51.465),
    (73.42, 51.465))
SHELL_PEGS = ((72.03, 2.43), (77.71, 2.43), (72.03, 15.57), (77.71, 15.57), (72.68, 20.43),
              (78.36, 20.43), (72.68, 33.57), (78.36, 33.57), (81.31, 39.11), (81.31, 41.65),
              (81.31, 49.85), (81.31, 52.39))

# Small parts on both sides: outlines from the drawing's PARTS_TOP and PARTS_BOTTOM layers,
# without the parts drawn above. "x0 y0 x1 y1" for a rectangle, "P x y x y ..." for a polygon.
SMALL_PARTS_TOP = """
    31.12 5.53 32.27 6.67
    33.23 5.53 34.38 6.67
    66.40 17.30 68.40 18.50
    51.40 18.60 54.40 20.60
    50.50 13.90 52.50 16.90
    66.40 15.30 68.40 16.50
    2.60 14.30 3.80 16.30
    P 12.9 15.1 12.9 17.1 13.9 18.1 15.9 18.1 16.9 17.1 16.9 15.1 15.9 14.1 13.9 14.1
    65.55 29.75 69.85 37.05
    60.00 16.10 63.00 18.30
    62.70 14.25 63.70 14.75
    38.70 42.75 39.70 43.25
    52.40 17.65 53.40 18.15
    56.45 32.60 56.95 33.60
    10.40 7.75 11.40 8.25
    8.50 7.75 9.50 8.25
    66.90 19.25 67.90 19.75
    66.75 2.40 67.25 3.40
    68.75 2.40 69.25 3.40
    67.75 2.40 68.25 3.40
    56.95 15.70 57.45 16.70
    56.95 13.80 57.45 14.80
    62.05 7.50 62.55 8.50
    53.75 15.70 54.25 16.70
    53.75 13.80 54.25 14.80
    62.05 9.60 62.55 10.60
    69.35 10.00 69.85 11.00
    69.35 8.00 69.85 9.00
    28.25 22.10 28.75 23.10
    18.20 48.35 19.20 48.85
    18.20 47.35 19.20 47.85
    62.50 34.75 63.50 35.25
    58.90 31.90 59.40 32.90
    4.00 12.50 5.60 13.30
    14.20 11.10 15.80 11.90
    15.70 8.50 16.50 10.10
    4.00 11.10 5.60 11.90
    14.20 12.50 15.80 13.30
    7.50 15.80 9.10 16.60
    60.20 31.80 61.00 33.40
    57.50 31.80 58.30 33.40
    64.70 8.20 65.50 9.80
    12.05 7.40 12.55 8.40
    2.85 42.30 3.15 42.90
    2.85 41.00 3.15 41.60
    0.50 42.30 0.80 42.90
    0.50 41.00 0.80 41.60
    P 4.682 47.956 5.106 47.532 5.318 47.744 4.894 48.168
    P 62.963 39.030 64.730 37.263 65.437 37.970 63.670 39.737
    P 62.963 42.330 64.730 40.563 65.437 41.270 63.670 43.037
    30.05 21.35 30.35 21.95
    P 36.294 43.082 36.718 43.506 36.506 43.718 36.082 43.294
    62.50 35.85 63.10 36.15
    65.10 19.35 65.70 19.65
    38.55 41.80 39.15 42.10
    21.60 40.70 22.20 41.00
    21.60 41.35 22.20 41.65
    43.70 40.55 44.30 40.85
    45.30 40.55 45.90 40.85
    55.45 13.70 55.75 14.30
    32.90 2.85 33.50 3.15
    32.00 7.40 32.60 7.70
    65.35 5.90 65.65 6.50
    56.15 13.70 56.45 14.30
    54.75 13.70 55.05 14.30
    P 26.218 47.094 25.794 47.518 25.582 47.306 26.006 46.882
    7.55 7.80 7.85 8.40
    59.05 16.90 59.35 17.50
    63.65 16.90 63.95 17.50
    53.40 25.35 54.00 25.65
    53.40 25.95 54.00 26.25
    53.40 22.95 54.00 23.25
    53.40 23.55 54.00 23.85
    65.10 24.75 65.70 25.05
    65.10 24.10 65.70 24.40
    65.10 21.15 65.70 21.45
    65.10 20.50 65.70 20.80
    53.40 24.15 54.00 24.45
    53.40 24.75 54.00 25.05
    62.30 11.50 64.10 12.50
    0.75 10.70 1.45 12.30
    0.75 7.10 1.45 8.70
    6.70 17.10 9.50 19.10
    6.20 17.50 7.15 18.70
    9.05 17.50 10.00 18.70
    18.20 6.30 21.80 10.70
    30.80 1.50 32.40 4.50
    58.90 11.50 60.50 14.50
    67.20 4.25 68.80 7.15
    54.60 15.70 56.60 16.90
    55.50 20.00 63.50 28.00
    55.50 35.00 61.50 41.00
    11.00 16.10 12.20 18.10
    51.10 29.40 52.30 31.40
    47.50 40.60 49.50 41.80
    67.60 12.30 68.80 14.30
    58.70 9.50 60.70 10.70
    58.70 7.40 60.70 8.60
    62.00 31.85 63.20 33.85
    17.60 14.90 18.80 16.90
    19.50 14.90 20.70 16.90
    2.60 8.15 3.80 10.15
    64.90 12.05 66.90 14.55
    4.50 7.95 6.50 10.45
    13.10 7.95 15.10 10.45
    4.50 14.05 6.50 16.55
    66.05 8.70 68.95 10.30
    7.40 9.70 12.40 14.70
"""
SMALL_PARTS_BOTTOM = """
    14.15 16.95 15.45 17.85
    10.40 18.00 12.00 18.80
    21.85 41.05 23.35 42.05
    57.80 30.85 60.80 33.05
    18.40 33.92 20.60 36.92
    25.40 45.50 28.00 47.30
    9.15 14.30 10.15 14.80
    10.65 15.50 11.15 16.50
    63.40 19.95 64.40 20.45
    18.15 8.00 18.65 9.00
    2.55 15.95 3.55 16.45
    56.75 19.80 57.25 20.80
    4.95 7.80 5.45 8.80
    18.15 4.30 18.65 5.30
    16.50 15.95 17.50 16.45
    14.30 15.95 15.30 16.45
    17.25 4.30 17.75 5.30
    7.35 15.90 7.85 16.90
    60.50 38.45 61.50 38.95
    60.50 40.65 61.50 41.15
    5.30 34.75 6.30 35.25
    23.10 46.75 24.10 47.25
    28.30 10.95 29.30 11.45
    28.30 11.95 29.30 12.45
    23.10 43.85 24.10 44.35
    8.35 15.90 8.85 16.90
    53.65 33.50 54.15 34.50
    23.10 44.95 24.10 45.45
    23.10 45.85 24.10 46.35
    51.45 16.90 51.95 17.90
    30.45 18.55 31.45 19.05
    32.55 21.90 33.05 22.90
    33.40 21.90 33.90 22.90
    34.25 21.90 34.75 22.90
    21.25 33.30 22.25 33.80
    35.50 40.35 36.50 40.85
    30.85 19.70 32.45 20.50
    54.50 26.00 56.10 26.80
    21.10 38.55 21.90 40.15
    9.55 9.25 11.15 10.05
    2.65 13.50 3.45 15.10
    55.85 34.15 56.65 35.75
    55.55 39.20 56.35 40.80
    62.60 27.40 63.40 29.00
    58.50 19.10 59.30 20.70
    32.45 18.40 34.05 19.20
    28.20 21.75 29.00 23.35
    31.30 21.75 32.10 23.35
    29.55 21.75 30.35 23.35
    54.50 23.60 56.10 24.40
    14.30 18.35 15.30 18.85
    21.25 34.15 22.25 34.65
    45.60 24.35 46.60 24.85
    41.95 40.20 42.95 40.70
    6.00 16.60 6.60 16.90
    26.70 42.90 27.30 43.20
    26.70 42.15 27.30 42.45
    20.85 41.05 21.45 41.35
    27.80 42.90 28.40 43.20
    7.65 19.20 7.95 19.80
    37.90 41.85 38.20 42.45
    21.15 34.95 21.75 35.25
    21.15 35.60 21.75 35.90
    44.70 9.15 45.30 9.45
    63.00 39.65 63.60 39.95
    59.35 27.80 59.65 28.40
    55.10 22.65 55.70 22.95
    59.85 19.30 60.15 19.90
    63.30 23.85 63.90 24.15
    63.30 21.25 63.90 21.55
    63.30 25.05 63.90 25.35
    63.30 22.65 63.90 22.95
    63.30 26.25 63.90 26.55
    55.10 25.05 55.70 25.35
    61.75 27.80 62.05 28.40
    58.15 27.80 58.45 28.40
    55.10 21.45 55.70 21.75
    60.75 19.30 61.05 19.90
    60.55 27.80 60.85 28.40
    56.95 27.80 57.25 28.40
    61.75 19.30 62.05 19.90
    52.75 19.30 53.05 19.90
    51.20 15.80 51.80 16.10
    59.85 34.70 60.15 35.30
    60.65 34.70 60.95 35.30
    61.20 37.35 61.80 37.65
    61.20 39.85 61.80 40.15
    57.35 34.70 57.65 35.30
    56.85 40.70 57.15 41.30
    61.45 31.70 61.75 32.30
    56.85 32.30 57.15 32.90
    17.55 34.20 17.85 34.80
    18.70 37.55 19.30 37.85
    46.90 40.15 47.50 40.45
    19.10 15.30 20.10 17.10
    49.30 16.50 50.30 18.30
    32.40 8.90 34.20 9.90
    30.20 8.90 32.00 9.90
    0.70 17.60 2.50 18.60
    4.25 18.50 7.15 20.10
"""

# The mesh's frame: the board lies in x -7.569 ... 48.431 and y -42.123 ... 42.877, the USB and
# Ethernet end at +y, the GPIO header along -x, z = 0 in the board's mid-plane. Remo's rpi_joint
# then puts the mounting holes on the deck's standoff bores and the mid-plane on their tops.
# So (x, y) of the drawing maps to (48.431 - y, x - 42.123): a turn by +90 degrees about z and
# a shift.
BOARD_TO_MESH = (48.431, -42.123)


# Materials (generators/board_parts.py): colours sampled from Raspberry Pi's product photo.
MATERIALS = materials(['solder_mask_green', 'fr4_edge', 'gold', 'nickel', 'steel_lid', 'tin',
                       'black_plastic', 'chip', 'ceramic', 'substrate', 'inductor', 'usb3_blue',
                       'cream_plastic', 'led_red', 'led_green', 'led_yellow'])
TOLERANCE = 0.03  # Tessellation: linear (mm) and angular (rad) deflection
ANGULAR_TOLERANCE = 0.35
Piece = Union[Part, Tuple[str, Surface]]


# ---------------------------------------------------------------------------------------------
# Parts
# ---------------------------------------------------------------------------------------------
def board() -> List[Piece]:
    """Return the board: solder mask, FR-4 edges, plated holes with gold pads on both sides."""
    plate = cq.Workplane('XY', origin=(BOARD_LENGTH / 2, BOARD_WIDTH / 2, BOTTOM)) \
        .rect(BOARD_LENGTH, BOARD_WIDTH).extrude(BOARD_THICKNESS).edges('|Z') \
        .fillet(CORNER_RADIUS).val()
    parts: List[Piece] = []
    for x, y in HOLES:
        plate = plate.cut(cylinder(HOLE_DIAMETER / 2, (x, y, BOTTOM - 1), (0, 0, 1), 5))
        parts.append(('gold', ring((x, y), HOLE_PAD_DIAMETER, HOLE_DIAMETER, TOP, TOP + 0.04)))
        parts.append(('gold', ring((x, y), HOLE_PAD_DIAMETER, HOLE_DIAMETER,
                                   BOTTOM - 0.04, BOTTOM)))
    for x, y in J2_PADS:
        plate = plate.cut(cylinder(0.5, (x, y, BOTTOM - 1), (0, 0, 1), 5))
        parts.append(('gold', ring((x, y), 1.6, 1.0, TOP, TOP + 0.04)))
        parts.append(('gold', ring((x, y), 1.6, 1.0, BOTTOM - 0.04, BOTTOM)))
    surface = tessellate_shape(plate, TOLERANCE, ANGULAR_TOLERANCE)
    plated = list(HOLES) + list(J2_PADS)
    return list(split_board(surface, 'solder_mask_green', plated, HOLE_DIAMETER / 2 + 0.1)) \
        + parts


def headers() -> List[Piece]:
    """Return the 40-pin GPIO header and the 4-pin PoE header."""
    gpio = [(x, y) for x in GPIO_PINS_X for y in GPIO_PINS_Y]
    return list(pin_header(gpio, TOP, BOTTOM, HEADER_PIN_HEIGHT, PIN_TAIL)) \
        + list(pin_header(POE_PINS, TOP, BOTTOM, HEADER_PIN_HEIGHT, PIN_TAIL))


def ethernet() -> List[Piece]:
    """Return the RJ45 jack, its opening towards +x."""
    x0, y0, x1, y1, height = ETHERNET
    return list(place(rj45_jack(y1 - y0, x1 - x0, height), 0.0, (x1, (y0 + y1) / 2, TOP)))


def usb_stack(spec: Tuple[float, float, float, float, float], tongue: str) -> List[Piece]:
    """Return a stacked double USB-A receptacle, its openings towards +x."""
    x0, y0, x1, y1, height = spec
    return list(place(usb_a_stack(y1 - y0, x1 - x0, height, tongue), 0.0,
                      (x1, (y0 + y1) / 2, TOP)))


def usb_c() -> List[Piece]:
    """Return the USB-C power receptacle, its opening towards -y."""
    centre_x, front, back, width, height = USB_C
    centre_z = TOP + height / 2
    shell = cq.Workplane('XZ', origin=(0, front, 0)).center(centre_x, centre_z) \
        .slot2D(width, height).extrude(-(back - front)).val()
    opening = cq.Workplane('XZ', origin=(0, front - 0.1, 0)).center(centre_x, centre_z) \
        .slot2D(width - 0.6, height - 0.6).extrude(-6.0).val()
    liner = cq.Workplane('XZ', origin=(0, front, 0)).center(centre_x, centre_z) \
        .slot2D(width - 0.6, height - 0.6).extrude(-5.9).val().cut(
            cq.Workplane('XZ', origin=(0, front - 0.1, 0)).center(centre_x, centre_z)
            .slot2D(width - 1.0, height - 1.0).extrude(-5.8).val())
    parts: List[Piece] = [('nickel', shell.cut(opening)), ('black_plastic', liner),
                          ('black_plastic', box(centre_x - 3.3, front + 0.9, centre_z - 0.35,
                                                centre_x + 3.3, front + 5.9, centre_z + 0.35))]
    for k in range(12):
        x = centre_x - 2.75 + 0.5 * k
        for z in (centre_z + 0.35, centre_z - 0.4):
            parts.append(('gold', box(x - 0.13, front + 1.4, z, x + 0.13, front + 4.5, z + 0.05)))
    parts.append(('nickel', box(centre_x - 4.33, back - 2.0, TOP, centre_x + 4.33, back,
                                TOP + 0.6)))
    return parts


def micro_hdmi() -> List[Piece]:
    """Return the two micro-HDMI receptacles, their openings towards -y."""
    width, height = MICRO_HDMI_SIZE
    opening = chamfered_rectangle(width - 0.6, height - 0.7, 0.5, 0.35)
    parts: List[Piece] = []
    for centre_x, front, back in MICRO_HDMI:
        receptacle = shell_with_opening(width, back - front, height, opening, 5.0, 0.5,
                                        contacts=(1.55, 1.9, 10), tongue=(2.1, 1.05, 1.55))
        parts += place(receptacle, -90.0, (centre_x, front, TOP))
    return parts


def audio_jack() -> List[Piece]:
    """Return the 3.5 mm AV jack: black body, barrel with hole, metal contact clips."""
    x0, y0, x1, y1, height = AUDIO
    centre_x, front, diameter = AUDIO_BARREL
    centre_z = TOP + height / 2
    body = rounded_box(x0, y0, TOP, x1, y1, TOP + height, chamfer_top=0.3)
    barrel = cylinder(diameter / 2, (centre_x, y0 + 0.01, centre_z), (0, -1, 0), y0 - front)
    hole = cylinder(1.8, (centre_x, front - 0.1, centre_z), (0, 1, 0), 9.0)
    clip = box(x0 + 1.4, y0 + 3.0, TOP + height - 0.01, x1 - 1.4, y1 - 2.0, TOP + height + 0.3)
    side_clip = box(x1 - 0.3, y0 + 4.0, TOP, x1 - 0.01, y1 - 3.0, TOP + height - 1.0)
    return [('black_plastic', body.fuse(barrel).cut(hole)), ('nickel', clip),
            ('nickel', side_clip)]


def ffc_connector(spec: Tuple[float, float, float, float, float],
                  pins_at_low_x: bool) -> List[Piece]:
    """Return a vertical flat flex connector (camera or display): cream body, dark latch, pins."""
    x0, y0, x1, y1, height = spec
    body = rounded_box(x0 + 0.9, y0, TOP, x1 - 0.2, y1, TOP + height - 1.2, vertical=0.2) \
        if pins_at_low_x else \
        rounded_box(x0 + 0.2, y0, TOP, x1 - 0.9, y1, TOP + height - 1.2, vertical=0.2)
    latch = box(x0 + 1.1, y0 + 1.0, TOP + height - 1.2, x1 - 0.4, y1 - 1.0, TOP + height) \
        if pins_at_low_x else \
        box(x0 + 0.4, y0 + 1.0, TOP + height - 1.2, x1 - 1.1, y1 - 1.0, TOP + height)
    centre_x = (x0 + x1) / 2 + (0.35 if pins_at_low_x else -0.35)
    slot = box(centre_x - 0.35, y0 + 3.0, TOP + 1.0, centre_x + 0.35, y1 - 3.0, TOP + height + 0.1)
    parts: List[Piece] = [('cream_plastic', body.cut(slot)), ('black_plastic', latch.cut(slot))]
    tail_x0, tail_x1 = (x0, x0 + 0.9) if pins_at_low_x else (x1 - 0.9, x1)
    for k in range(15):
        y = (y0 + y1) / 2 - 7.0 + k
        parts.append(('gold', box(tail_x0, y - 0.15, TOP, tail_x1, y + 0.15, TOP + 0.25)))
        parts.append(('gold', box(centre_x - 0.36, y - 0.12, TOP + 1.0, centre_x - 0.3, y + 0.12,
                                  TOP + height - 1.4)))
    return parts


def chips() -> List[Piece]:
    """Return the processor with its stamped lid, the RAM and the wireless module's shield."""
    x0, y0, x1, y1, height = SOC
    centre = ((x0 + x1) / 2, (y0 + y1) / 2)
    parts: List[Piece] = [('substrate', box(x0, y0, TOP, x1, y1, TOP + 1.1))]
    flange = cq.Workplane('XY', origin=(centre[0], centre[1], TOP + 1.1)).rect(14.2, 14.2) \
        .extrude(0.4).edges('|Z').fillet(1.0).val()
    cap = cq.Workplane('XY', origin=(centre[0], centre[1], TOP + 1.5)).rect(12.2, 12.2) \
        .extrude(height - 1.5).edges('|Z').fillet(0.8).faces('>Z').edges().fillet(0.3).val()
    parts += [('steel_lid', flange), ('steel_lid', cap)]
    x0, y0, x1, y1, height = RAM
    parts.append(('chip', rounded_box(x0 + 0.3, y0 + 0.3, TOP, x1 - 0.3, y1 - 0.3,
                                      TOP + height, chamfer_top=0.15)))
    x0, y0, x1, y1, height = WIRELESS_CAN
    can = rounded_box(x0, y0, TOP, x1, y1, TOP + height, vertical=0.4, top=0.2)
    parts.append(('nickel', can.cut(box(x0 + 1.2, y0 + 1.2, TOP + height - 0.06, x1 - 1.2,
                                        y1 - 1.2, TOP + height + 0.1))))
    return parts


def parse_outlines(table: str) -> List[List[Tuple[float, float]]]:
    """Return the outlines of a parts table as polygons."""
    outlines = []
    for line in table.strip().splitlines():
        values = line.split()
        if values[0] == 'P':
            coords = [float(v) for v in values[1:]]
            outlines.append(list(zip(coords[0::2], coords[1::2])))
        else:
            x0, y0, x1, y1 = (float(v) for v in values)
            outlines.append([(x0, y0), (x1, y0), (x1, y1), (x0, y1)])
    return outlines


def polygon_area(points: Sequence[Tuple[float, float]]) -> float:
    """Return the area of a polygon."""
    return 0.5 * abs(sum(points[i][0] * points[i - 1][1] - points[i - 1][0] * points[i][1]
                         for i in range(len(points))))


def outline_box(points: Sequence[Tuple[float, float]]) -> Tuple[float, float, float, float]:
    """Return the bounding box of an outline, rounded like the tables."""
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return (round(min(xs), 2), round(min(ys), 2), round(max(xs), 2), round(max(ys), 2))


def small_part(points: Sequence[Tuple[float, float]], top_side: bool) -> List[Piece]:
    """Return one small part, its look chosen by size and outline.

    Two-terminal parts (rectangles below 3.5 mm²) get tin end caps, larger rectangles are
    chips with pads along their edges, a few parts are named by their outline.
    """
    area = polygon_area(points)
    bounds = outline_box(points)
    height = 0.3 if area < 0.3 else 0.45 if area < 1.0 else 0.6 if area < 3.5 else 0.9
    z0, z1 = (TOP, TOP + height) if top_side else (BOTTOM - height, BOTTOM)
    rectangle = len(points) == 4 and abs(polygon_area(points) - (bounds[2] - bounds[0])
                                         * (bounds[3] - bounds[1])) < 1e-6
    if bounds == INDUCTOR_BOX:
        z1 = TOP + 2.0
        return [('inductor', prism(points, z0, z1))]
    if bounds in TALL_PARTS:
        z1 = TOP + TALL_PARTS[bounds]
        return [('chip', rounded_box(*bounds[:2], z0, *bounds[2:], z1, chamfer_top=0.15))]
    for led_box, led_material in LEDS:
        if bounds == led_box:
            return [(led_material, prism(points, z0, TOP + 0.6))]
    if not rectangle:
        return [('chip', prism(points, z0, z1))]
    if area < 3.5:
        return list(two_terminal(points, z0, z1, 'ceramic'))
    if top_side:
        return list(quad_package(*bounds[:2], *bounds[2:], z0, height))
    return [('chip', prism(points, z0, z1))]


def small_parts() -> List[Piece]:
    """Return the small parts on both sides of the board."""
    parts: List[Piece] = []
    for points in parse_outlines(SMALL_PARTS_TOP):
        parts += small_part(points, True)
    for points in parse_outlines(SMALL_PARTS_BOTTOM):
        parts += small_part(points, False)
    return parts


def bottom_side() -> List[Piece]:
    """Return the microSD card socket and the solder joints of the through-hole connectors."""
    x0, y0, x1, y1, height = MICRO_SD
    socket = box(x0, y0, BOTTOM - height, x1, y1, BOTTOM)
    slot = box(x0 - 0.1, y0 + 0.6, BOTTOM - height + 0.25, x0 + 8.0, y1 - 0.6, BOTTOM - 0.3)
    parts: List[Piece] = [('nickel', socket.cut(slot))]
    for x, y in THROUGH_HOLE_PINS:
        joint = cq.Workplane('XY').circle(0.55).workplane(offset=0.9).circle(0.25) \
            .loft().translate((x, y, BOTTOM - 0.9)).mirror('XY', (0, 0, BOTTOM - 0.45)).val()
        parts.append(('tin', joint))
    for x, y in SHELL_PEGS:
        parts.append(('nickel', box(x - 0.6, y - 0.3, BOTTOM - 1.4, x + 0.6, y + 0.3, BOTTOM)))
    return parts


# ---------------------------------------------------------------------------------------------
# Mesh
# ---------------------------------------------------------------------------------------------
FRAME = ("The board's mid-plane at z = 0, the board from x = -7.569 to 48.431 mm and from "
         'y = -42.123 to 42.877 mm, the USB and Ethernet end at +y, the GPIO header along -x.')


def to_mesh_frame(surface: Surface) -> Surface:
    """Return the surface turned by +90 degrees about z and moved into the mesh's frame."""
    turn = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    return transform_surface(surface, turn, (BOARD_TO_MESH[0], BOARD_TO_MESH[1], 0.0))


def sub_part(name: str, pieces: Sequence[Piece]) -> SubPart:
    """Return a named sub-part from its (material, shape or surface) pieces."""
    surfaces = []
    for material, piece in pieces:
        surface = piece if isinstance(piece, Surface) else \
            drop_degenerate(tessellate_shape(piece, TOLERANCE, ANGULAR_TOLERANCE))
        surfaces.append((material, to_mesh_frame(surface)))
    return SubPart(name, surfaces_by_material(surfaces, list(MATERIALS)))


def build() -> List[Model]:
    """Return the board's mesh, one sub-part per connector or group of parts."""
    groups = [('board', board()), ('headers', headers()), ('ethernet', ethernet()),
              ('usb_2', usb_stack(USB2, 'black_plastic')),
              ('usb_3', usb_stack(USB3, 'usb3_blue')), ('usb_c', usb_c()),
              ('micro_hdmi', micro_hdmi()), ('audio_jack', audio_jack()),
              ('camera_connector', ffc_connector(CSI, True)),
              ('display_connector', ffc_connector(DSI, False)), ('chips', chips()),
              ('small_parts', small_parts()), ('bottom_side', bottom_side())]
    sub_parts = [sub_part(name, parts) for name, parts in groups]
    return [Model(NAME, TITLE, FRAME, MATERIALS, sub_parts)]
