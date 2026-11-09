# SPDX-FileCopyrightText: 2026 ROS Mobile Robots
# SPDX-License-Identifier: BSD-3-Clause
"""Generate the display model of a Jetson Nano Developer Kit B01.

The model is drawn from scratch from NVIDIA's public documents: the developer kit user guide's
top, front and rear views (connector positions and heights), the 2022 developer kit data sheet
(100 x 79 mm carrier, 30.21 mm total height), the getting started page's B01 picture (the two
camera connectors, the heatsink's fins and fan holes) and the module size (69.6 x 45 mm). The
mounting holes are from a public NVIDIA forum post, the connectors' insides from the connector
makers' public drawings, the B01's fan header, capacitors, J48 and some widths from Waveshare's
top photo. No geometry comes from a 3D model. Sources and how each number was read in
provenance/jetson_nano_dev_kit_b01.md.

Coordinates while drawing: the carrier board's frame in mm as in the user guide's top view,
u from the left edge (DC jack side) to the right edge (40-pin header side), v from the port edge
inwards, z up with the carrier's bottom face at z = 0. The meshes are turned and moved into
Remo's frame for the kit (see BOARD_TO_MESH).
"""
from typing import Dict
from typing import List
from typing import Sequence
from typing import Tuple
from typing import Union

import cadquery as cq
import numpy as np

from generators.board_parts import bent_strip
from generators.board_parts import box
from generators.board_parts import chamfered_rectangle
from generators.board_parts import contact_rows
from generators.board_parts import cylinder
from generators.board_parts import drop_degenerate
from generators.board_parts import fin_polygon
from generators.board_parts import front_profile
from generators.board_parts import header_pin
from generators.board_parts import materials
from generators.board_parts import opening_with_liner
from generators.board_parts import Part
from generators.board_parts import pin_header
from generators.board_parts import place
from generators.board_parts import Point
from generators.board_parts import quad_package
from generators.board_parts import ring
from generators.board_parts import rj45_jack
from generators.board_parts import rounded_box
from generators.board_parts import small_receptacle
from generators.board_parts import split_board
from generators.board_parts import spring_finger
from generators.board_parts import tongue_block
from generators.board_parts import usb_a_stack
from generators.mesh_export import Model
from generators.mesh_export import SubPart
from generators.mesh_export import Surface
from generators.mesh_export import surfaces_by_material
from generators.mesh_export import tessellate_shape
from generators.mesh_export import transform_surface

TITLE = 'Jetson Nano Developer Kit B01'

# ---------------------------------------------------------------------------------------------
# Parameters (mm), see provenance/jetson_nano_dev_kit_b01.md for each source
# ---------------------------------------------------------------------------------------------
NAME = 'jetson_nano_dev_kit_b01'
BOARD_U = 100.0
BOARD_V = 79.0
BOARD_THICKNESS = 1.6  # Not published: the common 1.6 mm
CORNER_RADIUS = 1.0
TOP = BOARD_THICKNESS
HOLE_DIAMETER = 2.75
HOLES = ((4.0, 17.0), (90.0, 17.0), (4.0, 75.0), (90.0, 75.0))
HEADER_PIN_HEIGHT = 8.5
PIN_TAIL = 1.95  # Solder tails below the carrier, from the 30.21 mm total height

# Port row, facing -v: (u0, u1, front v, back v, height above the carrier's top face).
DC_JACK = (1.6, 10.6, -1.97, 12.8, 11.05)  # 9.0 wide as Same Sky's PJ-002A
DC_JACK_HOLE = (6.5, 6.33, 9.5)  # diameter (PJ-002A), centre height above the top face, depth
DC_PIN = (2.0, 8.0)  # PJ-002A: diameter, length from the hole's floor
HDMI_DP = (15.8, 32.6, -1.52, 16.3, 18.6)  # u from the photo
HDMI_OPENING = (14.0, 4.55, 1.4)  # width, height (Same Sky HD05), bottom above the top face
# HDMI insides, Same Sky HD05 and HD10: insulator face behind the front, tongue (width,
# thickness, tip behind the front), 10 contacts on the top face and 9 below at 1.0 mm.
HDMI_INSULATOR = 6.8
HDMI_TONGUE = (10.8, 0.9, 2.0)
DP_OPENING = (16.1, 4.76, 12.7)
# DisplayPort insides, JAE DP1RD20: insulator face behind the front, tongue (width, thickness
# (scaled from the front view), tip behind the front, its centreline above the opening's
# bottom: 3.4 mm above the board, the opening's bottom 0.88 mm (scaled)), its centre 0.5 mm off
# the opening's, 10 contacts on each face at 1.0 mm.
DP_INSULATOR = 8.88
DP_TONGUE = (12.0, 1.0, 4.1, 2.52)
DP_TONGUE_OFFSET = 0.5
HDMI_HOUSING_HEIGHT = 10.2  # the HDMI receptacle's black housing, the DP shell above it
USB_STACKS = ((36.15, 49.25, -1.52, 15.9, 16.0), (53.2, 66.3, -1.52, 15.9, 16.0))  # photo
RJ45 = (68.9, 85.0, -1.97, 19.8, 13.7)
MICRO_USB = (88.3, 95.8, -0.94, 4.6, 3.0)
# Micro-USB insides, GCT USB3131: the opening 6.9 wide at the top with a 5.2 mm flat at the
# bottom, 1.85 high, 3.5 deep; the tongue (half width, z0, z1, tip behind the front) with five
# contacts underneath at 0.65 mm.
MICRO_USB_OPENING = (6.9, 1.85, 0.85, 0.55)  # width, height, chamfer, bottom above the top face
MICRO_USB_TONGUE = (1.75, 1.62, 2.18, 0.8)  # 0.56 mm, with the 0.04 mm contacts GCT's 0.60
MICRO_USB_SHELL_WIDTH = 7.4
POWER_LED = (96.4, 10.7, 98.0, 11.5)

# Module (P3448) in the SO-DIMM connector J2, and the heatsink on it.
MODULE = (13.3, 32.4, 82.9, 77.4)  # u0, v0, u1, v1: 69.6 x 45
MODULE_Z = (9.2, 10.2)
MODULE_STANDOFFS = ((16.3, 75.4), (79.9, 75.4))
MODULE_PAD = 5.6  # gold pads round the standoff holes on the module's top (photo)
# Under the carrier (Waveshare's bottom photo): the standoffs' screws in holes with copper pads,
# and the twelve unpopulated holes of the button header J50, pin 1 at u 49.71.
CARRIER_SCREW_HOLE = 2.6
CARRIER_SCREW_PAD = 5.8
CARRIER_SCREW_HEAD = (4.0, 1.2)  # diameter, height
J50_PINS = [(49.71 - 2.54 * k, 69.47) for k in range(12)]
J50_HOLE = 1.0
J50_PAD = 1.7
SODIMM = (13.3, 28.0, 82.9, 36.6, 11.0)  # u0, v0, u1, v1, top z
SODIMM_POSTS = ((11.0, 13.3), (82.9, 85.6))  # u ranges of the connector's end posts
SODIMM_LATCHES = ((12.3, 13.1), (83.1, 83.9))  # u ranges of the metal latches, v to 54.1
SODIMM_PITCH = 0.5  # 260 contacts, 130 a row
SODIMM_CONTACTS = 130
MODULE_SD_SLOT = (36.0, 63.0, 49.5, 77.4, 7.8)  # on the module's underside, top z = module bottom
MODULE_FINGERS = (35.4, 0.3)  # the gold fingers reach to v 35.4, 0.3 mm wide
MODULE_KEY = (51.0, 1.0)  # u of the key notch in the module's edge, its width
MODULE_FINGER_BANKS = (71, 59)  # fingers a face beside the key: u 15.0 to 50.0, 52.0 to 81.0

# Parts on the module under the heatsink, placed by the heatsink's underside (drawing of
# source 7): its contact area over the processor, its 2 mm deep pocket over taller parts and
# its four M2 holes, through which screws from below the module hold it.
SOC_CENTRE = (38.45, 50.7)
SOC_SUBSTRATE = (23.0, 0.45)  # Tegra X1 package side, height
SOC_DIE = (10.9, 0.25)  # side, height
THERMAL_PAD = (12.0, 0.05)
MEMORY = (61.0, 44.0, 71.0, 58.5, 1.0)  # u0, v0, u1, v1, height: a 10 x 14.5 mm package
MODULE_CHIPS = ((62.0, 63.0, 68.0, 69.0, 0.9), (70.0, 61.0, 74.0, 65.0, 1.8),
                (70.0, 67.0, 74.0, 71.0, 1.8))  # power management chip and two inductors
HEATSINK_SCREWS = ((22.08, 45.5), (55.51, 45.5), (22.08, 60.4), (55.51, 60.4))
HEATSINK_POCKET = (18.95, 2.0, 4.0, 1.0)  # reach from the end, depth, corner radius, rim

# The official heatsink (drawing of the Jetson Nano heatsink, see provenance): an extruded
# profile 58.7 long (along u), 39.4 wide (along v) and 17.3 high, centred on the module's
# u = 47.9 with its back edge on the module's back edge.
HEATSINK_LENGTH = 58.7
HEATSINK_WIDTH = 39.4
HEATSINK_HEIGHT = 17.3
HEATSINK_U0 = 47.9 - HEATSINK_LENGTH / 2
HEATSINK_V0 = 77.4 - HEATSINK_WIDTH
HEATSINK_TOP = 28.25
HEATSINK_BASE = (38.2, 3.8)  # width, thickness
FIN_THICKNESS = 0.9
# Fins from the base, mirrored about the profile's centre line: (root offset, tip offset, tip
# height above the heatsink's bottom). The centre fin stands upright, the others splay out.
FINS = ((0.0, 0.0, 16.4), (4.8, 6.8, 16.4), (13.2, 16.6, 11.7), (16.9, 19.2, 7.6))
# The two webs that carry the flat flanges along the long edges: (root offset, top offset,
# height where the web meets the flange); the flanges reach from FLANGE_INNER to the edge.
WEB = (8.9, 12.5, 16.1)
FLANGE_INNER = 12.0
FLANGE_THICKNESS = 0.9
FAN_HOLES_U = (13.35, 45.35)  # from the heatsink's end, 32 mm apart for a 40 mm fan
FAN_HOLES_V = (3.7, 35.7)  # from the heatsink's front long edge
FAN_HOLE_DIAMETER = 2.7

# Carrier parts beside and under the module.
GPIO_PINS = [(u, 46.0 + 2.54 * (k - 9.5)) for k in range(20) for u in (88.73, 91.27)]
POE_PINS = ((94.88, 73.48), (97.42, 73.48), (94.88, 76.02), (97.42, 76.02))
POWER_SELECT_PINS = ((12.99, 16.58), (12.99, 19.12))  # J48, from the photo
FAN_HEADER = (67.26, 22.2, 77.46, 27.95, 6.0)  # J15: u0, v0, u1, v1, height
FAN_PINS_V = 24.21  # the pin row, from the photo
CAPACITORS = ((82.3, 24.5), (51.55, 18.75))  # centres, from the photo
CAPACITOR_SIZE = (6.6, 8.0)  # diameter (photo), height (estimated)
CAMERA_CENTRES_V = (60.0, 35.4)  # J13 and J49 along the left edge
CAMERA = (0.4, 7.2, 21.9, 2.5)  # u0, u1, length along v (photo), height
M2_SLOT = (71.0, 46.6, 76.6, 67.6, 4.2)  # J18, Key E: u0, v0, u1, v1, height
M2_STANDOFF = (44.4, 57.25)

# The mesh's frame: the carrier lies in x -28.982 ... 71.018 and y -38.847 ... 40.153 with the
# ports at +y, the 40-pin header at -x and z = 0 at the carrier's bottom face. Remo's
# jetson_nano_joint then puts the mounting holes on the deck's standoff bores. So (u, v) maps to
# (71.018 - u, 40.153 - v): a turn by 180 degrees about z and a shift.
BOARD_TO_MESH = (71.018, 40.153)

# Materials (generators/board_parts.py): colours from NVIDIA's and CNX Software's photos.
MATERIALS = materials(['solder_mask_black', 'fr4_edge', 'anodised_black', 'nickel', 'gold',
                       'tin', 'brass', 'black_plastic', 'chip', 'die', 'substrate', 'inductor',
                       'thermal_pad', 'usb3_blue', 'latch_brown', 'capacitor_stripe',
                       'led_green', 'led_yellow'])
TOLERANCE = 0.03
ANGULAR_TOLERANCE = 0.35
Piece = Union[Part, Tuple[str, Surface]]


# ---------------------------------------------------------------------------------------------
# Parts
# ---------------------------------------------------------------------------------------------
def board_surface(u0: float, v0: float, u1: float, v1: float, z0: float, z1: float,
                  radius: float, holes: Sequence[Tuple[float, float, float]]) -> Surface:
    """Return a board's tessellated surface: a rounded rectangle with holes (u, v, diameter)."""
    plate = cq.Workplane('XY', origin=((u0 + u1) / 2, (v0 + v1) / 2, z0)) \
        .rect(u1 - u0, v1 - v0).extrude(z1 - z0).edges('|Z').fillet(radius).val()
    for u, v, diameter in holes:
        plate = plate.cut(cylinder(diameter / 2, (u, v, z0 - 1), (0, 0, 1), z1 - z0 + 2))
    return tessellate_shape(plate, TOLERANCE, ANGULAR_TOLERANCE)


def carrier() -> List[Piece]:
    """Return the carrier board: black solder mask, FR-4 edges, holes and pads.

    The mounting holes are bare. The holes of the module's standoff screws have copper pads
    on the bottom face, the twelve unpopulated holes of the button header J50 pads on both.
    """
    plated = [(u, v, CARRIER_SCREW_HOLE) for u, v in MODULE_STANDOFFS] \
        + [(u, v, J50_HOLE) for u, v in J50_PINS]
    holes = [(u, v, HOLE_DIAMETER) for u, v in HOLES] + plated
    surface = board_surface(0.0, 0.0, BOARD_U, BOARD_V, 0.0, BOARD_THICKNESS, CORNER_RADIUS,
                            holes)
    parts: List[Piece] = list(split_board(surface, 'solder_mask_black',
                                          [(u, v) for u, v, _ in plated],
                                          CARRIER_SCREW_HOLE / 2 + 0.1))
    for u, v in MODULE_STANDOFFS:
        parts.append(('gold', ring((u, v), CARRIER_SCREW_PAD, CARRIER_SCREW_HOLE, -0.03, 0.0)))
    for u, v in J50_PINS:
        for z0 in (-0.03, BOARD_THICKNESS):
            parts.append(('gold', ring((u, v), J50_PAD, J50_HOLE, z0, z0 + 0.03)))
    return parts


def dc_jack() -> List[Piece]:
    """Return the DC barrel jack J25, its opening towards -v.

    Black housing with a front recess, the round opening, the centre pin and the sleeve
    contact, a leaf spring along the bottom of the opening.
    """
    u0, u1, front, back, height = DC_JACK
    diameter, centre_height, depth = DC_JACK_HOLE
    pin_diameter, pin_length = DC_PIN
    length, half = back - front, (u1 - u0) / 2
    body = rounded_box(-length, -half, 0.0, 0.0, half, height, chamfer_top=0.4)
    hole = cylinder(diameter / 2, (0.1, 0.0, centre_height), (-1, 0, 0), depth + 0.1)
    recess = box(-0.4, -half + 0.6, 0.6, 0.1, half - 0.6, height - 0.6)
    pin = cylinder(pin_diameter / 2, (-depth + 0.01, 0.0, centre_height), (1, 0, 0), pin_length)
    floor = centre_height - diameter / 2 + 0.12
    spring = bent_strip([(-depth + 0.5, floor), (-4.5, floor), (-3.0, floor + 0.35),
                         (-2.2, floor + 0.35), (-1.6, floor + 0.05)], 0.15, 0.0, 1.6)
    terminal = box(-length + 1.0, -half, 0.0, -length + 4.0, -half + 0.3, 4.0)
    parts: List[Part] = [('black_plastic', body.cut(hole).cut(recess)), ('tin', pin),
                         ('nickel', spring), ('nickel', terminal)]
    return list(place(parts, -90.0, ((u0 + u1) / 2, front, TOP)))


def hdmi_inside(profile: Sequence[Point], wall: float, insulator: float,
                tongue: Tuple[float, float, float], centre: float, offset: float = 0.0) -> \
        List[Part]:
    """Return a receptacle's insulator face, tongue and contacts (10 on top, 9 or 10 below).

    The insulator stands in front of the liner's back wall (wall thick, at the insulator's
    depth), inside the liner.
    """
    tongue_width, thickness, tip = tongue
    half = tongue_width / 2
    z0, z1 = centre - thickness / 2, centre + thickness / 2
    face = -insulator + wall + 0.2
    parts: List[Part] = [
        ('black_plastic', front_profile(profile, face, 0.2, wall)),
        ('black_plastic', tongue_block(face, offset - half, z0, -tip, offset + half, z1, 0.15))]
    below = 9 if offset == 0.0 else 10
    return parts + contact_rows(((10, 1.0, 0.5, True), (below, 1.0, 0.5, False)),
                                (half, z0, z1, tip), 3.2, offset)


def hdmi_dp_stack() -> List[Part]:
    """Return the HDMI and DisplayPort stack J6 in its own frame (opening towards +x).

    The HDMI receptacle sits in a black housing with a metal frame round its opening, the
    DisplayPort receptacle above it in a metal shell with spring fingers. Inside both, the
    metal walls show up to the insulator, from which the tongue with its contacts reaches
    forward.
    """
    u0, u1, front, back, height = HDMI_DP
    depth, half = back - front, (u1 - u0) / 2
    parts: List[Part] = []
    housing = rounded_box(-depth, -half, 0.0, 0.0, half, HDMI_HOUSING_HEIGHT, vertical=0.3)
    opening_width, opening_height, bottom = HDMI_OPENING
    profile = chamfered_rectangle(opening_width, opening_height, 1.3, bottom)
    cut, liner = opening_with_liner(profile, HDMI_INSULATOR, 0.25)
    frame_outer = chamfered_rectangle(opening_width + 0.9, opening_height + 0.9, 1.6,
                                      bottom - 0.45)
    frame = front_profile(frame_outer, 0.0, 1.5).cut(front_profile(profile, 0.1, 1.7))
    # The frame sits in a seat cut into the housing, so their front faces don't overlap.
    seat = front_profile(frame_outer, 0.1, 1.6)
    parts += [('black_plastic', housing.cut(cut).cut(seat)), ('nickel', liner),
              ('nickel', frame)]
    parts += hdmi_inside(profile, 0.25, HDMI_INSULATOR, HDMI_TONGUE, bottom + opening_height / 2)
    shell = rounded_box(-depth, -half, HDMI_HOUSING_HEIGHT, -0.3, half, height, vertical=0.4)
    shell_frame = box(-0.3, -half, HDMI_HOUSING_HEIGHT, 0.0, half, height)
    opening_width, opening_height, bottom = DP_OPENING
    profile = chamfered_rectangle(opening_width, opening_height, 1.0, bottom, 'one')
    cut, liner = opening_with_liner(profile, DP_INSULATOR, 0.2)
    shell = shell.cut(cut)
    shell_frame = shell_frame.cut(cut)
    for y in (-4.5, 4.5):
        finger_cut, tab = spring_finger(-1.2, y, height)
        shell = shell.cut(finger_cut)
        parts.append(('nickel', tab))
    tongue_width, thickness, tip, centre = DP_TONGUE
    parts += [('nickel', shell), ('nickel', shell_frame), ('nickel', liner)]
    parts += hdmi_inside(profile, 0.2, DP_INSULATOR, (tongue_width, thickness, tip),
                         bottom + centre, DP_TONGUE_OFFSET)
    return parts


def hdmi_dp() -> List[Piece]:
    """Return the HDMI and DisplayPort stack J6, its openings towards -v."""
    u0, u1, front, _, _ = HDMI_DP
    return list(place(hdmi_dp_stack(), -90.0, ((u0 + u1) / 2, front, TOP)))


def usb_stack(spec: Tuple[float, float, float, float, float]) -> List[Piece]:
    """Return a stacked double USB 3.0 Type-A receptacle (J32, J33), openings towards -v."""
    u0, u1, front, back, height = spec
    return list(place(usb_a_stack(u1 - u0, back - front, height, 'usb3_blue'), -90.0,
                      ((u0 + u1) / 2, front, TOP)))


def rj45() -> List[Piece]:
    """Return the Gigabit Ethernet jack J43, its opening towards -v."""
    u0, u1, front, back, height = RJ45
    return list(place(rj45_jack(u1 - u0, back - front, height), -90.0,
                      ((u0 + u1) / 2, front, TOP)))


def micro_usb() -> List[Piece]:
    """Return the micro-USB receptacle J28, its opening towards -v."""
    u0, u1, front, back, height = MICRO_USB
    width, opening_height, chamfer, bottom = MICRO_USB_OPENING
    opening = chamfered_rectangle(width, opening_height, chamfer, bottom)
    receptacle = small_receptacle(MICRO_USB_SHELL_WIDTH, back - front, height, opening, 3.5, 0.4,
                                  MICRO_USB_TONGUE, ((5, 0.65, 0.35, False),), 2.0)
    receptacle.append(('nickel', box(-2.0, -(u1 - u0) / 2, 0.0, -1.0, (u1 - u0) / 2, 0.4)))
    return list(place(receptacle, -90.0, ((u0 + u1) / 2, front, TOP)))


def power_led() -> List[Piece]:
    """Return the green power LED DS3."""
    u0, v0, u1, v1 = POWER_LED
    return [('led_green', box(u0, v0, TOP, u1, v1, TOP + 0.6))]


def sodimm_connector() -> List[Piece]:
    """Return the SO-DIMM connector J2: body, end posts, 130 contacts in front and latches."""
    u0, v0, u1, v1, top = SODIMM
    connector = rounded_box(u0, v0, TOP, u1, v1, top, chamfer_top=0.3)
    slot = box(u0 + 2.0, v0 + 3.0, MODULE_Z[0] - 0.3, u1 - 2.0, v1 + 0.1, MODULE_Z[1] + 0.3)
    parts: List[Piece] = [('black_plastic', connector.cut(slot))]
    start = (u0 + u1) / 2 - (SODIMM_CONTACTS - 1) * SODIMM_PITCH / 2
    for k in range(SODIMM_CONTACTS):
        u = start + k * SODIMM_PITCH
        parts.append(('tin', box(u - 0.12, v0 - 0.02, TOP, u + 0.12, v0 + 0.3, TOP + 0.25)))
    for pu0, pu1 in SODIMM_POSTS:
        parts.append(('black_plastic', rounded_box(pu0, v0, TOP, pu1, v1, top, chamfer_top=0.3)))
    for lu0, lu1 in SODIMM_LATCHES:
        parts.append(('nickel', box(lu0, v1, TOP, lu1, 54.1, MODULE_Z[1] + 0.5)))
    return parts


def module_board() -> List[Piece]:
    """Return the module's board: black solder mask, FR-4 edges, gold fingers on both faces.

    The fingers along the edge in the SO-DIMM connector (130 a face at 0.5 mm) leave out the
    key notch's place. The holes take the standoffs' screws and the heatsink's screws.
    """
    mu0, mv0, mu1, mv1 = MODULE
    holes = list(MODULE_STANDOFFS) + list(HEATSINK_SCREWS)
    plate = cq.Workplane('XY', origin=((mu0 + mu1) / 2, (mv0 + mv1) / 2, MODULE_Z[0])) \
        .rect(mu1 - mu0, mv1 - mv0).extrude(MODULE_Z[1] - MODULE_Z[0]).edges('|Z') \
        .fillet(0.8).val()
    key_u, key_width = MODULE_KEY
    plate = plate.cut(box(key_u - key_width / 2, mv0 - 0.1, MODULE_Z[0] - 0.1,
                          key_u + key_width / 2, mv0 + 3.0, MODULE_Z[1] + 0.1))
    for u, v in holes:
        diameter = 2.8 if (u, v) in MODULE_STANDOFFS else 2.2
        plate = plate.cut(cylinder(diameter / 2, (u, v, MODULE_Z[0] - 1), (0, 0, 1), 3.0))
    parts: List[Piece] = list(split_board(tessellate_shape(plate, TOLERANCE, ANGULAR_TOLERANCE),
                                          'solder_mask_black'))
    parts += [('gold', ring((u, v), MODULE_PAD, 2.8, MODULE_Z[1], MODULE_Z[1] + 0.03))
              for u, v in MODULE_STANDOFFS]
    reach, finger_width = MODULE_FINGERS
    # Two banks beside the key notch, 130 fingers a face in all, the first 1.0 mm from the key.
    left = MODULE_FINGER_BANKS[0]
    fingers = [key_u - 1.0 - SODIMM_PITCH * k for k in range(left)] \
        + [key_u + 1.0 + SODIMM_PITCH * k for k in range(SODIMM_CONTACTS - left)]
    for u in fingers:
        for z0, z1 in ((MODULE_Z[1], MODULE_Z[1] + 0.02), (MODULE_Z[0] - 0.02, MODULE_Z[0])):
            parts.append(('gold', box(u - finger_width / 2, mv0 + 0.3, z0, u + finger_width / 2,
                                      reach, z1)))
    return parts


def module_sd_slot() -> List[Piece]:
    """Return the microSD slot under the module's back edge."""
    su0, sv0, su1, sv1, sz0 = MODULE_SD_SLOT
    socket = box(su0, sv0, sz0, su1, sv1, MODULE_Z[0])
    sd_slot = box(su0 + 1.2, sv1 - 8.0, sz0 + 0.3, su1 - 1.2, sv1 + 0.1, MODULE_Z[0] - 0.3)
    return [('nickel', socket.cut(sd_slot))]


def module_soc() -> Dict[str, List[Piece]]:
    """Return the processor on the module: its substrate and its bare die."""
    u, v = SOC_CENTRE
    side, height = SOC_SUBSTRATE
    die_side, die_height = SOC_DIE
    top = MODULE_Z[1] + height
    return {'soc_substrate': [('substrate', box(u - side / 2, v - side / 2, MODULE_Z[1],
                                                u + side / 2, v + side / 2, top))],
            'soc_die': [('die', box(u - die_side / 2, v - die_side / 2, top, u + die_side / 2,
                                    v + die_side / 2, top + die_height))]}


def thermal_pad() -> List[Piece]:
    """Return the thermal pad between the die and the heatsink."""
    u, v = SOC_CENTRE
    side, thickness = THERMAL_PAD
    bottom = HEATSINK_TOP - HEATSINK_HEIGHT - thickness
    return [('thermal_pad', box(u - side / 2, v - side / 2, bottom, u + side / 2, v + side / 2,
                                bottom + thickness))]


def module_chips() -> Dict[str, List[Piece]]:
    """Return the memory and the power parts on the module, under the heatsink's pocket."""
    u0, v0, u1, v1, height = MEMORY
    memory: List[Piece] = [('chip', rounded_box(u0, v0, MODULE_Z[1], u1, v1, MODULE_Z[1] + height,
                                                chamfer_top=0.15))]
    power: List[Piece] = []
    for k, (cu0, cv0, cu1, cv1, chip_height) in enumerate(MODULE_CHIPS):
        if k == 0:
            power += quad_package(cu0, cv0, cu1, cv1, MODULE_Z[1], chip_height)
        else:
            power.append(('inductor', rounded_box(cu0, cv0, MODULE_Z[1], cu1, cv1,
                                                  MODULE_Z[1] + chip_height, vertical=0.3)))
    return {'memory': memory, 'module_power': power}


def cross_head(u: float, v: float, z: float, diameter: float, height: float,
               up: bool) -> cq.Shape:
    """Return a pan head with a cross recess, its flat face on z, the head towards +z or -z."""
    head = cq.Workplane('XY').circle(diameter / 2).extrude(height) \
        .faces('>Z' if up else '<Z').edges().fillet(min(0.5, height / 2.5)) \
        .translate((u, v, z if up else z - height)).val()
    face = z + height if up else z - height
    for du, dv in ((1.2, 0.25), (0.25, 1.2)):
        head = head.cut(box(u - du, v - dv, face - 0.5, u + du, v + dv, face + 0.5))
    return head


def carrier_screws() -> List[Piece]:
    """Return the screws that hold the module's standoffs, from below the carrier."""
    diameter, height = CARRIER_SCREW_HEAD
    screws: List[Piece] = []
    for u, v in MODULE_STANDOFFS:
        shank = cylinder(CARRIER_SCREW_HOLE / 2 - 0.1, (u, v, -0.01), (0, 0, 1), TOP + 3.0)
        screws.append(('nickel', cross_head(u, v, 0.0, diameter, height, False).fuse(shank)
                       .clean()))
    return screws


def module_mounting() -> Dict[str, List[Piece]]:
    """Return the module's brass standoffs and their cross-head screws."""
    standoffs: List[Piece] = []
    screws: List[Piece] = []
    for u, v in MODULE_STANDOFFS:
        standoffs.append(('brass', cq.Workplane('XY').polygon(6, 5.0)
                          .extrude(MODULE_Z[0] - TOP).translate((u, v, TOP)).val()))
        head = cq.Workplane('XY').circle(2.2).extrude(1.3).faces('>Z').edges().fillet(0.5) \
            .translate((u, v, MODULE_Z[1])).val()
        head = head.cut(box(u - 1.2, v - 0.25, MODULE_Z[1] + 0.8, u + 1.2, v + 0.25,
                            MODULE_Z[1] + 1.5))
        head = head.cut(box(u - 0.25, v - 1.2, MODULE_Z[1] + 0.8, u + 0.25, v + 1.2,
                            MODULE_Z[1] + 1.5))
        screws.append(('nickel', head))
    return {'module_standoffs': standoffs, 'module_screws': screws}


def heatsink_profile() -> List[List[Tuple[float, float]]]:
    """Return the heatsink's cross-section as polygons (w across, h up), from the drawing."""
    centre = HEATSINK_WIDTH / 2
    base_width, base_height = HEATSINK_BASE
    polygons = [[(centre - base_width / 2, 0.0), (centre + base_width / 2, 0.0),
                 (centre + base_width / 2, base_height), (centre - base_width / 2, base_height)]]
    root = base_height - 0.6  # the fins start inside the base
    for root_offset, tip_offset, tip_height in FINS:
        for side in ((1.0,) if root_offset == 0.0 else (-1.0, 1.0)):
            polygons.append(fin_polygon((centre + side * root_offset, root),
                                        (centre + side * tip_offset, tip_height), FIN_THICKNESS))
    web_root, web_top, web_height = WEB
    for side in (-1.0, 1.0):
        polygons.append(fin_polygon((centre + side * web_root, root),
                                    (centre + side * web_top, web_height + 0.4), FIN_THICKNESS))
        inner, outer = centre + side * FLANGE_INNER, centre + side * centre
        polygons.append([(min(inner, outer), HEATSINK_HEIGHT - FLANGE_THICKNESS),
                         (max(inner, outer), HEATSINK_HEIGHT - FLANGE_THICKNESS),
                         (max(inner, outer), HEATSINK_HEIGHT),
                         (min(inner, outer), HEATSINK_HEIGHT)])
    return polygons


def heatsink() -> List[Piece]:
    """Return the heatsink: the extruded profile, its fan holes, the pocket and the M2 holes."""
    bottom = HEATSINK_TOP - HEATSINK_HEIGHT
    pieces = []
    for polygon in heatsink_profile():
        points = [(HEATSINK_V0 + w, bottom + h) for w, h in polygon]
        pieces.append(cq.Workplane('YZ', origin=(HEATSINK_U0, 0, 0)).polyline(points).close()
                      .extrude(HEATSINK_LENGTH).val())
    shape = pieces[0].fuse(*pieces[1:])
    for du in FAN_HOLES_U:
        for dv in FAN_HOLES_V:
            shape = shape.cut(cylinder(FAN_HOLE_DIAMETER / 2,
                                       (HEATSINK_U0 + du, HEATSINK_V0 + dv, HEATSINK_TOP - 3.0),
                                       (0, 0, 1), 3.1))
    length, depth, radius, rim = HEATSINK_POCKET
    base_v0 = HEATSINK_V0 + (HEATSINK_WIDTH - HEATSINK_BASE[0]) / 2
    u1 = HEATSINK_U0 + HEATSINK_LENGTH
    # A bounded recess: it reaches `length` from the end, with a rim at the end and both sides.
    pocket = cq.Workplane('XY', origin=(u1 - (length + rim) / 2, base_v0 + HEATSINK_BASE[0] / 2,
                                        bottom - 0.1)) \
        .rect(length - rim, HEATSINK_BASE[0] - 2 * rim).extrude(depth + 0.1) \
        .edges('|Z').fillet(radius).val()
    shape = shape.cut(pocket)
    for u, v in HEATSINK_SCREWS:
        shape = shape.cut(cylinder(0.8, (u, v, bottom - 0.1), (0, 0, 1), 3.1))
    return [('anodised_black', shape.clean())]


def heatsink_screws() -> List[Piece]:
    """Return the four M2 screws that hold the heatsink, from below the module."""
    screws: List[Piece] = []
    for u, v in HEATSINK_SCREWS:
        head = cq.Workplane('XY').circle(1.75).extrude(1.2).faces('<Z').edges().chamfer(0.3) \
            .translate((u, v, MODULE_Z[0] - 1.2)).val()
        shank = cylinder(0.8, (u, v, MODULE_Z[0] - 0.01), (0, 0, 1),
                         HEATSINK_TOP - HEATSINK_HEIGHT - MODULE_Z[0] + 2.8)
        screws.append(('nickel', head.fuse(shank).clean()))
    return screws


def headers() -> Dict[str, List[Piece]]:
    """Return the 40-pin header J41, the PoE header J38 and the power select header J48."""
    return {'gpio_header': list(pin_header(GPIO_PINS, TOP, 0.0, HEADER_PIN_HEIGHT, PIN_TAIL)),
            'poe_header': list(pin_header(POE_PINS, TOP, 0.0, HEADER_PIN_HEIGHT, PIN_TAIL)),
            'power_select_header': list(pin_header(POWER_SELECT_PINS, TOP, 0.0,
                                                   HEADER_PIN_HEIGHT, PIN_TAIL))}


def fan_header() -> List[Piece]:
    """Return the 4-pin fan header J15: black shroud with a latch ramp, pins at 2.54 mm."""
    u0, v0, u1, v1, height = FAN_HEADER
    shroud = rounded_box(u0, v0, TOP, u1, v1, TOP + height, vertical=0.3)
    pocket = box(u0 + 0.6, v0 + 0.6, TOP + 1.0, u1 - 0.6, v1 - 1.6, TOP + height + 0.1)
    ramp = box(u0 + 2.5, v1 - 1.0, TOP + height - 2.5, u1 - 2.5, v1 + 0.0, TOP + height)
    parts: List[Piece] = [('black_plastic', shroud.cut(pocket).cut(ramp)),
                          ('black_plastic', box(u0 + 2.5, v1 - 1.6, TOP + 1.0, u1 - 2.5,
                                                v1 - 0.6, TOP + height - 0.6))]
    for k in range(4):
        u = (u0 + u1) / 2 + 2.54 * (k - 1.5)
        parts.append(('gold', header_pin(u, FAN_PINS_V, -PIN_TAIL, TOP + height - 1.0)))
    return parts


def capacitors() -> List[Piece]:
    """Return the two electrolytic capacitors: black sleeve with a polarity stripe."""
    diameter, height = CAPACITOR_SIZE
    parts: List[Piece] = []
    for u, v in CAPACITORS:
        can = cq.Workplane('XY').circle(diameter / 2).extrude(height).faces('>Z').edges() \
            .fillet(0.4).translate((u, v, TOP)).val()
        stripe = cq.Workplane('XY').circle(diameter / 2 + 0.02).circle(diameter / 2 - 0.1) \
            .extrude(height - 0.6).translate((u, v, TOP + 0.3)).val() \
            .cut(box(u - diameter, v - diameter, TOP, u + diameter, v + diameter * 0.35,
                     TOP + height))
        parts += [('black_plastic', can), ('capacitor_stripe', stripe)]
    return parts


def camera_connector(centre_v: float) -> List[Piece]:
    """Return a 15-pin flat cable camera connector (J13, J49) along the left edge.

    The cable enters from the board's edge (low u) under the brown latch, the solder tails
    stand out on the inner side, as in the photo.
    """
    cu0, cu1, length, height = CAMERA
    cv0, cv1 = centre_v - length / 2, centre_v + length / 2
    body = rounded_box(cu0 + 1.0, cv0, TOP, cu1 - 1.0, cv1, TOP + height, vertical=0.3)
    slot = box(cu0 + 0.9, cv0 + 2.2, TOP + 0.9, cu1 - 3.0, cv1 - 2.2, TOP + 1.5)
    parts: List[Piece] = [('black_plastic', body.cut(slot)),
                          ('latch_brown', box(cu0, cv0 + 1.0, TOP, cu0 + 1.0, cv1 - 1.0,
                                              TOP + 2.0))]
    for k in range(15):
        v = centre_v - 7.0 + k
        parts.append(('gold', box(cu1 - 1.2, v - 0.15, TOP, cu1, v + 0.15, TOP + 0.25)))
    return parts


def m2_slot() -> Dict[str, List[Piece]]:
    """Return the M.2 Key E slot J18 and the standoff with its screw for the card."""
    u0, v0, u1, v1, height = M2_SLOT
    u, v = M2_STANDOFF
    return {'m2_slot': [('black_plastic', rounded_box(u0, v0, TOP, u1, v1, TOP + height,
                                                      chamfer_top=0.3))],
            'm2_standoff': [('brass', cq.Workplane('XY').polygon(6, 4.4).extrude(3.0)
                             .translate((u, v, TOP)).val()),
                            ('nickel', cylinder(2.0, (u, v, TOP + 3.0), (0, 0, 1), 1.0))]}


def solder_joints() -> List[Piece]:
    """Return the solder joints under the carrier.

    Cones round the headers' tails, and approximate joints of the port connectors' pins.
    """
    joints = []
    for u0, u1, front, back, _ in (DC_JACK, HDMI_DP, *USB_STACKS, RJ45):
        for u in (u0 + 1.0, u1 - 1.0):
            for v in (max(front, 0.0) + 2.0, back - 2.0):
                joints.append(box(u - 0.6, v - 0.3, -1.4, u + 0.6, v + 0.3, 0.0))
    u0, _, u1, _, _ = FAN_HEADER
    fan_pins = [((u0 + u1) / 2 + 2.54 * (k - 1.5), FAN_PINS_V) for k in range(4)]
    for u, v in [*GPIO_PINS, *POE_PINS, *POWER_SELECT_PINS, *fan_pins]:
        joints.append(cq.Workplane('XY').polygon(8, 1.4).workplane(offset=0.8).polygon(8, 0.6)
                      .loft(ruled=True).translate((u, v, -0.8)).mirror('XY', (0, 0, -0.4))
                      .val())
    return [('tin', joint) for joint in joints]


# ---------------------------------------------------------------------------------------------
# Mesh
# ---------------------------------------------------------------------------------------------
FRAME = ("The carrier board's bottom face at z = 0, the carrier from x = -28.982 to 71.018 mm "
         'and from y = -38.847 to 40.153 mm, the port row at +y, the 40-pin header at -x.')


def to_mesh_frame(surface: Surface) -> Surface:
    """Return the surface turned by 180 degrees about z and moved into the mesh's frame."""
    turn = np.array([[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]])
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
    """Return the kit's mesh, one named sub-part per connector, chip or group of parts.

    The module, its parts, the heatsink, its screws and the thermal pad between them are
    sub-parts of their own, so an exploded view can lift the heatsink off the module and the
    module out of its connector.
    """
    groups: List[Tuple[str, List[Piece]]] = [
        ('carrier', carrier()), ('dc_jack', dc_jack()), ('hdmi_dp', hdmi_dp()),
        ('usb_stack_j32', usb_stack(USB_STACKS[0])), ('usb_stack_j33', usb_stack(USB_STACKS[1])),
        ('ethernet', rj45()), ('micro_usb', micro_usb()), ('power_led', power_led()),
        ('sodimm_connector', sodimm_connector()), ('module_board', module_board()),
        ('module_sd_slot', module_sd_slot())]
    groups += list(module_soc().items()) + list(module_chips().items())
    groups += [('thermal_pad', thermal_pad()), ('heatsink', heatsink()),
               ('heatsink_screws', heatsink_screws())]
    groups += list(module_mounting().items()) + [('carrier_screws', carrier_screws())]
    groups += list(headers().items())
    groups += [('fan_header', fan_header()), ('capacitors', capacitors()),
               ('camera_connector_j13', camera_connector(CAMERA_CENTRES_V[0])),
               ('camera_connector_j49', camera_connector(CAMERA_CENTRES_V[1]))]
    groups += list(m2_slot().items()) + [('solder_joints', solder_joints())]
    sub_parts = [sub_part(name, parts) for name, parts in groups]
    return [Model(NAME, TITLE, FRAME, MATERIALS, sub_parts)]
