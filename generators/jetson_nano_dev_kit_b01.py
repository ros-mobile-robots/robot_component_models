# SPDX-FileCopyrightText: 2026 ROS Mobile Robots
# SPDX-License-Identifier: BSD-3-Clause
"""Generate the display model of a Jetson Nano Developer Kit B01.

The model is drawn from scratch from NVIDIA's public documents: the developer kit user guide's
top, front and rear views (connector positions and heights), the 2022 developer kit data sheet
(100 x 79 mm carrier, 30.21 mm total height), the getting started page's B01 picture (the two
camera connectors, the heatsink's fins and fan holes) and the module size (69.6 x 45 mm). The
mounting holes are from a public NVIDIA forum post. No geometry comes from a 3D model. Sources
and how each number was read in provenance/jetson_nano_dev_kit_b01.md.

Coordinates while drawing: the carrier board's frame in mm as in the user guide's top view,
u from the left edge (DC jack side) to the right edge (40-pin header side), v from the port edge
inwards, z up with the carrier's bottom face at z = 0. The meshes are turned and moved into
Remo's frame for the kit (see BOARD_TO_MESH).
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
from generators.board_parts import fin_polygon
from generators.board_parts import front_profile
from generators.board_parts import materials
from generators.board_parts import opening_with_liner
from generators.board_parts import Part
from generators.board_parts import pin_header
from generators.board_parts import place
from generators.board_parts import rj45_jack
from generators.board_parts import rounded_box
from generators.board_parts import shell_with_opening
from generators.board_parts import split_board
from generators.board_parts import spring_finger
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
DC_JACK = (1.67, 10.53, -1.97, 12.8, 11.05)
DC_JACK_HOLE = (6.2, 6.33, 9.5)  # diameter, centre height above the top face, depth
HDMI_DP = (15.0, 33.0, -1.52, 16.3, 18.6)
HDMI_OPENING = (14.0, 4.45, 1.4)  # width, height, bottom above the top face
DP_OPENING = (16.1, 4.76, 12.7)
HDMI_HOUSING_HEIGHT = 10.2  # the HDMI receptacle's black housing, the DP shell above it
USB_STACKS = ((35.8, 49.6, -1.52, 15.9, 16.0), (52.9, 66.6, -1.52, 15.9, 16.0))
RJ45 = (68.9, 85.0, -1.97, 19.8, 13.7)
MICRO_USB = (88.3, 95.8, -0.94, 4.6, 3.0)
POWER_LED = (96.4, 10.7, 98.0, 11.5)

# Module (P3448) in the SO-DIMM connector J2, and the heatsink on it.
MODULE = (13.3, 32.4, 82.9, 77.4)  # u0, v0, u1, v1: 69.6 x 45
MODULE_Z = (9.2, 10.2)
MODULE_STANDOFFS = ((16.3, 75.4), (79.9, 75.4))
SODIMM = (13.3, 28.0, 82.9, 36.6, 11.0)  # u0, v0, u1, v1, top z
SODIMM_POSTS = ((11.0, 13.3), (82.9, 85.6))  # u ranges of the connector's end posts
SODIMM_LATCHES = ((12.3, 13.1), (83.1, 83.9))  # u ranges of the metal latches, v to 54.1
SODIMM_PITCH = 0.5  # 260 contacts, 130 a row
SODIMM_CONTACTS = 130
MODULE_SD_SLOT = (36.0, 63.0, 49.5, 77.4, 7.8)  # on the module's underside, top z = module bottom

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
POWER_SELECT_PINS = ((11.73, 20.5), (14.27, 20.5))  # J48
FAN_HEADER = (72.7, 21.5, 82.9, 27.4, 6.0)  # J15: u0, v0, u1, v1, height
CAMERA_CENTRES_V = (60.0, 35.4)  # J13 and J49 along the left edge
CAMERA = (1.0, 7.5, 20.8, 2.5)  # u0, u1, length along v, height
M2_SLOT = (71.0, 46.6, 76.6, 67.6, 4.2)  # J18, Key E: u0, v0, u1, v1, height
M2_STANDOFF = (44.4, 57.25)

# Frame of Remo's earlier Jetson Nano mesh (only its bounding box, origin and axes were
# read): the carrier lies in x -28.982 ... 71.018 and y -38.847 ... 40.153 with the ports at +y,
# the 40-pin header at -x and z = 0 at the carrier's bottom face. So (u, v) maps to
# (71.018 - u, 40.153 - v): a turn by 180 degrees about z and a shift.
BOARD_TO_MESH = (71.018, 40.153)

# Materials (generators/board_parts.py): colours from NVIDIA's and CNX Software's photos.
MATERIALS = materials(['solder_mask_black', 'module_mask_green', 'fr4_edge', 'anodised_black',
                       'nickel', 'gold', 'tin', 'brass', 'black_plastic', 'usb3_blue',
                       'white_plastic', 'latch_brown', 'led_green', 'led_yellow'])
TOLERANCE = 0.03
ANGULAR_TOLERANCE = 0.35
Piece = Union[Part, Tuple[str, Surface]]


# ---------------------------------------------------------------------------------------------
# Parts
# ---------------------------------------------------------------------------------------------
def board_surface(u0: float, v0: float, u1: float, v1: float, z0: float, z1: float,
                  radius: float, holes: Sequence[Tuple[float, float]],
                  hole_diameter: float) -> Surface:
    """Return a board's tessellated surface: a rounded rectangle with holes."""
    plate = cq.Workplane('XY', origin=((u0 + u1) / 2, (v0 + v1) / 2, z0)) \
        .rect(u1 - u0, v1 - v0).extrude(z1 - z0).edges('|Z').fillet(radius).val()
    for u, v in holes:
        plate = plate.cut(cylinder(hole_diameter / 2, (u, v, z0 - 1), (0, 0, 1), z1 - z0 + 2))
    return tessellate_shape(plate, TOLERANCE, ANGULAR_TOLERANCE)


def carrier() -> List[Piece]:
    """Return the carrier board: black solder mask, FR-4 edges, unplated mounting holes."""
    surface = board_surface(0.0, 0.0, BOARD_U, BOARD_V, 0.0, BOARD_THICKNESS, CORNER_RADIUS,
                            HOLES, HOLE_DIAMETER)
    return list(split_board(surface, 'solder_mask_black'))


def dc_jack() -> List[Piece]:
    """Return the DC barrel jack J25: black housing, round opening, centre pin, contact."""
    u0, u1, front, back, height = DC_JACK
    diameter, centre_height, depth = DC_JACK_HOLE
    centre = ((u0 + u1) / 2, TOP + centre_height)
    body = rounded_box(u0, front, TOP, u1, back, TOP + height, chamfer_top=0.4)
    hole = cylinder(diameter / 2, (centre[0], front - 0.1, centre[1]), (0, 1, 0), depth)
    recess = box(u0 + 0.6, front - 0.1, TOP + 0.6, u1 - 0.6, front + 0.4, TOP + height - 0.6)
    pin = cylinder(1.0, (centre[0], front + 2.0, centre[1]), (0, 1, 0), depth - 1.9)
    contact = box(u0 - 0.0, back - 4.0, TOP, u0 + 0.3, back - 1.0, TOP + 4.0)
    return [('black_plastic', body.cut(hole).cut(recess)), ('tin', pin), ('nickel', contact)]


def hdmi_dp_stack() -> List[Part]:
    """Return the HDMI and DisplayPort stack J6 in its own frame (opening towards +x).

    The HDMI receptacle sits in a black housing with a metal frame round its opening, the
    DisplayPort receptacle above it in a metal shell with spring fingers.
    """
    u0, u1, front, back, height = HDMI_DP
    depth, half = back - front, (u1 - u0) / 2
    parts: List[Part] = []
    housing = rounded_box(-depth, -half, 0.0, 0.0, half, HDMI_HOUSING_HEIGHT, vertical=0.3)
    opening_width, opening_height, bottom = HDMI_OPENING
    profile = chamfered_rectangle(opening_width, opening_height, 1.3, bottom)
    cut, liner = opening_with_liner(profile, 9.0, 0.25)
    frame_outer = chamfered_rectangle(opening_width + 0.9, opening_height + 0.9, 1.6,
                                      bottom - 0.45)
    frame = front_profile(frame_outer, 0.0, 1.5).cut(front_profile(profile, 0.1, 1.7))
    parts += [('black_plastic', housing.cut(cut)), ('black_plastic', liner), ('nickel', frame)]
    middle = bottom + opening_height / 2
    parts.append(('black_plastic', box(-8.9, -5.0, middle - 0.7, -1.5, 5.0, middle + 0.7)))
    for k in range(10):
        y = -4.4 + k * 0.98
        parts.append(('gold', box(-7.0, y - 0.25, middle + 0.7, -2.5, y + 0.25, middle + 0.75)))
    shell = rounded_box(-depth, -half, HDMI_HOUSING_HEIGHT, -0.3, half, height, vertical=0.4)
    shell_frame = box(-0.3, -half, HDMI_HOUSING_HEIGHT, 0.0, half, height)
    opening_width, opening_height, bottom = DP_OPENING
    profile = chamfered_rectangle(opening_width, opening_height, 1.0, bottom, 'one')
    cut, liner = opening_with_liner(profile, 9.0, 0.2)
    shell = shell.cut(cut)
    shell_frame = shell_frame.cut(cut)
    for y in (-4.5, 4.5):
        finger_cut, tab = spring_finger(-1.2, y, height)
        shell = shell.cut(finger_cut)
        parts.append(('nickel', tab))
    middle = bottom + opening_height / 2
    parts += [('nickel', shell), ('nickel', shell_frame), ('black_plastic', liner),
              ('black_plastic', box(-8.9, -6.5, middle - 0.5, -1.5, 6.5, middle + 0.5))]
    for k in range(10):
        y = -5.85 + k * 1.3
        parts.append(('gold', box(-7.0, y - 0.3, middle + 0.5, -2.5, y + 0.3, middle + 0.55)))
    return parts


def hdmi_dp() -> List[Piece]:
    """Return the HDMI and DisplayPort stack J6, its openings towards -v."""
    u0, u1, front, _, _ = HDMI_DP
    return list(place(hdmi_dp_stack(), -90.0, ((u0 + u1) / 2, front, TOP)))


def usb_stacks() -> List[Piece]:
    """Return the two stacked USB 3.0 Type-A receptacles J32 and J33, openings towards -v."""
    parts: List[Piece] = []
    for u0, u1, front, back, height in USB_STACKS:
        parts += place(usb_a_stack(u1 - u0, back - front, height, 'usb3_blue'), -90.0,
                       ((u0 + u1) / 2, front, TOP))
    return parts


def rj45() -> List[Piece]:
    """Return the Gigabit Ethernet jack J43, its opening towards -v."""
    u0, u1, front, back, height = RJ45
    return list(place(rj45_jack(u1 - u0, back - front, height), -90.0,
                      ((u0 + u1) / 2, front, TOP)))


def micro_usb() -> List[Piece]:
    """Return the micro-USB receptacle J28 and the green power LED DS3."""
    u0, u1, front, back, height = MICRO_USB
    opening = chamfered_rectangle(6.0, 1.9, 0.6, 0.55)
    receptacle = shell_with_opening(u1 - u0 - 0.8, back - front, height, opening, 4.5, 0.4,
                                    contacts=(1.6, 1.3, 5), tongue=(1.9, 1.15, 1.6))
    receptacle.append(('nickel', box(-2.0, -(u1 - u0) / 2, 0.0, -1.0, (u1 - u0) / 2, 0.4)))
    parts: List[Piece] = list(place(receptacle, -90.0, ((u0 + u1) / 2, front, TOP)))
    u0, v0, u1, v1 = POWER_LED
    parts.append(('led_green', box(u0, v0, TOP, u1, v1, TOP + 0.6)))
    return parts


def module() -> List[Piece]:
    """Return the SO-DIMM connector, the module with its microSD slot, standoffs and screws."""
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
    mu0, mv0, mu1, mv1 = MODULE
    surface = board_surface(mu0, mv0, mu1, mv1, MODULE_Z[0], MODULE_Z[1], 0.8, MODULE_STANDOFFS,
                            2.8)
    parts += split_board(surface, 'module_mask_green')
    for u, v in MODULE_STANDOFFS:
        standoff = cq.Workplane('XY').polygon(6, 5.0).extrude(MODULE_Z[0] - TOP) \
            .translate((u, v, TOP)).val()
        head = cq.Workplane('XY').circle(2.2).extrude(1.3).faces('>Z').edges().fillet(0.5) \
            .translate((u, v, MODULE_Z[1])).val()
        head = head.cut(box(u - 1.2, v - 0.25, MODULE_Z[1] + 0.8, u + 1.2, v + 0.25,
                            MODULE_Z[1] + 1.5))
        head = head.cut(box(u - 0.25, v - 1.2, MODULE_Z[1] + 0.8, u + 0.25, v + 1.2,
                            MODULE_Z[1] + 1.5))
        parts += [('brass', standoff), ('nickel', head)]
    su0, sv0, su1, sv1, sz0 = MODULE_SD_SLOT
    socket = box(su0, sv0, sz0, su1, sv1, MODULE_Z[0])
    sd_slot = box(su0 + 1.2, sv1 - 8.0, sz0 + 0.3, su1 - 1.2, sv1 + 0.1, MODULE_Z[0] - 0.3)
    parts.append(('nickel', socket.cut(sd_slot)))
    return parts


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
    """Return the heatsink: the extruded profile with the four fan holes in its flanges."""
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
    return [('anodised_black', shape.clean())]


def carrier_parts() -> List[Piece]:
    """Return the headers, camera connectors, fan header, M.2 slot and the M.2 standoff."""
    parts: List[Piece] = []
    for pins in (GPIO_PINS, POE_PINS, POWER_SELECT_PINS):
        parts += pin_header(pins, TOP, 0.0, HEADER_PIN_HEIGHT, PIN_TAIL)
    u0, v0, u1, v1, height = FAN_HEADER
    shroud = rounded_box(u0, v0, TOP, u1, v1, TOP + height, vertical=0.3)
    pocket = box(u0 + 0.6, v0 + 0.6, TOP + 1.0, u1 - 0.6, v1 - 0.9, TOP + height + 0.1)
    parts.append(('white_plastic', shroud.cut(pocket)))
    for k in range(4):
        u = (u0 + u1) / 2 + 2.54 * (k - 1.5)
        v = (v0 + v1) / 2 - 0.15
        parts.append(('gold', box(u - 0.32, v - 0.32, TOP, u + 0.32, v + 0.32,
                                  TOP + height - 1.0)))
    cu0, cu1, length, height = CAMERA
    for centre_v in CAMERA_CENTRES_V:
        cv0, cv1 = centre_v - length / 2, centre_v + length / 2
        body = rounded_box(cu0 + 1.0, cv0, TOP, cu1 - 1.0, cv1, TOP + height, vertical=0.3)
        slot = box(cu0 + 3.0, cv0 + 2.2, TOP + 0.9, cu1 - 0.9, cv1 - 2.2, TOP + 1.5)
        parts.append(('black_plastic', body.cut(slot)))
        parts.append(('latch_brown', box(cu1 - 1.0, cv0 + 1.0, TOP, cu1, cv1 - 1.0, TOP + 2.0)))
        for k in range(15):
            v = centre_v - 7.0 + k
            parts.append(('gold', box(cu0, v - 0.15, TOP, cu0 + 1.2, v + 0.15, TOP + 0.25)))
    u0, v0, u1, v1, height = M2_SLOT
    parts.append(('black_plastic', rounded_box(u0, v0, TOP, u1, v1, TOP + height,
                                               chamfer_top=0.3)))
    u, v = M2_STANDOFF
    parts.append(('brass', cq.Workplane('XY').polygon(6, 4.4).extrude(3.0)
                  .translate((u, v, TOP)).val()))
    parts.append(('nickel', cylinder(2.0, (u, v, TOP + 3.0), (0, 0, 1), 1.0)))
    return parts


def bottom_side() -> List[Piece]:
    """Return solder joints of the through-hole connectors under the carrier (approximate)."""
    joints = []
    for u0, u1, front, back, _ in (DC_JACK, HDMI_DP, *USB_STACKS, RJ45):
        for u in (u0 + 1.0, u1 - 1.0):
            for v in (max(front, 0.0) + 2.0, back - 2.0):
                joints.append(box(u - 0.6, v - 0.3, -1.4, u + 0.6, v + 0.3, 0.0))
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
    """Return the kit's mesh, one sub-part per connector or group of parts."""
    groups = [('carrier', carrier()), ('dc_jack', dc_jack()), ('hdmi_dp', hdmi_dp()),
              ('usb_stacks', usb_stacks()), ('ethernet', rj45()), ('micro_usb', micro_usb()),
              ('module', module()), ('heatsink', heatsink()), ('carrier_parts', carrier_parts()),
              ('bottom_side', bottom_side())]
    sub_parts = [sub_part(name, parts) for name, parts in groups]
    return [Model(NAME, TITLE, FRAME, MATERIALS, sub_parts)]
