# Copyright (c) 2026, ROS Mobile Robots
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

import cadquery as cq
import numpy as np

from generators.mesh_export import Model
from generators.mesh_export import phong
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
PIN_SIZE = 0.64
HEADER_BASE_HEIGHT = 2.5
HEADER_PIN_HEIGHT = 8.5
PIN_TAIL = 1.95  # Solder tails below the carrier, from the 30.21 mm total height

# Port row, facing -v: (u0, u1, front v, back v, height above the carrier's top face).
DC_JACK = (1.67, 10.53, -1.97, 12.8, 11.05)
DC_JACK_HOLE = (6.2, 6.33, 9.5)  # diameter, centre height above the top face, depth
HDMI_DP = (15.0, 33.0, -1.52, 16.3, 18.6)
HDMI_OPENING = (14.0, 4.45, 1.4)  # width, height, bottom above the top face
DP_OPENING = (16.1, 4.76, 12.7)
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
MODULE_SD_SLOT = (36.0, 63.0, 49.5, 77.4, 7.8)  # on the module's underside, top z = module bottom
HEATSINK = (18.4, 37.0, 77.4, 77.4)  # u0, v0, u1, v1
HEATSINK_BASE_Z = (11.0, 15.0)
HEATSINK_TOP = 28.25
# Fins along u: (centre v, thickness, top z, width of the flat top or 0)
FINS = ((37.6, 1.2, 24.5, 0.0), (42.15, 1.6, HEATSINK_TOP, 6.5), (50.15, 1.2, 27.0, 0.0),
        (58.15, 1.2, 27.0, 0.0), (66.15, 1.2, 27.0, 0.0), (74.15, 1.6, HEATSINK_TOP, 6.5))
FAN_HOLES_U = (31.9, 63.9)  # 32 mm apart like a 40 mm fan's holes
FAN_HOLES_V = (42.15, 74.15)
FAN_HOLE_DIAMETER = 2.5

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

COLOURS = {
    'pcb_black': (0.07, 0.07, 0.08, 1.0),
    'module_green': (0.16, 0.44, 0.19, 1.0),
    'heatsink': (0.13, 0.13, 0.14, 1.0),
    'black': (0.04, 0.04, 0.04, 1.0),
    'silver': (0.78, 0.78, 0.80, 1.0),
    'gold': (0.86, 0.69, 0.27, 1.0),
    'usb3_blue': (0.10, 0.38, 0.85, 1.0),
    'white': (0.92, 0.91, 0.86, 1.0),
    'latch_brown': (0.30, 0.22, 0.14, 1.0),
    'led_green': (0.20, 0.85, 0.25, 1.0),
    'led_yellow': (0.95, 0.80, 0.10, 1.0),
}
METALS = ('gold', 'silver')
# A darker ambient than diffuse keeps the shading in Gazebo, whose scenes have ambient light.
AMBIENT_FACTOR = 0.6
MATERIALS = {name: phong(rgba, (0.5, 0.5, 0.5) if name in METALS else (0.1, 0.1, 0.1),
                         30.0 if name in METALS else 10.0, name in METALS, AMBIENT_FACTOR)
             for name, rgba in COLOURS.items()}
TOLERANCE = 0.03
ANGULAR_TOLERANCE = 0.35

Part = Tuple[str, cq.Shape]


# ---------------------------------------------------------------------------------------------
# Shape helpers
# ---------------------------------------------------------------------------------------------
def box(x0: float, y0: float, z0: float, x1: float, y1: float, z1: float) -> cq.Shape:
    """Return the box between two corners."""
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, cq.Vector(x0, y0, z0))


def cylinder(radius: float, base: Tuple[float, float, float],
             direction: Tuple[float, float, float], length: float) -> cq.Shape:
    """Return a cylinder from its base centre along the direction."""
    return cq.Solid.makeCylinder(radius, length, cq.Vector(*base), cq.Vector(*direction))


def chamfered_profile(width: float, height: float, chamfer: float, centre_x: float,
                      bottom_z: float) -> List[Tuple[float, float]]:
    """Return the (x, z) outline of an opening: a rectangle with chamfered lower corners."""
    half = width / 2
    points = [(-half + chamfer, 0), (half - chamfer, 0), (half, chamfer), (half, height),
              (-half, height), (-half, chamfer)]
    return [(centre_x + px, bottom_z + pz) for px, pz in points]


def profile_prism(points: Sequence[Tuple[float, float]], front_y: float, depth: float,
                  inset: float = 0.0) -> cq.Shape:
    """Return the (x, z) outline extruded from front_y along +y, optionally inset."""
    sketch = cq.Workplane('XZ', origin=(0, front_y, 0)).polyline(list(points)).close()
    if inset:
        sketch = sketch.offset2D(-inset)
    return sketch.extrude(-depth).val()


def opening_and_liner(points: Sequence[Tuple[float, float]], front_y: float, depth: float,
                      wall: float) -> Tuple[cq.Shape, cq.Shape]:
    """Return the cut for an opening in a front face and a dark liner for its walls and back."""
    opening = profile_prism(points, front_y - 0.1, depth + 0.1)
    liner = profile_prism(points, front_y, depth).cut(
        profile_prism(points, front_y - 0.1, depth - wall + 0.1, wall))
    return opening, liner


def header(pins: Sequence[Tuple[float, float]], base: Tuple[float, float, float, float]) \
        -> List[Part]:
    """Return a 2.54 mm pin header: black base and gold square pins with solder tails."""
    half = PIN_SIZE / 2
    parts: List[Part] = [('black', box(base[0], base[1], TOP, base[2], base[3],
                                       TOP + HEADER_BASE_HEIGHT))]
    parts += [('gold', box(u - half, v - half, -PIN_TAIL, u + half, v + half,
                           TOP + HEADER_PIN_HEIGHT)) for u, v in pins]
    return parts


# ---------------------------------------------------------------------------------------------
# Parts
# ---------------------------------------------------------------------------------------------
def carrier() -> List[Part]:
    """Return the carrier board with its four mounting holes."""
    plate = cq.Workplane('XY', origin=(BOARD_U / 2, BOARD_V / 2, 0)).rect(BOARD_U, BOARD_V) \
        .extrude(BOARD_THICKNESS).edges('|Z').fillet(CORNER_RADIUS).val()
    for u, v in HOLES:
        plate = plate.cut(cylinder(HOLE_DIAMETER / 2, (u, v, -1), (0, 0, 1), 5))
    return [('pcb_black', plate)]


def dc_jack() -> List[Part]:
    """Return the DC barrel jack J25 with its hole and centre pin."""
    u0, u1, front, back, height = DC_JACK
    diameter, centre_height, depth = DC_JACK_HOLE
    centre = ((u0 + u1) / 2, TOP + centre_height)
    body = box(u0, front, TOP, u1, back, TOP + height)
    hole = cylinder(diameter / 2, (centre[0], front - 0.1, centre[1]), (0, 1, 0), depth)
    pin = cylinder(1.0, (centre[0], front + 2.0, centre[1]), (0, 1, 0), depth - 1.9)
    return [('black', body.cut(hole)), ('silver', pin)]


def hdmi_dp() -> List[Part]:
    """Return the stacked HDMI (below) and DisplayPort (above) receptacle J6."""
    u0, u1, front, back, height = HDMI_DP
    centre_u = (u0 + u1) / 2
    shell = box(u0, front, TOP, u1, back, TOP + height)
    parts: List[Part] = []
    for (width, opening_height, bottom), chamfer, tongue in (
            (HDMI_OPENING, 1.3, (10.0, 1.4)), (DP_OPENING, 1.0, (13.0, 1.0))):
        profile = chamfered_profile(width, opening_height, chamfer, centre_u, TOP + bottom)
        opening, liner = opening_and_liner(profile, front, 9.0, 0.25)
        shell = shell.cut(opening)
        parts.append(('black', liner))
        middle = TOP + bottom + opening_height / 2
        parts.append(('black', box(centre_u - tongue[0] / 2, front + 1.5, middle - tongue[1] / 2,
                                   centre_u + tongue[0] / 2, front + 8.9, middle + tongue[1] / 2)))
        for k in range(9):
            u = centre_u - tongue[0] / 2 + 0.8 + k * (tongue[0] - 1.6) / 8
            parts.append(('gold', box(u - 0.2, front + 2.5, middle + tongue[1] / 2, u + 0.2,
                                      front + 7.0, middle + tongue[1] / 2 + 0.06)))
    return [('silver', shell)] + parts


def usb_stacks() -> List[Part]:
    """Return the two stacked USB 3.0 Type A receptacles J32 and J33 with blue tongues."""
    parts: List[Part] = []
    for u0, u1, front, back, height in USB_STACKS:
        centre_u = (u0 + u1) / 2
        shell = box(u0, front + 0.3, TOP, u1, back, TOP + height)
        flange = box(u0 - 0.35, front, TOP, u1 + 0.35, front + 0.3, TOP + height)
        for bottom in (TOP + 1.5, TOP + 9.3):
            opening = box(centre_u - 6.25, front - 0.1, bottom, centre_u + 6.25, front + 14.0,
                          bottom + 5.12)
            shell = shell.cut(opening)
            flange = flange.cut(opening)
            parts.append(('black', box(centre_u - 6.25, front + 13.5, bottom, centre_u + 6.25,
                                       front + 14.0, bottom + 5.12)))
            parts.append(('usb3_blue', box(centre_u - 5.55, front + 1.0, bottom + 2.7,
                                           centre_u + 5.55, front + 13.5, bottom + 4.55)))
            for k in range(4):
                u = centre_u - 3.5 + 2.33 * k
                parts.append(('gold', box(u - 0.5, front + 2.0, bottom + 2.62, u + 0.5,
                                          front + 9.0, bottom + 2.7)))
        parts += [('silver', shell), ('silver', flange)]
    return parts


def rj45() -> List[Part]:
    """Return the Gigabit Ethernet jack J43: shield, dark opening, contacts and two LEDs."""
    u0, u1, front, back, height = RJ45
    centre_u = (u0 + u1) / 2
    shell = box(u0, front, TOP, u1, back, TOP + height)
    bottom = TOP + 2.2
    profile = [(centre_u - 5.85, bottom + 1.6), (centre_u - 3.0, bottom + 1.6),
               (centre_u - 3.0, bottom), (centre_u + 3.0, bottom), (centre_u + 3.0, bottom + 1.6),
               (centre_u + 5.85, bottom + 1.6), (centre_u + 5.85, bottom + 9.0),
               (centre_u - 5.85, bottom + 9.0)]
    opening, liner = opening_and_liner(profile, front, 12.0, 0.3)
    parts: List[Part] = [('silver', shell.cut(opening)), ('black', liner)]
    for k in range(8):
        u = centre_u - 3.57 + 1.02 * k
        parts.append(('gold', box(u - 0.15, front + 4.0, bottom + 8.3, u + 0.15, front + 9.0,
                                  bottom + 8.7)))
    parts.append(('led_green', box(u0 + 0.8, front - 0.1, TOP + 0.8, u0 + 2.8, front + 0.05,
                                   TOP + 2.0)))
    parts.append(('led_yellow', box(u1 - 2.8, front - 0.1, TOP + 0.8, u1 - 0.8, front + 0.05,
                                    TOP + 2.0)))
    return parts


def micro_usb() -> List[Part]:
    """Return the micro-USB receptacle J28 and the green power LED DS3."""
    u0, u1, front, back, height = MICRO_USB
    centre_u = (u0 + u1) / 2
    shell = box(u0 + 0.4, front, TOP, u1 - 0.4, back, TOP + height)
    ears = box(u0, front + 1.0, TOP, u1, front + 2.0, TOP + 0.4)
    opening = profile_prism(chamfered_profile(6.0, 1.9, 0.6, centre_u, TOP + 0.55), front - 0.1,
                            4.5)
    tongue = box(centre_u - 1.9, front + 0.6, TOP + 1.65, centre_u + 1.9, front + 4.4, TOP + 2.2)
    u0, v0, u1, v1 = POWER_LED
    return [('silver', shell.fuse(ears).cut(opening)), ('black', tongue),
            ('led_green', box(u0, v0, TOP, u1, v1, TOP + 0.6))]


def module() -> List[Part]:
    """Return the SO-DIMM connector, the module with its microSD slot, standoffs and screws."""
    u0, v0, u1, v1, top = SODIMM
    connector = box(u0, v0, TOP, u1, v1, top)
    slot = box(u0 + 2.0, v0 + 3.0, MODULE_Z[0] - 0.3, u1 - 2.0, v1 + 0.1, MODULE_Z[1] + 0.3)
    parts: List[Part] = [('black', connector.cut(slot))]
    for pu0, pu1 in SODIMM_POSTS:
        parts.append(('black', box(pu0, v0, TOP, pu1, v1, top)))
    for lu0, lu1 in SODIMM_LATCHES:
        parts.append(('silver', box(lu0, v1, TOP, lu1, 54.1, MODULE_Z[1] + 0.5)))
    mu0, mv0, mu1, mv1 = MODULE
    board = box(mu0, mv0, MODULE_Z[0], mu1, mv1, MODULE_Z[1])
    for u, v in MODULE_STANDOFFS:
        board = board.cut(cylinder(1.4, (u, v, MODULE_Z[0] - 0.1), (0, 0, 1), 1.5))
        parts.append(('silver', cylinder(2.25, (u, v, TOP), (0, 0, 1), MODULE_Z[0] - TOP)))
        parts.append(('silver', cylinder(2.25, (u, v, MODULE_Z[1]), (0, 0, 1), 1.3)))
    parts.append(('module_green', board))
    su0, sv0, su1, sv1, sz0 = MODULE_SD_SLOT
    socket = box(su0, sv0, sz0, su1, sv1, MODULE_Z[0])
    sd_slot = box(su0 + 1.2, sv1 - 8.0, sz0 + 0.3, su1 - 1.2, sv1 + 0.1, MODULE_Z[0] - 0.3)
    parts.append(('silver', socket.cut(sd_slot)))
    return parts


def heatsink() -> List[Part]:
    """Return the heatsink: a base on the module and fins along u, two with flat fan mounts."""
    u0, v0, u1, v1 = HEATSINK
    base_z0, base_z1 = HEATSINK_BASE_Z
    shape = box(u0, v0, base_z0, u1, v1, base_z1)
    for centre_v, thickness, top, flat in FINS:
        fin_v0 = max(v0, centre_v - thickness / 2)
        fin_v1 = min(v1, centre_v + thickness / 2)
        shape = shape.fuse(box(u0, fin_v0, base_z1 - 0.01, u1, fin_v1, top))
        if flat:
            flat_v0 = max(v0, centre_v - flat / 2)
            flat_v1 = min(v1, centre_v + flat / 2)
            shape = shape.fuse(box(u0, flat_v0, top - 1.5, u1, flat_v1, top))
    for u in FAN_HOLES_U:
        for v in FAN_HOLES_V:
            hole = cylinder(FAN_HOLE_DIAMETER / 2, (u, v, HEATSINK_TOP - 4.0), (0, 0, 1), 4.1)
            shape = shape.cut(hole)
    return [('heatsink', shape.clean())]


def carrier_parts() -> List[Part]:
    """Return the headers, camera connectors, fan header, M.2 slot and the M.2 standoff."""
    parts = header(GPIO_PINS, (87.46, 20.6, 92.54, 71.4))
    parts += header(POE_PINS, (93.6, 72.2, 98.7, 77.3))
    parts += header(POWER_SELECT_PINS, (10.45, 19.23, 15.55, 21.77))
    u0, v0, u1, v1, height = FAN_HEADER
    shroud = box(u0, v0, TOP, u1, v1, TOP + height)
    pocket = box(u0 + 0.6, v0 + 0.6, TOP + 1.0, u1 - 0.6, v1 - 0.9, TOP + height + 0.1)
    parts.append(('white', shroud.cut(pocket)))
    for k in range(4):
        u = (u0 + u1) / 2 + 2.54 * (k - 1.5)
        v = (v0 + v1) / 2 - 0.15
        parts.append(('gold', box(u - 0.32, v - 0.32, TOP, u + 0.32, v + 0.32,
                                  TOP + height - 1.0)))
    cu0, cu1, length, height = CAMERA
    for centre_v in CAMERA_CENTRES_V:
        cv0, cv1 = centre_v - length / 2, centre_v + length / 2
        body = box(cu0 + 1.0, cv0, TOP, cu1 - 1.0, cv1, TOP + height)
        slot = box(cu0 + 3.0, cv0 + 2.2, TOP + 0.9, cu1 - 0.9, cv1 - 2.2, TOP + 1.5)
        parts.append(('black', body.cut(slot)))
        parts.append(('latch_brown', box(cu1 - 1.0, cv0 + 1.0, TOP, cu1, cv1 - 1.0, TOP + 2.0)))
        for k in range(15):
            v = centre_v - 7.0 + k
            parts.append(('gold', box(cu0, v - 0.15, TOP, cu0 + 1.2, v + 0.15, TOP + 0.25)))
    u0, v0, u1, v1, height = M2_SLOT
    parts.append(('black', box(u0, v0, TOP, u1, v1, TOP + height)))
    u, v = M2_STANDOFF
    parts.append(('silver', cylinder(2.0, (u, v, TOP), (0, 0, 1), 3.0)))
    parts.append(('silver', cylinder(2.0, (u, v, TOP + 3.0), (0, 0, 1), 1.0)))
    return parts


def bottom_side() -> List[Part]:
    """Return solder joints of the through-hole connectors under the carrier (approximate)."""
    joints = []
    for u0, u1, front, back, _ in (DC_JACK, HDMI_DP, *USB_STACKS, RJ45):
        for u in (u0 + 1.0, u1 - 1.0):
            for v in (max(front, 0.0) + 2.0, back - 2.0):
                joints.append(box(u - 0.6, v - 0.3, -1.4, u + 0.6, v + 0.3, 0.0))
    return [('silver', joint) for joint in joints]


# ---------------------------------------------------------------------------------------------
# Mesh
# ---------------------------------------------------------------------------------------------
FRAME = ("The carrier board's bottom face at z = 0, the carrier from x = -28.982 to 71.018 mm "
         'and from y = -38.847 to 40.153 mm, the port row at +y, the 40-pin header at -x.')


def to_mesh_frame(surface: Surface) -> Surface:
    """Return the surface turned by 180 degrees about z and moved into the mesh's frame."""
    turn = np.array([[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]])
    return transform_surface(surface, turn, (BOARD_TO_MESH[0], BOARD_TO_MESH[1], 0.0))


def sub_part(name: str, parts: Sequence[Part]) -> SubPart:
    """Return a named sub-part from its (material, shape) pieces."""
    pieces = [(material, to_mesh_frame(tessellate_shape(shape, TOLERANCE, ANGULAR_TOLERANCE)))
              for material, shape in parts]
    return SubPart(name, surfaces_by_material(pieces, list(MATERIALS)))


def build() -> List[Model]:
    """Return the kit's mesh, one sub-part per connector or group of parts."""
    groups = [('carrier', carrier()), ('dc_jack', dc_jack()), ('hdmi_dp', hdmi_dp()),
              ('usb_stacks', usb_stacks()), ('ethernet', rj45()), ('micro_usb', micro_usb()),
              ('module', module()), ('heatsink', heatsink()), ('carrier_parts', carrier_parts()),
              ('bottom_side', bottom_side())]
    sub_parts = [sub_part(name, parts) for name, parts in groups]
    return [Model(NAME, TITLE, FRAME, MATERIALS, sub_parts)]
