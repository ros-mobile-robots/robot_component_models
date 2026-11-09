# SPDX-FileCopyrightText: 2026 ROS Mobile Robots
# SPDX-License-Identifier: BSD-3-Clause
"""Generate a 360 degree laser scanner of the RPLIDAR A1 type: base, motor, belt, rotating head.

Modelled from the public datasheet's mechanical drawing only (RPLIDAR A1 datasheet, model A1M8,
rev. 3.0, Figure 5-2 "The Mechanical Dimensions of RPLIDAR A1"): the base's outline and its four
mounting holes, the motor, the belt with its two pulleys, the head with its two openings, and the
board with its connectors under the base. Values that the drawing doesn't dimension are read off
its scale. The colours follow Slamtec's product pictures. No logos and no printed text. Sources
in provenance/rplidar_a1.md.

Frame: the head's rotation axis along z, z = 0 in the scan plane (42 mm above the motor's
bottom), the base's round end towards +x, the motor's axis at x = -45.5 mm, the window's middle
towards -x. Positions in mm, the files are in metres.
"""
import math
from typing import Dict
from typing import List
from typing import Sequence
from typing import Tuple

import cadquery as cq

from generators.mesh_export import Model
from generators.mesh_export import phong
from generators.mesh_export import SubPart
from generators.mesh_export import tessellate_faces

TITLE = '360 degree laser scanner, RPLIDAR A1 size'

# ---------------------------------------------------------------------------------------------
# Parameters, in mm. Heights are above the motor's bottom, the drawing's reference. Values of the
# datasheet's drawing are marked (DS), values read off the drawing's scale (drawing) and choices
# (choice).
# ---------------------------------------------------------------------------------------------
SCAN_HEIGHT = 42.0           # DS: the laser working area's middle, z = 0 of the mesh's frame
TOTAL_HEIGHT = 51.0          # DS: the head's top
LENGTH = 96.74               # DS: from the base's round end to its motor end
WIDTH = 70.28                # DS: the base's width, at the head's axis
ROUND_END = 35.14            # DS: from the head's axis to the base's round end
MOTOR_OFFSET = 45.50         # DS: from the head's axis to the motor's axis
MOTOR_DIAMETER = 32.0        # DS
HEAD_DIAMETER = 70.04        # DS
HEAD_BOTTOM = 31.5           # DS: the head's cylinder starts above its pulley
BASE_BOTTOM = 21.0           # DS
BASE_TOP = 26.5              # DS
PCB_BOTTOM = 15.0            # DS: 6 below the base
MOUNT_HOLE_DIAMETER = 3.0    # DS: 4 x diameter 3
# DS: two holes 56 apart, 28 from the head's axis towards the round end, two 40 apart, 42 from it
# towards the motor
MOUNT_HOLES = ((28.0, 28.0), (28.0, -28.0), (-42.0, 20.0), (-42.0, -20.0))

# The base's outline: the long sides are arcs through the widest point at the head's axis and
# tangent to the motor end's arc (drawing), so their radius follows from the DS values: 79.99.
MOTOR_END_RADIUS = LENGTH - ROUND_END - MOTOR_OFFSET  # 16.10: the arc around the motor's axis
END_RADIUS = 90.0            # drawing: the round end's flat arc between the two ears
EAR_RADIUS = 3.3             # drawing: the outline around the two holes at the round end
TOP_EDGE_ROUND = 0.6         # drawing: a second line about 0.5 inside the outline
SPOT_FACE = (5.7, 0.4)       # drawing: diameter of the ring around each hole; depth: choice
PLATE_THICKNESS = 2.5        # choice: the base is a shell, open at the bottom (bottom view)
WALL_THICKNESS = 1.6         # choice
BOSS_DIAMETER = 5.7          # drawing: around each hole, bottom view
HOUSING_RADIUS = 29.5        # drawing: the block under the base, side view
HOUSING_BOTTOM = 17.8        # drawing

# Motor below the base's narrow end, its pulley above
MOTOR_CAN = (3.5, 20.9)      # drawing: the can's bottom and top
MOTOR_CAN_ROUND = (2.0, 0.5)  # drawing: the can's lower and upper rims
MOTOR_BOSS = (7.0, 0.0)      # drawing: the boss below the can, diameter and bottom
MOTOR_SCREW_Y = 11.25        # drawing: two screws either side of the pulley
MOTOR_SCREW_HEAD = (3.7, 1.0)  # drawing: head diameter; height: choice
PULLEY_RADII = (9.0, 8.7, 6.0, 2.5)  # drawing: flanges, belt seat, top step, top boss
PULLEY_HEIGHTS = (26.8, 27.6, 31.0, 31.6, 32.0, 32.4)  # choice, inside the drawing's pulley band:
# bottom, lower flange's top, upper flange's bottom, cap, step, boss
BELT_OUTER = (35.5, 9.5)     # drawing: the belt's outer radius around the head and the pulley
BELT_THICKNESS = 0.8         # choice
BELT_HEIGHTS = (28.2, 30.8)  # drawing: between the side view's lines at 27.5 and 31.0
HEAD_PULLEY = (26.7, 27.6, 35.2)  # choice: bottom, lower flange's top, flange radius

# Head
HEAD_TOP_ROUND = 1.0         # drawing: side view 50.0 to 51.0, top view R34.06
HEAD_BOTTOM_CHAMFER = 0.3    # choice
LOGO_RING = (14.44, 14.96)   # drawing: the groove around the badge, inner and outer radius
LOGO_DEPTH = 0.3             # choice
TOP_SCREW_RADIUS = 28.0      # drawing
TOP_SCREW_ANGLES = (3.0, -128.0, 138.0)  # drawing: from the window's middle, anticlockwise
TOP_SCREW_RECESS = (5.0, 2.0)  # drawing: diameter 5.06; depth: choice, the screws left out
WINDOW_AZIMUTH = 180.0       # choice: the window's middle towards -x, the motor's side
LENS_ANGLE = 32.5            # drawing: the receiver's opening, from the window's middle
EMITTER_ANGLE = -32.5        # drawing: the emitter's opening, on the other side
LENS_OPENING = (17.5, 10.0, 3.5)  # drawing: width along the head, height, corner radius
EMITTER_OPENING = (9.6, 7.8)  # drawing: a rounded slot, width and height
OPENING_DEPTH = 5.0          # choice
PANE_THICKNESS = 0.8         # choice: dark glass at the bottom of each opening
LENS = (6.0, 8.5, 0.6)       # drawing: lens diameter; barrel diameter, lens proud: choice
EMITTER_LENS = (2.6, 4.4, 0.6)  # drawing: lens diameter; barrel diameter, lens proud: choice
BARREL_HEIGHT = 1.2          # choice

# Board under the base, its parts and connectors (bottom view, y as seen from above)
PCB_X = (-28.5, 17.2)        # drawing
PCB_HALF_WIDTH = 28.6        # drawing
PCB_CORNER = 2.9             # drawing
PCB_THICKNESS = 1.6          # drawing: 15.0 to 16.6
PCB_HOLES = (22.5, 2.2)      # drawing: two holes at x = 0, y = +-22.5, diameter
PCB_SCREWS = (11.2, 2.3, 0.8)  # drawing: four screws at x, y = +-11.2, head diameter; height:
# choice
SQUARE_PART = (-20.6, -13.7, 17.7, 25.0, 3.8)  # drawing: x from, x to, y from, y to, height
ROUND_PART = (-23.8, 15.9, 5.6, 7.5)  # drawing: x, y, diameter; DS: 7.5 high
CONNECTORS = ((-19.1, 2), (-9.0, 3), (2.7, 4))  # drawing: y of the middle and pins; the 3-pin
# (motor) and 4-pin (data) connectors are PH1.25 vertical headers (DS)
CONNECTOR_X = -24.8          # drawing
CONNECTOR_PITCH = 1.25       # DS
CONNECTOR_SIZE = (3.0, 4.6, 0.6)  # drawing: depth, height (end view); wall: choice
CONNECTOR_TAB = (1.3, 2.6, 2.0)  # drawing: width beyond each end, depth; height: choice
PIN_SIZE = 0.45              # choice

LINEAR_DEFLECTION = 0.04
ANGULAR_DEFLECTION = 0.3

MATERIALS = {
    'body': phong((0.09, 0.09, 0.10, 1.0), (0.15, 0.15, 0.15, 1.0), 12.0),
    'rubber': phong((0.04, 0.04, 0.04, 1.0), (0.05, 0.05, 0.05, 1.0), 4.0),
    'steel': phong((0.76, 0.77, 0.79, 1.0), (0.60, 0.60, 0.60, 1.0), 40.0, metallic=True),
    'screw': phong((0.30, 0.30, 0.32, 1.0), (0.50, 0.50, 0.50, 1.0), 40.0, metallic=True),
    'pcb': phong((0.10, 0.42, 0.22, 1.0), (0.20, 0.20, 0.20, 1.0), 20.0),
    'part': phong((0.11, 0.11, 0.12, 1.0), (0.20, 0.20, 0.20, 1.0), 20.0),
    'connector': phong((0.90, 0.88, 0.80, 1.0), (0.15, 0.15, 0.15, 1.0), 10.0),
    'contact': phong((0.80, 0.80, 0.78, 1.0), (0.60, 0.60, 0.60, 1.0), 40.0, metallic=True),
    'window': phong((0.02, 0.02, 0.03, 1.0), (0.60, 0.60, 0.60, 1.0), 80.0),
}

Pieces = Dict[str, List[cq.Workplane]]


# ---------------------------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------------------------
def z_at(height: float) -> float:
    """Return the frame z of a height above the motor's bottom."""
    return height - SCAN_HEIGHT


def disc(x: float, y: float, radius: float, bottom: float, top: float) -> cq.Workplane:
    """Return a vertical cylinder between two heights above the motor's bottom."""
    return (cq.Workplane('XY', origin=(x, y, z_at(bottom))).circle(radius)
            .extrude(top - bottom))


def block(x: Tuple[float, float], y: Tuple[float, float], bottom: float,
          top: float) -> cq.Workplane:
    """Return a box between two x, two y and two heights above the motor's bottom."""
    return (cq.Workplane('XY', origin=((x[0] + x[1]) / 2, (y[0] + y[1]) / 2, z_at(bottom)))
            .rect(x[1] - x[0], y[1] - y[0]).extrude(top - bottom))


def revolve_path(start: Tuple[float, float], pieces: Sequence) -> cq.Workplane:
    """Revolve a closed (r, z) path of ('line', end) and ('arc', middle, end) about the z axis."""
    wp = cq.Workplane('XZ').moveTo(*start)
    for piece in pieces:
        if piece[0] == 'line':
            wp = wp.lineTo(*piece[1])
        else:
            wp = wp.threePointArc(piece[1], piece[2])
    return wp.close().revolve(360, (0, 0, 0), (0, 1, 0))


def quarter_arc(corner: Tuple[float, float], radius: float, first: Tuple[float, float],
                second: Tuple[float, float]) -> List:
    """Return the path pieces that round a right-angled profile corner by radius.

    first and second are the unit directions from the corner to the previous and the next point.
    """
    cx, cz = corner
    start = (cx + first[0] * radius, cz + first[1] * radius)
    end = (cx + second[0] * radius, cz + second[1] * radius)
    centre = (cx + (first[0] + second[0]) * radius, cz + (first[1] + second[1]) * radius)
    k = radius / math.sqrt(2)
    middle = (centre[0] - (first[0] + second[0]) * k, centre[1] - (first[1] + second[1]) * k)
    return [('line', start), ('arc', middle, end)]


def union_all(shapes: Sequence[cq.Workplane]) -> cq.Workplane:
    """Return the union of the shapes."""
    result = shapes[0]
    for shape in shapes[1:]:
        result = result.union(shape)
    return result


def turned(shape: cq.Workplane, degrees: float) -> cq.Workplane:
    """Return the shape turned about the z axis."""
    return shape.rotate((0, 0, 0), (0, 0, 1), degrees)


# ---------------------------------------------------------------------------------------------
# Base
# ---------------------------------------------------------------------------------------------
def side_radius() -> float:
    """Return the radius of the long sides: through (0, +-WIDTH/2), tangent to the motor end."""
    half, end = WIDTH / 2, MOTOR_END_RADIUS
    return (MOTOR_OFFSET ** 2 + half ** 2 - end ** 2) / (2 * (half - end))


def outline(bottom: float, top: float, inset: float = 0.0) -> cq.Workplane:
    """Return the base's outline, moved inwards by inset, between two heights.

    Two long arcs and the round end's arc meet the motor end's arc, and two ears go round the
    mounting holes at the round end. Moving every arc inwards by the same inset keeps the long
    arcs tangent to the motor end's arc.
    """
    radius = side_radius()
    centre = radius - WIDTH / 2
    distance = math.hypot(MOTOR_OFFSET, centre)
    tangent_x = -MOTOR_OFFSET * (radius - inset) / distance
    body = (disc(0, centre, radius - inset, bottom, top)
            .intersect(disc(0, -centre, radius - inset, bottom, top))
            .intersect(disc(ROUND_END - END_RADIUS, 0, END_RADIUS - inset, bottom, top))
            .intersect(block((tangent_x, tangent_x + 200), (-100, 100), bottom, top)))
    parts = [body, disc(-MOTOR_OFFSET, 0, MOTOR_END_RADIUS - inset, bottom, top)]
    parts += [disc(x, y, EAR_RADIUS - inset, bottom, top) for x, y in MOUNT_HOLES[:2]]
    return union_all(parts)


def base() -> cq.Workplane:
    """Return the base: a plate with a rounded top edge, its wall, bosses and the housing."""
    plate_bottom = BASE_TOP - PLATE_THICKNESS
    plate = outline(plate_bottom, BASE_TOP).faces('>Z').edges().fillet(TOP_EDGE_ROUND)
    wall = (outline(BASE_BOTTOM, plate_bottom + 0.01)
            .cut(outline(BASE_BOTTOM - 1, plate_bottom, WALL_THICKNESS)))
    bosses = [disc(x, y, BOSS_DIAMETER / 2, BASE_BOTTOM, plate_bottom + 0.01)
              for x, y in MOUNT_HOLES]
    housing = disc(0, 0, HOUSING_RADIUS, HOUSING_BOTTOM, plate_bottom + 0.01)
    position, _, _ = PCB_SCREWS
    standoffs = [disc(sx * position, sy * position, 2.0, PCB_BOTTOM + PCB_THICKNESS,
                      HOUSING_BOTTOM + 0.01) for sx in (-1, 1) for sy in (-1, 1)]
    solid = union_all([plate, wall, housing] + bosses + standoffs)
    holes = [disc(x, y, MOUNT_HOLE_DIAMETER / 2, BASE_BOTTOM - 1, BASE_TOP + 1)
             for x, y in MOUNT_HOLES]
    diameter, depth = SPOT_FACE
    holes += [disc(x, y, diameter / 2, BASE_TOP - depth, BASE_TOP + 1) for x, y in MOUNT_HOLES]
    return solid.cut(union_all(holes))


# ---------------------------------------------------------------------------------------------
# Motor, pulley and belt
# ---------------------------------------------------------------------------------------------
def motor() -> Pieces:
    """Return the motor's can with its boss (steel) and the two screws on the base (screw)."""
    radius = MOTOR_DIAMETER / 2
    bottom, top = z_at(MOTOR_CAN[0]), z_at(MOTOR_CAN[1])
    lower, upper = MOTOR_CAN_ROUND
    can = revolve_path((0, bottom), [
        *quarter_arc((radius, bottom), lower, (-1, 0), (0, 1)),
        *quarter_arc((radius, top), upper, (0, -1), (-1, 0)),
        ('line', (0, top))])
    diameter, boss_bottom = MOTOR_BOSS
    can = can.union(disc(0, 0, diameter / 2, boss_bottom, MOTOR_CAN[0] + 0.01))
    head, height = MOTOR_SCREW_HEAD
    screws = [disc(-MOTOR_OFFSET, side * MOTOR_SCREW_Y, head / 2, BASE_TOP, BASE_TOP + height)
              .faces('>Z').edges().chamfer(0.4) for side in (-1, 1)]
    return {'steel': [can.translate((-MOTOR_OFFSET, 0, 0))], 'screw': screws}


def pulley() -> cq.Workplane:
    """Return the motor's pulley: two flanges round the belt's seat, a stepped cap."""
    flange, seat, step, boss = PULLEY_RADII
    h = [z_at(height) for height in PULLEY_HEIGHTS]
    shape = revolve_path((0, h[0]), [
        ('line', (flange, h[0])), ('line', (flange, h[1])), ('line', (seat, h[1])),
        ('line', (seat, h[2])), ('line', (flange, h[2])), ('line', (flange, h[3])),
        ('line', (step, h[3])), ('line', (step, h[4])), ('line', (boss, h[4])),
        ('line', (boss, h[5])), ('line', (0, h[5]))])
    return shape.translate((-MOTOR_OFFSET, 0, 0))


def belt_loop(head_radius: float, pulley_radius: float, bottom: float,
              top: float) -> cq.Workplane:
    """Return the region inside a belt round two circles: the head's and the pulley's."""
    cosine = (head_radius - pulley_radius) / MOTOR_OFFSET
    nx, ny = -cosine, math.sqrt(1 - cosine * cosine)
    head_point = (head_radius * nx, head_radius * ny)
    pulley_point = (-MOTOR_OFFSET + pulley_radius * nx, pulley_radius * ny)
    return (cq.Workplane('XY', origin=(0, 0, z_at(bottom))).moveTo(*head_point)
            .lineTo(*pulley_point)
            .threePointArc((-MOTOR_OFFSET - pulley_radius, 0), (pulley_point[0], -pulley_point[1]))
            .lineTo(head_point[0], -head_point[1])
            .threePointArc((head_radius, 0), head_point).close().extrude(top - bottom))


def belt() -> cq.Workplane:
    """Return the flat belt from the head's pulley to the motor's pulley."""
    head, motor_side = BELT_OUTER
    bottom, top = BELT_HEIGHTS
    return (belt_loop(head, motor_side, bottom, top)
            .cut(belt_loop(head - BELT_THICKNESS, motor_side - BELT_THICKNESS, bottom - 1,
                           top + 1)))


# ---------------------------------------------------------------------------------------------
# Head
# ---------------------------------------------------------------------------------------------
def radial_prism(section: cq.Workplane, inner: float, outer: float,
                 azimuth: float) -> cq.Workplane:
    """Return a prism from radius inner to outer at the scan height, turned to the azimuth.

    section is a workplane on the YZ plane at x = inner with the prism's cross-section.
    """
    return turned(section.extrude(outer - inner), azimuth)


def yz_plane(x: float) -> cq.Workplane:
    """Return a YZ workplane at x, centred on the scan height."""
    return cq.Workplane('YZ', origin=(x, 0, 0))


def opening(kind: str, inner: float, outer: float, azimuth: float) -> cq.Workplane:
    """Return the prism of the lens's or the emitter's opening between two radii."""
    if kind == 'lens':
        width, height, corner = LENS_OPENING
        return turned(yz_plane(inner).rect(width, height).extrude(outer - inner)
                      .edges('|X').fillet(corner), azimuth)
    width, height = EMITTER_OPENING
    return radial_prism(yz_plane(inner).slot2D(width, height), inner, outer, azimuth)


def head() -> Pieces:
    """Return the rotating part: its body and the glass in its two openings."""
    radius = HEAD_DIAMETER / 2
    bottom, top = z_at(HEAD_BOTTOM), z_at(TOTAL_HEIGHT)
    pulley_bottom, flange_top, flange_radius = HEAD_PULLEY
    seat = BELT_OUTER[0] - BELT_THICKNESS
    body = revolve_path((0, z_at(pulley_bottom)), [
        ('line', (flange_radius, z_at(pulley_bottom))),
        ('line', (flange_radius, z_at(flange_top))), ('line', (seat, z_at(flange_top))),
        ('line', (seat, bottom)), ('line', (radius - HEAD_BOTTOM_CHAMFER, bottom)),
        ('line', (radius, bottom + HEAD_BOTTOM_CHAMFER)),
        *quarter_arc((radius, top), HEAD_TOP_ROUND, (0, -1), (-1, 0)),
        ('line', (0, top))])
    inner, outer = LOGO_RING
    body = body.cut(disc(0, 0, outer, TOTAL_HEIGHT - LOGO_DEPTH, TOTAL_HEIGHT + 1)
                    .cut(disc(0, 0, inner, TOTAL_HEIGHT - 2, TOTAL_HEIGHT + 2)))
    recess, depth = TOP_SCREW_RECESS
    for angle in TOP_SCREW_ANGLES:
        a = math.radians(WINDOW_AZIMUTH + angle)
        x, y = TOP_SCREW_RADIUS * math.cos(a), TOP_SCREW_RADIUS * math.sin(a)
        body = body.cut(disc(x, y, recess / 2, TOTAL_HEIGHT - depth, TOTAL_HEIGHT + 1))
    floor = radius - OPENING_DEPTH
    glass = []
    for kind, angle, (lens, barrel, proud) in (('lens', LENS_ANGLE, LENS),
                                               ('emitter', EMITTER_ANGLE, EMITTER_LENS)):
        azimuth = WINDOW_AZIMUTH + angle
        body = body.cut(opening(kind, floor, radius + 2, azimuth))
        # The lens's barrel stands on the opening's floor, the pane fills the floor around it
        body = body.union(radial_prism(yz_plane(floor).circle(barrel / 2).circle(lens / 2),
                                       floor, floor + PANE_THICKNESS + BARREL_HEIGHT, azimuth))
        glass.append(opening(kind, floor, floor + PANE_THICKNESS, azimuth).cut(
            radial_prism(yz_plane(floor - 1).circle(barrel / 2), floor - 1, floor + 2, azimuth)))
        glass.append(radial_prism(yz_plane(floor).circle(lens / 2), floor,
                                  floor + PANE_THICKNESS + proud, azimuth))
    return {'body': [body], 'window': glass}


# ---------------------------------------------------------------------------------------------
# Board and connectors
# ---------------------------------------------------------------------------------------------
def board() -> Pieces:
    """Return the circuit board with its parts and its four screws."""
    top = PCB_BOTTOM + PCB_THICKNESS
    pcb = (block(PCB_X, (-PCB_HALF_WIDTH, PCB_HALF_WIDTH), PCB_BOTTOM, top)
           .edges('|Z').fillet(PCB_CORNER))
    offset, diameter = PCB_HOLES
    pcb = pcb.cut(union_all([disc(0, side * offset, diameter / 2, PCB_BOTTOM - 1, top + 1)
                             for side in (-1, 1)]))
    x0, x1, y0, y1, height = SQUARE_PART
    x, y, round_diameter, round_height = ROUND_PART
    parts = [block((x0, x1), (y0, y1), PCB_BOTTOM - height, PCB_BOTTOM),
             disc(x, y, round_diameter / 2, PCB_BOTTOM - round_height, PCB_BOTTOM)
             .faces('<Z').edges().fillet(0.4)]
    position, head, head_height = PCB_SCREWS
    screws = [disc(sx * position, sy * position, head / 2, PCB_BOTTOM - head_height,
                   PCB_BOTTOM).faces('<Z').edges().chamfer(0.3)
              for sx in (-1, 1) for sy in (-1, 1)]
    return {'pcb': [pcb], 'part': parts, 'screw': screws}


def connectors() -> Pieces:
    """Return the vertical headers under the board: housings, pins and fixing tabs."""
    depth, height, wall = CONNECTOR_SIZE
    tab_width, tab_depth, tab_height = CONNECTOR_TAB
    bottom = PCB_BOTTOM - height
    roof = PCB_BOTTOM - 0.8  # the cavity's roof, where the pins come out
    housings, metal = [], []
    for y, pins in CONNECTORS:
        width = (pins - 1) * CONNECTOR_PITCH + 3.5
        xs = (CONNECTOR_X - depth / 2, CONNECTOR_X + depth / 2)
        ys = (y - width / 2, y + width / 2)
        cavity = block((xs[0] + wall, xs[1] - wall), (ys[0] + wall, ys[1] - wall), bottom - 1,
                       roof)
        housings.append(block(xs, ys, bottom, PCB_BOTTOM).cut(cavity))
        for k in range(pins):
            pin_y = y + (k - (pins - 1) / 2) * CONNECTOR_PITCH
            metal.append(block((CONNECTOR_X - PIN_SIZE / 2, CONNECTOR_X + PIN_SIZE / 2),
                               (pin_y - PIN_SIZE / 2, pin_y + PIN_SIZE / 2), bottom + 0.5,
                               roof))
        for tab_y in ((ys[0] - tab_width, ys[0]), (ys[1], ys[1] + tab_width)):
            metal.append(block((CONNECTOR_X - tab_depth / 2, CONNECTOR_X + tab_depth / 2), tab_y,
                               PCB_BOTTOM - tab_height, PCB_BOTTOM))
    return {'connector': housings, 'contact': metal}


# ---------------------------------------------------------------------------------------------
# Mesh
# ---------------------------------------------------------------------------------------------
FRAME = ("The head's rotation axis along z at x = y = 0, z = 0 in the scan plane (42 mm above "
         "the motor's bottom), the base's round end towards +x, the motor's axis at "
         "x = -45.5 mm, the window's middle towards -x.")


def compound(shapes: Sequence[cq.Workplane]) -> cq.Shape:
    """Return the solids of the workplanes as one compound."""
    return cq.Compound.makeCompound([solid for shape in shapes for solid in shape.solids().vals()])


def sub_part(name: str, pieces: Pieces) -> SubPart:
    """Return a named sub-part with one surface per material, in the materials' order."""
    return SubPart(name, {material: tessellate_faces(compound(pieces[material]),
                                                     LINEAR_DEFLECTION, ANGULAR_DEFLECTION)
                          for material in MATERIALS if material in pieces})


def build() -> List[Model]:
    """Return the scanner's mesh with its sub-parts, for an exploded view."""
    head_pieces = head()
    sub_parts = [
        sub_part('base', {'body': [base()]}),
        sub_part('pcb', board()),
        sub_part('connectors', connectors()),
        sub_part('motor', motor()),
        sub_part('pulley', {'body': [pulley()]}),
        sub_part('belt', {'rubber': [belt()]}),
        sub_part('head', {'body': head_pieces['body']}),
        sub_part('window', {'window': head_pieces['window']}),
    ]
    return [Model('rplidar_a1', TITLE, FRAME, MATERIALS, sub_parts)]
