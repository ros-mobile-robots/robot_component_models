# SPDX-FileCopyrightText: 2026 ROS Mobile Robots
# SPDX-License-Identifier: BSD-3-Clause
"""Generate DiffBot's swivel caster in three meshes: mounting base, swivel fork and wheel.

A pressed-steel swivel caster with a 30 mm plastic wheel, as on DiffBot and in the robot kit
DiffBot is built on (Joy-IT Robot Car Kit 2WD, robot05): a plate with four corner lugs on four
brass hex standoffs below the chassis, held by Phillips screws. The plate is pressed into a
raceway: a trough that bulges below it, an arch over a row of balls, and a centre with the
kingpin's flat head on top. The fork's top is a shallow cup that opens towards the wheel, with
the lower ball track, a stepped race cap and the kingpin's flat lower head inside. On the axle
sides the cup's wall runs on into the arms: curved at the cup, flat lower down, each narrowing
to a rounded tip around the axle. The axle has a pan head on one side and a domed head with a
washer on the other. Sizes come from DiffBot's
caster_wheel.yaml (wheel radius, joint offsets), the shapes from photos and video frames of
DiffBot's caster and from Joy-IT's product photos. Sources in provenance/swivel_caster_30mm.md.

Frames (positions in mm, the files are in metres, z up):
- swivel_caster_30mm_base: the fixed part, with standoffs, plate, bearing and rivet. z = 0 is
  the swivel bearing's lower face. The standoffs end at z = +15 mm. The swivel axis is at
  x = -1 mm (DiffBot's hub joint is 1 mm behind its base joint).
- swivel_caster_30mm_wheel_support: the swivelling fork, its swivel axis at x = y = 0, its top
  at z = 0, the wheel axle 16 mm behind the axis and 20 mm below the top.
- swivel_caster_30mm_wheel: the wheel, centred on its axle, the axle along y.
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

TITLE = 'Swivel caster with a 30 mm wheel (DiffBot)'

# ---------------------------------------------------------------------------------------------
# Parameters, in mm. The joint and visual offsets come from DiffBot's caster_wheel.yaml and caster
# macro, the shapes from video frames of DiffBot's caster and Joy-IT's photos
# (provenance/swivel_caster_30mm.md).
# ---------------------------------------------------------------------------------------------
SWIVEL_AXIS_X = -1.0          # in the base's frame: hub joint dx -0.077 minus base dx -0.076
STANDOFF_TOP_Z = 15.0         # the standoffs' top: on DiffBot 0.5 mm below the chassis plate's top
STANDOFF_LENGTH = 10.0        # brass hex standoffs between chassis and plate (video frames)
STANDOFF_ACROSS_FLATS = 5.0   # hexagonal M3 standoff
STANDOFF_CHAMFER = 0.3        # the hexagon's end edges (choice)
# The M3 internal thread at the chassis end, shown as a smooth blind bore of the tapping
# diameter, without a thread profile (choice)
STANDOFF_THREAD = (2.5, 3.0)  # diameter, depth
PLATE_SIZE = (32.0, 38.0)     # x, y: the plate's footprint
PLATE_THICKNESS = 1.3         # listing of the same 38 x 32 mm plate (Electrokit): 1.3 mm
PLATE_CORNER_RADIUS = 3.0
PLATE_HOLES = (12.0, 15.0)    # hole centres at (+-x, +-y) from the swivel axis: 24 x 30 mm
PLATE_HOLE_DIAMETER = 3.4
PLATE_SIDE_CUT = 3.0          # the plate's sides curve inwards between the corner lugs (photos)
# The plate's pressed raceway, a revolved sheet: (r, z of the sheet's middle above the flat
# plate's middle), from the flat plate inwards. A trough that bulges 3.35 mm below the plate
# (Tente's dome Ø 26.5 mm, frames 134022 t107-110 s), 0.35 mm above the fork's cup, so the
# balls only show through that slit. Then an arch over the balls, an inner groove
# and the centre under the kingpin's head (Tente's section, Joy-IT's top photo)
PLATE_RACE = ((13.4, 0.0), (12.5, -0.6), (11.6, -2.7), (10.7, -3.35), (9.9, -3.05),
              (9.3, -1.9), (8.8, -1.35), (8.3, -1.1), (7.5, -1.0), (6.7, -1.1), (6.2, -1.4),
              (5.6, -1.9), (4.8, -1.45), (4.2, -0.9), (3.6, -0.65), (0.0, -0.65))
BALLS = 12                    # one row (Tente: single ball race, Adafruit's side photo)
BALL_DIAMETER = 3.2           # choice (Tente's section: about 3.5 to 4)
BALL_CIRCLE_RADIUS = 7.5      # Tente's section: about Ø 15 mm
BALL_Z = 0.95                 # the balls' centres, between the plate's arch and the fork's track
SCREW_HEAD = (5.6, 1.7)       # M3 Phillips pan heads under the plate: diameter, height
SCREW_CROSS = (3.0, 0.6, 0.6)  # cross recess: length, width, depth
KINGPIN_DIAMETER = 4.0
KINGPIN_HEAD = (7.5, 2.0, 0.4)  # flat head on the plate's centre: diameter, height, edge round
# Race cap under the cup's floor, inside the cup, concentric pressed steps (frame 140705 t33 s):
# (radius, depth below the cup floor's underside) from the rim inwards to the flat rivet head
RACE_CAP = ((5.6, 0.1), (5.6, 0.5), (5.2, 0.9), (4.9, 0.9), (4.8, 1.3), (4.7, 1.3))
RIVET_HEAD_BOTTOM_RADIUS = 4.6  # the kingpin's flat lower head (frame 140705 t33 s)
RIVET_BOTTOM_Z = -3.3         # the flat lower head's bottom, the base's lowest point

# The fork, bent from one sheet (Makerfabs' listing of the family: 1.2 mm cold-rolled steel)
FORK_THICKNESS = 1.2
# Its top is a shallow cup that opens towards the wheel, rim Ø 24.5 mm (frames 140705 t33 s,
# 134022 t107-110 s, Adafruit): (r, z of the sheet's middle) from the axis out. The floor's
# groove is the lower ball track (a raised ring seen from inside), then the bend into the
# wall, down to the rim
FORK_CUP = ((0.0, -0.65), (3.0, -0.65), (5.7, -0.65), (6.3, -1.0), (6.9, -1.35), (7.5, -1.45),
            (8.1, -1.35), (8.7, -1.0), (9.3, -0.65), (10.0, -0.65), (10.65, -0.65),
            (11.15, -0.79), (11.52, -1.15), (11.65, -1.65), (11.65, -2.6), (11.65, -3.55))
KINGPIN_HOLE = 2.2            # radius of the hole in the cup's floor
# The arms: on the axle sides the cup's wall runs on into the arms, curved at the cup and flat
# and parallel to the wheel lower down. Seen along the axle each arm narrows like a boot to a
# rounded tip around the axle: a diagonal front edge, a steeper rear edge (frames 134022
# t107-110 s, Adafruit, Bits & Parts)
FORK_ARM_ANGLES = (48.0, 132.0)  # where each arm leaves the cup's wall, deg from +x (front)
FORK_ARM_FLAT_Z = -9.0        # below this the arms are flat
FORK_ARM_BEND_STEPS = 20      # sections between the cup's wall and the flat arm
FORK_INNER_HALF_WIDTH = 8.6   # 17.2 mm between the flat arms for the 15.6 mm wheel
FORK_TIP_RADIUS = 4.75        # the arm's rounded tip around the axle (about 9.5 mm wide)
FORK_FRONT_FILLET = 3.0       # where the arm's front edge curves into the cup's rim (Adafruit)
AXLE_X, AXLE_Z = -16.0, -20.0  # caster_wheel.yaml: wheel dx -0.016, dz -0.04 (fork offset 0.02)
AXLE_DIAMETER = 3.0
AXLE_WASHER = (3.75, 0.5)     # on the domed side: radius, thickness
AXLE_HEAD = (3.25, 1.4)       # domed head: radius, height (frames 134022 t107-110 s)
AXLE_PAN_HEAD = (5.8, 1.6)    # Phillips pan head on the other side (frame 131939 t107.5 s)

WHEEL_RADIUS = 15.0           # caster_wheel.yaml: radius 0.015
WHEEL_HALF_WIDTH = 7.8        # hub width 15.6 mm, 0.8 mm clear of each fork arm
TREAD_HALF_WIDTH = 6.0
TREAD_ROUND = 3.0             # crowned tread (video frames)
WHEEL_DISH = (5.5, 10.5, 5.0)  # recessed side: hub radius, dish outer radius, dish face |y|
WHEEL_BORE = AXLE_DIAMETER / 2 + 0.1


LINEAR_DEFLECTION = 0.03
ANGULAR_DEFLECTION = 0.3

# Materials, as on DiffBot's caster (video frames): zinc-plated steel plate, raceway, fork and
# rivets, brass standoffs, a matte white plastic wheel. In COLLADA (Gazebo, RViz) a darker
# diffuse colour with strong, tight highlights reads as metal, a light diffuse colour with weak,
# broad highlights as plastic. In glTF the metals are metallic, with a low roughness from the
# shininess, the wheel is not metallic and rough.
STEEL = phong((0.56, 0.58, 0.62, 1.0), (0.90, 0.91, 0.95, 1.0), 30.0, metallic=True)
MATERIALS = {
    'steel': STEEL,
    'brass': phong((0.72, 0.53, 0.20, 1.0), (0.95, 0.85, 0.55, 1.0), 30.0, metallic=True),
    'axle': phong((0.62, 0.63, 0.66, 1.0), (0.92, 0.92, 0.95, 1.0), 40.0, metallic=True),
    'wheel': phong((0.93, 0.93, 0.90, 1.0), (0.08, 0.08, 0.08, 1.0), 6.0),
}


# ---------------------------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------------------------
def round_corner(corner: Tuple[float, float], radius: float,
                 first: Tuple[float, float], second: Tuple[float, float]) -> List:
    """Return the path pieces that round a profile corner (convex or concave) by radius.

    first and second are the unit directions from the corner to the previous and the next
    point of the path, along two perpendicular profile edges.
    The pieces are ('line', end) and ('arc', middle, end) for revolve_path.
    """
    cx, cz = corner
    start = (cx + first[0] * radius, cz + first[1] * radius)
    end = (cx + second[0] * radius, cz + second[1] * radius)
    centre = (cx + (first[0] + second[0]) * radius, cz + (first[1] + second[1]) * radius)
    k = radius / math.sqrt(2)
    middle = (centre[0] - (first[0] + second[0]) * k, centre[1] - (first[1] + second[1]) * k)
    return [('line', start), ('arc', middle, end)]


def revolve_path(start: Tuple[float, float], pieces: Sequence, plane: str = 'XZ') -> cq.Workplane:
    """Revolve a closed path of lines and arcs about the plane's second axis.

    On 'XZ' the path is (r, z) and the axis is z. On 'XY' the path is (r, y) and the axis is y.
    """
    wp = cq.Workplane(plane).moveTo(*start)
    for piece in pieces:
        if piece[0] == 'line':
            wp = wp.lineTo(*piece[1])
        else:
            wp = wp.threePointArc(piece[1], piece[2])
    return wp.close().revolve(360, (0, 0, 0), (0, 1, 0))


def union_all(shapes: Sequence[cq.Workplane]) -> cq.Workplane:
    """Return the union of the shapes."""
    result = shapes[0]
    for shape in shapes[1:]:
        result = result.union(shape)
    return result


def sheet(nodes: Sequence[Tuple[float, float]], thickness: float) -> cq.Workplane:
    """Revolve a sheet of the given thickness about z, its middle through the (r, z) nodes.

    Each node moves by half the thickness along the curve's normal (from its neighbours) to
    both sides. The two offset curves are splines with the curve's direction at every node,
    so straight runs stay straight, joined by straight ends.
    """
    upper, lower, tangents = [], [], []
    for k, (r, z) in enumerate(nodes):
        r0, z0 = nodes[max(k - 1, 0)]
        r1, z1 = nodes[min(k + 1, len(nodes) - 1)]
        length = math.hypot(r1 - r0, z1 - z0)
        tr, tz = (r1 - r0) / length, (z1 - z0) / length
        nr, nz = -tz, tr
        if nz < 0 or (nz == 0 and nr < 0):
            nr, nz = -nr, -nz
        tangents.append((tr, tz))
        upper.append((r + nr * thickness / 2, z + nz * thickness / 2))
        lower.append((r - nr * thickness / 2, z - nz * thickness / 2))
    backwards = [(-tr, -tz) for tr, tz in reversed(tangents)]
    wp = (cq.Workplane('XZ').moveTo(*upper[0])
          .spline(upper[1:], tangents=tangents, includeCurrent=True)
          .lineTo(*lower[-1])
          .spline(lower[-2::-1], tangents=backwards, includeCurrent=True).close())
    return wp.revolve(360, (0, 0, 0), (0, 1, 0))


def ball(x: float) -> cq.Workplane:
    """Return one bearing ball at x on the y = 0 plane, at the balls' height."""
    return cq.Workplane('XY').sphere(BALL_DIAMETER / 2).translate((x, 0, BALL_Z))


def volume(shape: cq.Workplane) -> float:
    """Return the total volume of the solids in shape (0 if there are none)."""
    return sum(solid.Volume() for solid in shape.solids().vals())


def disc(diameter: float, z0: float, height: float,
         x: float = 0.0, y: float = 0.0) -> cq.Workplane:
    """Return a vertical cylinder from z0 up by height."""
    return cq.Workplane('XY', origin=(x, y, z0)).circle(diameter / 2).extrude(height)


def dome(diameter: float, z_flat: float, height: float, up: bool, x: float = 0.0) -> cq.Workplane:
    """Return a rivet head: a flat-bottomed dome of the given height above (or below) z_flat."""
    r = diameter / 2
    sign = 1 if up else -1
    # A spherical cap: sphere radius from the cap's base radius and height, centre on the axis
    sphere = (r * r + height * height) / (2 * height)
    centre = z_flat + sign * (height - sphere)
    mid = (r / 2, centre + sign * math.sqrt(sphere * sphere - r * r / 4))
    body = revolve_path((0, z_flat), [('line', (r, z_flat)),
                                      ('arc', mid, (0, z_flat + sign * height))])
    return body.translate((x, 0, 0))


def phillips_screw(x: float, y: float, z_flat: float, diameter: float = SCREW_HEAD[0],
                   height: float = SCREW_HEAD[1]) -> cq.Workplane:
    """Return a pan head screw head below z_flat with a cross recess."""
    head = dome(diameter, z_flat, height, up=False)
    length, width, depth = SCREW_CROSS
    tip = z_flat - height
    cross = (cq.Workplane('XY', origin=(0, 0, tip - 0.01)).rect(length, width).extrude(depth)
             .union(cq.Workplane('XY', origin=(0, 0, tip - 0.01)).rect(width, length)
                    .extrude(depth)))
    return head.cut(cross).translate((x, y, 0))


def make_base() -> Dict[str, cq.Workplane]:
    """Return the fixed part per material: steel plate, raceway, balls, kingpin, screws, brass.

    The plate's raceway is a revolved sheet in a round hole of the flat plate. The kingpin's
    flat head stands on the raceway's centre, its shaft runs down through the fork's cup to
    the stepped race cap and the flat lower head inside the cup.
    """
    ax = SWIVEL_AXIS_X
    plate_bottom = STANDOFF_TOP_Z - STANDOFF_LENGTH - PLATE_THICKNESS
    plate_top = plate_bottom + PLATE_THICKNESS
    plate_middle = plate_bottom + PLATE_THICKNESS / 2
    holes = [(ax + sx * PLATE_HOLES[0], sy * PLATE_HOLES[1]) for sx in (-1, 1) for sy in (-1, 1)]
    plate = (cq.Workplane('XY', origin=(ax, 0, plate_bottom))
             .rect(*PLATE_SIZE).extrude(PLATE_THICKNESS)
             .edges('|Z').fillet(PLATE_CORNER_RADIUS))
    # Concave sides: a circle cut on each side, as deep as PLATE_SIDE_CUT, between the lugs
    half_x, half_y = PLATE_SIZE[0] / 2, PLATE_SIZE[1] / 2
    for (cx, cy, radius) in ((0, half_y, 14.0), (0, -half_y, 14.0),
                             (half_x, 0, 16.0), (-half_x, 0, 16.0)):
        inward = PLATE_SIDE_CUT - radius
        centre = (ax + cx + math.copysign(-inward, cx) if cx else ax,
                  cy + math.copysign(-inward, cy) if cy else 0.0)
        plate = plate.cut(cq.Workplane('XY', origin=(*centre, plate_bottom - 1))
                          .circle(radius).extrude(PLATE_THICKNESS + 2))
    race_r = PLATE_RACE[0][0]
    plate = plate.cut(cq.Workplane('XY', origin=(0, 0, plate_bottom - 1))
                      .pushPoints(holes).circle(PLATE_HOLE_DIAMETER / 2)
                      .extrude(PLATE_THICKNESS + 2))
    plate = plate.cut(disc(2 * race_r, plate_bottom - 1, PLATE_THICKNESS + 2, x=ax))
    race = sheet([(r, plate_middle + dz) for r, dz in PLATE_RACE], PLATE_THICKNESS)
    # The balls between the plate's arch and the fork's track: the whole ball clear of the arch
    if volume(race.intersect(ball(BALL_CIRCLE_RADIUS))) > 0:
        raise ValueError("the balls must stay clear of the plate's raceway")
    steel = plate.union(race.translate((ax, 0, 0)))
    balls = union_all([cq.Workplane('XY').sphere(BALL_DIAMETER / 2)
                       .translate((ax + BALL_CIRCLE_RADIUS * math.cos(2 * math.pi * k / BALLS),
                                   BALL_CIRCLE_RADIUS * math.sin(2 * math.pi * k / BALLS),
                                   BALL_Z))
                       for k in range(BALLS)])
    steel = steel.union(balls)
    # Kingpin: the flat head on the raceway's centre, the shaft down through the fork's cup
    centre_top = plate_middle + PLATE_RACE[-1][1] + PLATE_THICKNESS / 2
    head_d, head_h, head_round = KINGPIN_HEAD
    head = revolve_path((0, centre_top - 0.01), [
        ('line', (head_d / 2, centre_top - 0.01)),
        *round_corner((head_d / 2, centre_top + head_h), head_round, (0, -1), (-1, 0)),
        ('line', (0, centre_top + head_h))]).translate((ax, 0, 0))
    shaft = disc(KINGPIN_DIAMETER, RIVET_BOTTOM_Z, centre_top - RIVET_BOTTOM_Z, x=ax)
    steel = steel.union(head).union(shaft)
    # The race cap and the flat lower head hang below the cup floor's underside, inside the cup
    floor_under = FORK_CUP[0][1] - FORK_THICKNESS / 2
    cap_points = [(r, floor_under - depth) for r, depth in RACE_CAP]
    head_r = RIVET_HEAD_BOTTOM_RADIUS
    head_top = cap_points[-1][1]
    head_h = head_top - RIVET_BOTTOM_Z
    if head_h < 0.2:
        raise ValueError(f'race cap reaches {head_top} mm: no room for the rivet head above '
                         f'{RIVET_BOTTOM_Z} mm')
    groove_start = next(r for r, z in FORK_CUP if z < FORK_CUP[0][1])
    if max(r for r, _ in RACE_CAP) >= groove_start:
        raise ValueError("the race cap must stay inside the cup floor's ball track")
    cap = revolve_path((0, cap_points[0][1]), [
        *[('line', point) for point in cap_points],
        ('line', (head_r, head_top)), ('line', (head_r, RIVET_BOTTOM_Z + head_h / 2)),
        ('arc', (head_r - head_h / 4, RIVET_BOTTOM_Z + head_h / 10),
         (head_r - head_h / 2, RIVET_BOTTOM_Z)),
        ('line', (0, RIVET_BOTTOM_Z))]).translate((ax, 0, 0))
    steel = steel.union(cap)
    for x, y in holes:
        steel = steel.union(phillips_screw(x, y, plate_bottom))
    if len(steel.solids().vals()) != BALLS + 1:
        raise ValueError('the balls must stay separate from the rest of the steel and each other')
    brass = union_all([standoff(x, y, plate_top) for x, y in holes])
    return {'steel': steel, 'brass': brass}


def standoff(x: float, y: float, z0: float) -> cq.Workplane:
    """Return a hexagonal brass standoff from z0 up to the chassis, chamfered, with a bore."""
    body = (cq.Workplane('XY', origin=(x, y, z0))
            .polygon(6, STANDOFF_ACROSS_FLATS / math.cos(math.pi / 6))
            .extrude(STANDOFF_TOP_Z - z0))
    body = body.faces('>Z').edges().chamfer(STANDOFF_CHAMFER)
    body = body.faces('<Z').edges().chamfer(STANDOFF_CHAMFER)
    diameter, depth = STANDOFF_THREAD
    return body.cut(cq.Workplane('XY', origin=(x, y, STANDOFF_TOP_Z - depth))
                    .circle(diameter / 2).extrude(depth + 0.01))


def tangent_point(point: Tuple[float, float], centre: Tuple[float, float], radius: float,
                  side: int) -> Tuple[float, float]:
    """Return where a line from point touches the circle (side +1 or -1 picks the tangent)."""
    dx, dz = point[0] - centre[0], point[1] - centre[1]
    distance = math.hypot(dx, dz)
    angle = math.atan2(dz, dx) + side * math.acos(radius / distance)
    return (centre[0] + radius * math.cos(angle), centre[1] + radius * math.sin(angle))


def arm_outline() -> cq.Workplane:
    """Return the arms' outline seen along the axle, as a prism along y, with room for the cup.

    Above the cup's rim everything stays. Below it each arm runs from the cup's rim, between
    the arm's ends on the wall, down to a rounded tip around the axle: a diagonal front edge
    that curves into the rim, and the rear edge.
    """
    rim_z = FORK_CUP[-1][1]
    outer = FORK_CUP[-1][0] + FORK_THICKNESS / 2
    front = (outer * math.cos(math.radians(FORK_ARM_ANGLES[0])), rim_z)
    rear = (outer * math.cos(math.radians(FORK_ARM_ANGLES[1])), rim_z)
    axle = (AXLE_X, AXLE_Z)
    front_tip = tangent_point(front, axle, FORK_TIP_RADIUS, -1)
    rear_tip = tangent_point(rear, axle, FORK_TIP_RADIUS, 1)
    bottom = (AXLE_X, AXLE_Z - FORK_TIP_RADIUS)
    edge = (front_tip[0] - front[0], front_tip[1] - front[1])
    length = math.hypot(*edge)
    fillet = FORK_FRONT_FILLET
    # The front edge's top corner, rounded: a circle tangent to the rim line and the edge
    half = math.acos(-edge[0] / length) / 2
    reach = fillet / math.tan(half)
    start = (front[0] + reach, rim_z)
    end = (front[0] + edge[0] / length * reach, rim_z + edge[1] / length * reach)
    bisector = math.atan2(edge[1] / length, edge[0] / length + 1.0)
    distance = fillet / math.sin(half)
    middle = (front[0] + math.cos(bisector) * (distance - fillet),
              rim_z + math.sin(bisector) * (distance - fillet))
    span = 2 * outer + 4
    return (cq.Workplane('XZ', origin=(0, span, 0))
            .moveTo(span, 2.0).lineTo(span, rim_z).lineTo(*start)
            .threePointArc(middle, end).lineTo(*front_tip)
            .threePointArc(bottom, rear_tip).lineTo(*rear)
            .lineTo(-span, rim_z).lineTo(-span, 2.0).close().extrude(2 * span))


def fork_arm() -> cq.Workplane:
    """Return the arm on the +y side, before its outline: the curved root and the flat sheet.

    A ruled loft through closed sections of four edges each (a spline along each face, joined
    by straight ends), so the faces are smooth across the arm. The sections step down evenly
    from a sector of the cup's wall at the rim to a flat strip at FORK_ARM_FLAT_Z, and blend
    across with a half cosine: the arm leaves the wall and meets the flat strip running
    straight down, and the bends between neighbouring sections stay small. A last strip at the
    bottom continues the flat sheet past the axle.
    """
    # The loft starts inside the cup's straight wall, 0.15 mm inside its faces, so the union
    # with the cup has no faces in common and no grazing intersections at the rim
    rim_z = FORK_CUP[-1][1] + 0.6
    middle = FORK_CUP[-1][0]
    r_in, r_out = middle - FORK_THICKNESS / 2 + 0.15, middle + FORK_THICKNESS / 2 - 0.15
    a0, a1 = (math.radians(a) for a in FORK_ARM_ANGLES)
    y_in, y_out = FORK_INNER_HALF_WIDTH, FORK_INNER_HALF_WIDTH + FORK_THICKNESS
    x_front = r_out * math.cos(a0)
    x_rear = AXLE_X - FORK_TIP_RADIUS - 2       # past the axle's tip: the outline trims it
    count = 16

    def section(outer: List[Tuple[float, float]], inner: List[Tuple[float, float]],
                z: float) -> cq.Wire:
        """Return a closed section at z: a spline along each face, joined by straight ends."""
        def point(p: Tuple[float, float]) -> cq.Vector:
            return cq.Vector(p[0], p[1], z)

        return cq.Wire.assembleEdges([
            cq.Edge.makeSpline([point(p) for p in outer]),
            cq.Edge.makeLine(point(outer[-1]), point(inner[0])),
            cq.Edge.makeSpline([point(p) for p in inner]),
            cq.Edge.makeLine(point(inner[-1]), point(outer[0]))])

    def blend(first: List[Tuple[float, float]], second: List[Tuple[float, float]],
              share: float) -> List[Tuple[float, float]]:
        return [(p[0] + (q[0] - p[0]) * share, p[1] + (q[1] - p[1]) * share)
                for p, q in zip(first, second)]

    angles = [a0 + (a1 - a0) * k / count for k in range(count + 1)]
    wall_outer = [(r_out * math.cos(a), r_out * math.sin(a)) for a in angles]
    wall_inner = [(r_in * math.cos(a), r_in * math.sin(a)) for a in reversed(angles)]
    xs = [x_front + (x_rear - x_front) * k / count for k in range(count + 1)]
    flat_outer = [(x, y_out) for x in xs]
    flat_inner = [(x, y_in) for x in reversed(xs)]
    sections = []
    for k in range(FORK_ARM_BEND_STEPS + 1):
        step = k / FORK_ARM_BEND_STEPS
        share = (1 - math.cos(math.pi * step)) / 2
        z = rim_z + (FORK_ARM_FLAT_Z - rim_z) * step
        sections.append(section(blend(wall_outer, flat_outer, share),
                                blend(wall_inner, flat_inner, share), z))
    sections.append(section(flat_outer, flat_inner, AXLE_Z - FORK_TIP_RADIUS - 1))
    return cq.Workplane('XY').add(cq.Solid.makeLoft(sections, True))


def make_fork() -> Dict[str, cq.Workplane]:
    """Return the swivelling fork per material: the steel cup with its arms, the axle rivet.

    The cup is a revolved sheet. On each axle side an arm runs on from its wall, trimmed by
    the outline seen along the axle. The axle has a domed head with a washer on the -y side and
    a Phillips pan head on the +y side.
    """
    if FORK_INNER_HALF_WIDTH >= FORK_CUP[-1][0] - FORK_THICKNESS / 2:
        raise ValueError("the flat arms must lie inside the cup's wall")
    cup = sheet(FORK_CUP, FORK_THICKNESS)
    if volume(cup.intersect(ball(BALL_CIRCLE_RADIUS))) > 0:
        raise ValueError("the balls must stay clear of the cup's ball track")
    if AXLE_Z - FORK_TIP_RADIUS - 1 >= FORK_ARM_FLAT_Z:
        raise ValueError("the arm's flat part must lie above its bottom")
    arm = fork_arm()
    arms = arm.union(arm.mirror('XZ')).intersect(arm_outline())
    steel = cup.union(arms)
    steel = steel.cut(disc(2 * KINGPIN_HOLE, FORK_CUP[0][1] - FORK_THICKNESS,
                           2 * FORK_THICKNESS))
    axle_hole = (cq.Workplane('XZ', origin=(0, 20, 0)).center(AXLE_X, AXLE_Z)
                 .circle(AXLE_DIAMETER / 2 + 0.1).extrude(40))
    steel = steel.cut(axle_hole)
    outer = FORK_INNER_HALF_WIDTH + FORK_THICKNESS
    washer_r, washer_t = AXLE_WASHER
    head_r, head_h = AXLE_HEAD
    pan_d, pan_h = AXLE_PAN_HEAD
    parts = [cq.Workplane('XZ', origin=(0, outer + washer_t + 0.01, 0)).center(AXLE_X, AXLE_Z)
             .circle(AXLE_DIAMETER / 2).extrude(2 * outer + washer_t + 0.02),
             cq.Workplane('XZ', origin=(0, outer + washer_t, 0)).center(AXLE_X, AXLE_Z)
             .circle(washer_r).extrude(washer_t)]
    dome_head = dome(2 * head_r, 0.0, head_h, up=True).rotate((0, 0, 0), (1, 0, 0), -90)
    parts.append(dome_head.translate((AXLE_X, outer + washer_t, AXLE_Z)))
    pan = phillips_screw(0.0, 0.0, 0.0, pan_d, pan_h)
    parts.append(pan.rotate((0, 0, 0), (1, 0, 0), -90).translate((AXLE_X, -outer, AXLE_Z)))
    # Mirrored: the domed head on the -y side, as in the frames 134022 t107-110 s
    return {'steel': steel, 'axle': union_all(parts).mirror('XZ')}


def make_wheel() -> Dict[str, cq.Workplane]:
    """Return the wheel: crowned tread, dished sides and a hub, about the y axis."""
    hub_r, dish_r, dish_y = WHEEL_DISH
    side_r = WHEEL_RADIUS - TREAD_ROUND
    pieces = [('line', (hub_r, -WHEEL_HALF_WIDTH)), ('line', (hub_r, -dish_y)),
              ('line', (dish_r, -dish_y)), ('line', (dish_r + 1.0, -TREAD_HALF_WIDTH)),
              *round_corner((WHEEL_RADIUS, -TREAD_HALF_WIDTH), TREAD_ROUND, (-1, 0), (0, 1)),
              *round_corner((WHEEL_RADIUS, TREAD_HALF_WIDTH), TREAD_ROUND, (0, -1), (-1, 0)),
              ('line', (dish_r + 1.0, TREAD_HALF_WIDTH)), ('line', (dish_r, dish_y)),
              ('line', (hub_r, dish_y)), ('line', (hub_r, WHEEL_HALF_WIDTH)),
              ('line', (WHEEL_BORE, WHEEL_HALF_WIDTH))]
    if side_r <= dish_r + 1.0:
        raise ValueError("the wheel's dish reaches into the tread's rounds")
    return {'wheel': revolve_path((WHEEL_BORE, -WHEEL_HALF_WIDTH), pieces, plane='XY')}


# ---------------------------------------------------------------------------------------------
# Meshes
# ---------------------------------------------------------------------------------------------
FRAMES = {
    'swivel_caster_30mm_base': ("z = 0 at the swivel bearing's lower face, the standoffs' tops "
                                'at z = +15 mm (the chassis), the swivel axis at x = -1 mm, '
                                'y = 0.'),
    'swivel_caster_30mm_wheel_support': ("The swivel axis at x = y = 0, the fork's top at z = 0, "
                                         'the wheel axle at x = -16 mm, z = -20 mm along y.'),
    'swivel_caster_30mm_wheel': "Centred on the wheel's axle, the axle along y. Radius 15 mm.",
}
SUB_PARTS = {
    'swivel_caster_30mm_base': {'plate': 'steel', 'standoffs': 'brass'},
    'swivel_caster_30mm_wheel_support': {'fork': 'steel', 'axle': 'axle'},
    'swivel_caster_30mm_wheel': {'wheel': 'wheel'},
}


def caster_model(name: str, solids: Dict[str, cq.Workplane]) -> Model:
    """Return one of the caster's meshes, one sub-part per material."""
    sub_parts = [SubPart(part, {material: tessellate_faces(solids[material], LINEAR_DEFLECTION,
                                                           ANGULAR_DEFLECTION)})
                 for part, material in SUB_PARTS[name].items()]
    return Model(name, f'{TITLE}: {name}', FRAMES[name], MATERIALS, sub_parts)


def build() -> List[Model]:
    """Return the caster's three meshes."""
    return [caster_model('swivel_caster_30mm_base', make_base()),
            caster_model('swivel_caster_30mm_wheel_support', make_fork()),
            caster_model('swivel_caster_30mm_wheel', make_wheel())]
