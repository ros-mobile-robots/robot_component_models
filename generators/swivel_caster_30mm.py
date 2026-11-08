# SPDX-FileCopyrightText: 2026 ROS Mobile Robots
# SPDX-License-Identifier: BSD-3-Clause
"""Generate DiffBot's swivel caster in three meshes: mounting base, swivel fork and wheel.

A pressed-steel swivel caster with a 30 mm plastic wheel, as on DiffBot and in the robot kit
DiffBot is built on (Joy-IT Robot Car Kit 2WD, robot05): a plate with four corner lugs on four
brass hex standoffs below the chassis, held by Phillips screws. A pressed bearing with a stepped
race cap and a large flat rivet head. A fork bent from one sheet, with rounded bends and curved
arms. The wheel on an axle rivet with domed heads and washers. Sizes come from DiffBot's
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
# Parameters, in mm. Frame values come from caster_wheel.yaml and the reference meshes' bounds,
# the shapes from video frames of DiffBot's caster and Joy-IT's photos
# (provenance/swivel_caster_30mm.md).
# ---------------------------------------------------------------------------------------------
SWIVEL_AXIS_X = -1.0          # in the base's frame: hub joint dx -0.077 minus base dx -0.076
STANDOFF_TOP_Z = 15.0         # frame alignment: the reference base's top
STANDOFF_LENGTH = 10.0        # brass hex standoffs between chassis and plate (video frames)
STANDOFF_ACROSS_FLATS = 5.0   # hexagonal M3 standoff
STANDOFF_CHAMFER = 0.3        # the hexagon's end edges (choice)
# The M3 internal thread at the chassis end, shown as a smooth blind bore of the tapping
# diameter, without a thread profile (choice)
STANDOFF_THREAD = (2.5, 3.0)  # diameter, depth
PLATE_SIZE = (32.0, 38.0)     # x, y: the reference base's footprint
PLATE_THICKNESS = 1.5
PLATE_CORNER_RADIUS = 3.0
PLATE_HOLES = (12.0, 15.0)    # hole centres at (+-x, +-y) from the swivel axis
PLATE_HOLE_DIAMETER = 3.4
PLATE_SIDE_CUT = 3.0          # the plate's sides curve inwards between the corner lugs (photos)
RACE_RIDGE = (5.5, 13.0, 1.6)  # pressed raceway dome on the plate: inner r, outer r, height
RACE_RING = (8.6, 12.6)       # the bearing ring between plate and fork: inner r, outer r
SCREW_HEAD = (5.6, 1.7)       # M3 Phillips pan heads under the plate: diameter, height
SCREW_CROSS = (3.0, 0.6, 0.6)  # cross recess: length, width, depth
RIVET_DIAMETER = 4.0
RIVET_HEAD_TOP = (6.5, 1.2)   # domed head on the plate's top: diameter, height
# Race cap under the fork's top disc (wheel side), concentric pressed steps (video frames):
# (radius, depth below the fork's underside) from the rim inwards to the flat rivet head.
# Across the video frame the cap is about 0.57 of the fork's top disc and the flat rivet head
# about 0.46 of it: on the r 10 mm disc r 5.7 and r 4.6
RACE_CAP = ((5.7, 0.0), (5.7, 0.4), (5.2, 0.8), (4.9, 0.8), (4.8, 1.2), (4.7, 1.2))
RIVET_HEAD_BOTTOM_RADIUS = 4.6  # the large flat rivet head (video frames)
RIVET_BOTTOM_Z = -3.3         # frame alignment: the reference base's lowest point

# The fork's top disc sits inside the race's outer lip, about three quarters of its diameter,
# and the arms leave the disc's sides (video frames)
FORK_TOP_RADIUS = 10.0
FORK_THICKNESS = 1.5
FORK_INNER_HALF_WIDTH = 8.6   # 17.2 mm between the arms for the 15.6 mm wheel
FORK_BEND_RADIUS = 1.0        # inner radius of the sheet bend from top plate to arm
# x range where each arm leaves the top disc: the disc's chord at the bend,
# sqrt(10.0^2 - 7.6^2) = 6.50
FORK_ARM_ROOT = (-6.49, 6.49)
FORK_FRONT_CURVE = (-3.6, -8.2)  # a point on the arm's concave front edge (photos)
AXLE_X, AXLE_Z = -16.0, -20.0  # caster_wheel.yaml: wheel dx -0.016, dz -0.04 (fork offset 0.02)
AXLE_BOSS_RADIUS = 4.5
AXLE_EMBOSS = (4.0, 0.4)      # pressed boss around the axle on the arm: radius, height
AXLE_DIAMETER = 3.0
AXLE_WASHER = (3.75, 0.6)     # radius, thickness
AXLE_HEAD = (3.0, 1.4)        # domed rivet head on each side: radius, height

WHEEL_RADIUS = 15.0           # caster_wheel.yaml: radius 0.015
WHEEL_HALF_WIDTH = 7.8        # hub width 15.6 mm (frame alignment)
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


def phillips_screw(x: float, y: float, z_flat: float) -> cq.Workplane:
    """Return a pan head screw head below z_flat with a cross recess."""
    head = dome(SCREW_HEAD[0], z_flat, SCREW_HEAD[1], up=False)
    length, width, depth = SCREW_CROSS
    tip = z_flat - SCREW_HEAD[1]
    cross = (cq.Workplane('XY', origin=(0, 0, tip - 0.01)).rect(length, width).extrude(depth)
             .union(cq.Workplane('XY', origin=(0, 0, tip - 0.01)).rect(width, length)
                    .extrude(depth)))
    return head.cut(cross).translate((x, y, 0))


def make_base() -> Dict[str, cq.Workplane]:
    """Return the fixed part per material: steel plate, bearing, rivet, screws, brass standoffs."""
    ax = SWIVEL_AXIS_X
    plate_bottom = STANDOFF_TOP_Z - STANDOFF_LENGTH - PLATE_THICKNESS
    plate_top = plate_bottom + PLATE_THICKNESS
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
    plate = plate.cut(cq.Workplane('XY', origin=(0, 0, plate_bottom - 1))
                      .pushPoints(holes).circle(PLATE_HOLE_DIAMETER / 2)
                      .extrude(PLATE_THICKNESS + 2))
    r0, r1, h = RACE_RIDGE
    ridge = revolve_path((r0, plate_top), [
        ('arc', ((r0 + r1) / 2, plate_top + h), (r1, plate_top))]).translate((ax, 0, 0))
    # The bearing ring between the plate and the fork's top plate (z = 0): rounded edges
    ri, ro = RACE_RING
    gap = plate_bottom
    ring = revolve_path((ri, 0.0), [
        ('line', (ro - gap / 2, 0.0)),
        ('arc', (ro, gap / 2), (ro - gap / 2, gap)),
        ('line', (ri, gap))]).translate((ax, 0, 0))
    steel = plate.union(ridge).union(ring)
    # Rivet: shaft, domed head on the plate's ridge, stepped race cap and flat head underneath
    rivet_top = plate_top + h
    steel = steel.union(disc(RIVET_DIAMETER, RIVET_BOTTOM_Z, rivet_top - RIVET_BOTTOM_Z, x=ax))
    steel = steel.union(dome(RIVET_HEAD_TOP[0], rivet_top, RIVET_HEAD_TOP[1], up=True, x=ax))
    # The cap hangs from the fork's underside. Its flat head ends at the frame's lowest point
    cap_points = [(r, -FORK_THICKNESS - depth) for r, depth in RACE_CAP]
    head_r = RIVET_HEAD_BOTTOM_RADIUS
    head_top = cap_points[-1][1]
    head_h = head_top - RIVET_BOTTOM_Z
    if head_h < 0.2:
        raise ValueError(f'race cap reaches {head_top} mm: no room for the rivet head above '
                         f'{RIVET_BOTTOM_Z} mm')
    if max(r for r, _ in RACE_CAP) >= FORK_INNER_HALF_WIDTH - FORK_BEND_RADIUS:
        raise ValueError("the race cap doesn't fit between the fork's bends")
    cap = revolve_path((0, cap_points[0][1]), [
        *[('line', point) for point in cap_points],
        ('line', (head_r, head_top)), ('line', (head_r, RIVET_BOTTOM_Z + head_h / 2)),
        ('arc', (head_r - head_h / 4, RIVET_BOTTOM_Z + head_h / 10),
         (head_r - head_h / 2, RIVET_BOTTOM_Z)),
        ('line', (0, RIVET_BOTTOM_Z))]).translate((ax, 0, 0))
    steel = steel.union(cap)
    for x, y in holes:
        steel = steel.union(phillips_screw(x, y, plate_bottom))
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


def fork_arm(side: int) -> cq.Workplane:
    """Return one fork arm on side +1 or -1: a sheet with a concave front edge and an axle boss."""
    x0, x1 = FORK_ARM_ROOT
    top = -FORK_THICKNESS - FORK_BEND_RADIUS
    inner = FORK_INNER_HALF_WIDTH if side > 0 else -FORK_INNER_HALF_WIDTH - FORK_THICKNESS
    r = AXLE_BOSS_RADIUS
    arm = (cq.Workplane('XZ', origin=(0, inner + FORK_THICKNESS, 0))
           .moveTo(x0, top).lineTo(x1, top)
           .threePointArc(FORK_FRONT_CURVE, (AXLE_X + r * 0.6, AXLE_Z + r * 0.8))
           .threePointArc((AXLE_X + r * 0.2, AXLE_Z - r * 0.98), (AXLE_X - r, AXLE_Z))
           .close().extrude(FORK_THICKNESS))
    # Sheet bend between the top plate and the arm: a quarter tube along x
    ri, ro = FORK_BEND_RADIUS, FORK_BEND_RADIUS + FORK_THICKNESS
    centre_y = side * (FORK_INNER_HALF_WIDTH - ri)
    bend = (cq.Workplane('YZ', origin=(x0, centre_y, top))
            .circle(ro).circle(ri).extrude(x1 - x0)
            .intersect(cq.Workplane('YZ', origin=(x0, centre_y + side * ro / 2, top + ro / 2))
                       .rect(ro, ro).extrude(x1 - x0)))
    outer_y = side * (FORK_INNER_HALF_WIDTH + FORK_THICKNESS)
    emboss_r, emboss_h = AXLE_EMBOSS
    emboss = (cq.Workplane('XZ', origin=(0, outer_y + (side > 0) * emboss_h, 0))
              .center(AXLE_X, AXLE_Z).circle(emboss_r).extrude(emboss_h)
              .edges().fillet(emboss_h * 0.45))
    return arm.union(bend).union(emboss)


def make_fork() -> Dict[str, cq.Workplane]:
    """Return the swivelling fork per material: bent steel fork, axle rivet with washers."""
    x0, x1 = FORK_ARM_ROOT
    flat = FORK_INNER_HALF_WIDTH - FORK_BEND_RADIUS
    chord = math.sqrt(FORK_TOP_RADIUS ** 2 - flat ** 2)
    if max(-x0, x1) > chord:
        raise ValueError("the fork's arms must leave the top disc within its chord at the bend")
    if FORK_TOP_RADIUS >= RACE_RING[1]:
        raise ValueError("the fork's top disc must leave the race's outer lip visible")
    top = (cq.Workplane('XY', origin=(0, 0, -FORK_THICKNESS)).circle(FORK_TOP_RADIUS)
           .extrude(FORK_THICKNESS)
           .intersect(cq.Workplane('XY', origin=(0, 0, -FORK_THICKNESS))
                      .rect(2 * FORK_TOP_RADIUS + 2, 2 * flat).extrude(FORK_THICKNESS)))
    top = top.cut(disc(RIVET_DIAMETER + 0.4, -FORK_THICKNESS - 1, FORK_THICKNESS + 2))
    steel = top.union(fork_arm(1)).union(fork_arm(-1))
    axle_hole = (cq.Workplane('XZ', origin=(0, 20, 0)).center(AXLE_X, AXLE_Z)
                 .circle(AXLE_DIAMETER / 2 + 0.1).extrude(40))
    steel = steel.cut(axle_hole)
    outer = FORK_INNER_HALF_WIDTH + FORK_THICKNESS + AXLE_EMBOSS[1]
    parts = [cq.Workplane('XZ', origin=(0, outer + 0.01, 0)).center(AXLE_X, AXLE_Z)
             .circle(AXLE_DIAMETER / 2).extrude(2 * outer + 0.02)]
    washer_r, washer_t = AXLE_WASHER
    head_r, head_h = AXLE_HEAD
    for side in (-1, 1):
        y_washer = side * outer
        parts.append(cq.Workplane('XZ', origin=(0, y_washer + (side > 0) * washer_t, 0))
                     .center(AXLE_X, AXLE_Z).circle(washer_r).extrude(washer_t))
        head = dome(2 * head_r, 0.0, head_h, up=True).rotate((0, 0, 0), (1, 0, 0), -90 * side)
        parts.append(head.translate((AXLE_X, y_washer + side * washer_t, AXLE_Z)))
    return {'steel': steel, 'axle': union_all(parts)}


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
