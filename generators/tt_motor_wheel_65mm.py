# SPDX-FileCopyrightText: 2026 ROS Mobile Robots
# SPDX-License-Identifier: BSD-3-Clause
"""Generate the 65 mm wheel for TT gear motors: yellow five-spoke rim, black block-tread tyre.

Modelled from caliper measurements of the real wheel (tyre taken off the rim), photos and video
frames of DiffBot's and Remo's wheels, shop photos and product listings. The rim is a barrel with
a flange at each end, four grooves and three ridges between them, and axial ribs across the
two middle grooves. The outer flange's face is flat with a fine groove. Inside, a lip stands
inward behind the outer face. Behind the lip the barrel's inside widens towards the motor
side. Five flat tapered spokes run outwards from the hub's face. They pass in front of the
lip, and each spoke's end comes down onto the lip in a round. Each spoke has a round dimple
with a rounded rim. Behind each spoke stands a fin, tall at the barrel and low at the hub. A
bowl is recessed into the spokes' central face, with the centre boss and four small pins in
it. On the motor side the cup's back edge is rounded, and the hub tube stands on a collar with
five bulges, one where each fin starts. The tyre is a hollow section like a
car tyre's: its beads sit inside the rim in the grooves next to the flanges, its sidewalls
rise over the flanges and are widest halfway up, and its wall is about equally thick
everywhere. Sources in provenance/tt_motor_wheel_65mm.md.

Frame (positions in mm, the files are in metres): the axle along z. The wheel's outer face is
at z = -13 mm: the rim's outer flange face, flush with the tyre's sidewall at its widest. The rim's
motor-side flange face is at +12.00 mm and the hub tube's end at +15.42 mm.
"""
import math
from typing import List
from typing import Sequence
from typing import Tuple

import cadquery as cq
from OCP.BRepBuilderAPI import BRepBuilderAPI_NurbsConvert

from generators.mesh_export import Model
from generators.mesh_export import phong
from generators.mesh_export import SubPart
from generators.mesh_export import tessellate_faces

TITLE = 'Wheel 65 mm for TT gear motors (DiffBot, Remo)'

# ---------------------------------------------------------------------------------------------
# Parameters, in mm, the source next to each value (provenance/tt_motor_wheel_65mm.md):
#   pNN       caliper photo NN of the measured wheel (tyre taken off the rim)
#   p37 sil.  the silhouette of side photo 37, scaled with the 25.00 flange to flange (p40)
#   p41 img.  face-on photo 41, 25.7 px/mm from its own 51.25 reading
#   tip photo a close photo across the outer face: the spokes' tips and the lip
#   video     video frames of DiffBot's wheel
#   measured  caliper measurement of the real wheel, without a photo
#   derived   follows from other values in this file
#   choice    not measured: an estimate that looks like the photos
# z is along the axle (see the module docstring). Diameters as D_, radii are derived.
# ---------------------------------------------------------------------------------------------
TYRE_OUTER_FACE_Z = -13.0    # the outer face: 13 mm outboard of both robots' wheel joints

# Tyre: a hollow section like a car tyre's (the real tyre, p03-p09). Its wall is about equally
# thick everywhere, so its inside follows its outside (measured). The beads at its inner
# diameter sit inside the rim, in the grooves between the flanges and the outer ridges. The
# sidewalls rise over the flanges and are widest halfway up, flush with the flanges' faces. The
# crown is a little narrower
TYRE_OUTER_D = 66.6          # measured on DiffBot: 66.3 and 66.8 (front_wheel.yaml)
TYRE_WALL = 1.9              # measured (about): the wall's thickness, everywhere
TYRE_BEAD_D = 50.0           # measured: the beads' inner diameter (p07, p08: bead to tread)
TYRE_BEAD_WIDTH = 3.3        # measured: the bead's base, along the axle
TYRE_BEAD_HEIGHT = 1.9       # measured (about): the bead's height, from its base to its top
TYRE_BEAD_ROUND = 0.5        # choice: the round where the sidewall's inside meets the bead's top
# The sidewalls stand this far past the flanges' faces at their widest: flush on the real
# tyre, not measured (provenance: measurement 1)
TYRE_BULGE = 0.0             # the real tyre
TYRE_WIDEST_D = 59.2         # the real tyre: widest halfway between flange (51.22) and tread
TYRE_CROWN_WIDTH = 18.6      # choice (shop photos): the tread between the shoulders' rounds
TYRE_CROWN_DROP = 0.25       # choice (shop photos): the crown's edges below its middle
TYRE_SEAT_GAP = 0.02         # choice: the tyre stands this far off the rim
TYRE_FOOT_INSET = 0.4        # choice: the sidewalls leave the flanges' tops behind the faces
TREAD_DEPTH = 0.6            # measured: the grooves' depth
TREAD_ROWS = 42              # p07, p08: about 5 mm row pitch, blocks about 4.3 mm long
TREAD_TRANSVERSE_GROOVE = 0.7  # measured: the grooves' width, across and around
# Grooves around the tyre (offset from the tyre's centre plane, width): six block columns
# (p07, p08, video). Measured: the offsets
TREAD_CIRCUMFERENTIAL_GROOVES = ((0.0, 0.7), (3.15, 0.7), (-3.15, 0.7), (7.8, 0.7), (-7.8, 0.7))
TREAD_SHOULDER_REACH = 31.5  # measured: the cross grooves reach down the sidewalls to this radius

# Rim barrel (from the outer flange face F towards the motor side)
RIM_WIDTH = 25.00            # p40: flange to flange
RIM_LENGTH = 28.42           # p33: the outer flange face to the hub tube's end
FLANGE_D = 51.22             # p01, p41 (51.25)
RIDGE_DS = (50.9, 50.8, 50.8)  # measured: ridge 1 (next to the outer flange), ridges 2 and 3
GROOVE_D = 47.91             # p02: the grooves' bottoms
# Behind the lip the barrel's inside is a cone that widens towards the motor side (measured)
BARREL_LIP_D = 44.6          # measured: 44.5 to 44.6, at the lip's back
BARREL_END_D = 45.55         # measured: where the round at the motor-side end begins
# The four grooves between the flanges and the three ridges, from the outer flange: the width
# between the walls where they reach the tops' level, the bottom's width, and how far the bottom
# starts behind that on the outer-face side (0: that wall is vertical). The motor-side wall
# takes the rest of the slope
BARREL_GROOVES = (
    (3.9, 3.9, 0.0),         # measured: a rectangle
    (5.8, 4.1, 0.85),        # measured: the widths; choice: both walls slope the same
    (4.8, 3.9, 0.0),         # measured: the outer-face wall vertical, the other one sloped
    (3.9, 3.9, 0.0))         # measured: a rectangle
FLANGE_TOPS = (1.42, 1.58)   # measured: the outer flange 1.4 to 1.45, the other 1.55 to 1.6
# measured: where the flat tops of ridges 2 and 3 start, from the outer face. None for ridge 1:
# its top edge on the outer-face side is sharp (measured), so its flat top starts at its wall
RIDGE_STARTS = (None, 12.4, 18.5)
RIDGE_TOP_MIN = 1.0          # measured: each ridge's flat top is 1.0 wide or a little more
# derived: what the flanges and the grooves leave of the rim's width for each ridge, between
# its walls continued to the tops' level: 1.2. The flat top of ridge 1 is that wide. The flat
# tops of ridges 2 and 3 start where they were measured, behind the ridge's wall on the
# outer-face side, and end at the ridge's motor-side wall. So they are 1.12 and 1.02 wide, and
# the top edge on the outer-face side is rounded: the round takes what lies between the
# continued wall and the flat top. A groove next to such an edge is therefore wider between
# the flat tops than between its walls
RIDGE_FOOT = (RIM_WIDTH - sum(FLANGE_TOPS) - sum(groove[0] for groove in BARREL_GROOVES)) / 3
RIBS = 7                     # p33, p35: about 50 deg apart
RIB_WIDTH = 1.0              # choice (photos: thin)
RIB_D = min(RIDGE_DS)        # choice: the ribs are as high as the lower ridges

# Outer face: the flange's flat face with a fine groove. Both flanges' outer corners are sharp,
# their inner corners are round (measured: the shapes and the radius)
FLANGE_FACE = 3.2            # measured: the face's radial height, so 44.82 inside at the face
# The fine groove lies in the middle of the face (measured: 1.3 from each of the face's edges).
# Its side toward the tyre is a chamfer, its side toward the axle a round, and its two edges at
# the face are rounded (measured: the shapes). The round's radius follows from the other values
FACE_GROOVE_MOUTH = 0.7      # measured: the groove's width at the face
FACE_GROOVE_BOTTOM = 0.1     # measured: the width of the groove's bottom
FACE_GROOVE_DEPTH = 0.2      # choice
FACE_GROOVE_CHAMFER = 45.0   # choice: the chamfer's angle to the face (deg)
FACE_GROOVE_EDGE = 0.1       # choice: the rounds of the groove's two edges at the face
FLANGE_INNER_ROUND = 0.5     # measured (about): the round into the barrel's inside, at both ends

# Barrel inside, from the outer face: plain, a lip that stands inward, then the cone above
LIP_START = 3.5              # measured: the lip's front behind the outer face
LIP_WIDTH = 1.7              # measured: along the axle (19.8 remain to the motor-side face)
LIP_INNER_D = 40.8           # measured. The lip stands 2.01 inward at its front, 1.9 at its back

# Hub, outer face: a bowl recessed into the spokes' flat central face, its edge in that face
# (no raised ring), the boss standing in it below the face (p21, p23, p24, p41)
BOSS_D = 9.69                # p23
BOSS_CHAMFER = 0.3           # choice
BOWL_EDGE_D = 17.9           # p41 img.: the bowl's edge in the spokes' face
BOWL_D = 17.62               # p21: inside jaws in the bowl at about the boss's top
# Below that the wall rounds into the floor: p20 reads 14.07 with the jaws deeper in the bowl
BOWL_FLOOR_D = 13.8          # p41 img.: the wall's foot at r 6.9
BOWL_DEPTH = 5.0             # measured: the floor behind the spokes' face
BOSS_TOP_DEPTH = 3.0         # measured: the boss's top behind the spokes' face
BOWL_FLOOR = 1.2             # choice: the floor's thickness, to the cup's back on the motor side
BOSS_CAP = 0.8               # choice: the boss's closed top (photos), down to the bore's end
BOWL_EDGE_ROUND = 0.2        # choice
BOSS_FILLET = 0.3            # choice: where the boss meets the floor
CUP_D = 20.64                # p24: the bowl's outside behind the face, between the spokes
HUB_PINS = 4                 # p41 img.: four small pins on the bowl's floor, 90 deg apart
HUB_PIN_R = 6.4              # p41 img.: at the foot of the bowl's wall
HUB_PIN_D = 1.2              # measured
HUB_PIN_HEIGHT = 1.45        # measured: above the bowl's floor, the tops below the boss's top
HUB_PIN_ANGLE = 0.0          # p41 img.: one pin in line with a spoke
WEB_R = 11.2                 # p41 img.: windows start at r 11.4 between the spokes; cup 10.32

# Spokes: five, flat and tapered, each symmetric about a radial centre line (p41 img.). The
# spokes' plate passes in front of the lip. A spoke's end comes down onto the lip in a round: a
# quarter circle from the flat face to the corner of the lip's front and the barrel's inside
# (measured: the shape). From the lip's inner edge outwards the end is solid behind the plate,
# down to the lip's front (choice), so the spoke stands on the lip. Seen from outside, a
# spoke's sides meet the lip's inner edge in sharp corners (measured). The spokes' back is flat
SPOKES = 5
SPOKE_FACE_DEPTH = 1.0       # measured: the spokes' face behind the outer flange face
SPOKE_PLATE = 1.7            # measured: the spokes' thickness
SPOKE_ROOT_RADIUS = 9.6      # choice: the plate starts inside the cup's wall
# Spoke widths between the sides. The sides are straight lines through two places: above the
# lip's inner edge, and at the spoke's end, its foot on the lip at the barrel's inside. The
# spoke is 5.95 wide where its end's round starts and 5.35 at r 21.9, on the round (p25). The
# taper gives 8.0 at r 13.2, 7.3 at r 15.6, 6.8 at r 17 and 6.2 at r 19 (p41 img.: 8.9, 7.6,
# 7.0 and 6.2)
SPOKE_LIP_WIDTH = 5.8        # measured: 5.8 to 6.0, above the lip's inner edge
SPOKE_END_WIDTH = 5.2        # measured: where the spoke ends on the lip
SPOKE_ROOT_BLEND = 3.0       # video, p41: curved sides into the web; radius: choice
# One round dimple per spoke: a small bowl with a straight sloped wall, a rounded bottom and a
# rounded rim (measured: closed, about 0.6 deep, a small round on the rim, a bit wider than 1.5
# at the face). The rim's round makes the dimple 1.74 wide at the face
SPOKE_MARK_FROM_END = 7.4    # measured: the dimple's centre before the spoke's end
SPOKE_MARK_D = 1.5           # p41 img.: across the dimple, its wall continued to the face
SPOKE_MARK_DEPTH = 0.6       # measured (about): at the centre of the dimple
SPOKE_MARK_ROUND = 0.3       # choice: the round of the dimple's rim
SPOKE_MARK_BOTTOM = 0.3      # choice: the radius of the dimple's rounded bottom

# Motor side
CUP_BACK_ROUND = 2.5         # measured: the cup's back edge is rounded. Choice (sketch): R2.5
# The collar at the hub tube's foot: five bulges, one where each fin starts, and a rounded
# notch between each two (measured: the shapes). In a notch the collar runs level from the
# cup's back for a short way and then curves down to the hub tube (sketch)
HUB_COLLAR_D = 15.27         # p15: across the collar; taken as the circle around the bulges' tops
HUB_BULGE_WIDTH = 3.47       # p18, p19: across a bulge, between its two flanks
HUB_BULGE_HEIGHT = 1.0       # measured: a bulge's top above the collar's level part in a notch
HUB_BULGE_START = 17.5       # measured: where the bulges start, before the hub tube's end
HUB_BULGE_ROUND = 0.5        # choice: the rounds of a bulge's two long edges
HUB_BULGE_END_ROUND = 0.3    # choice: the round of a bulge's edges at its end
HUB_COLLAR_FLAT = 0.8        # choice (sketch): how far the collar runs level in a notch
HUB_TUBE_D = 9.95            # p29, p30: 9.97 and 9.92
HUB_TUBE_CHAMFER = 0.5       # choice
BORE_D = 5.44                # p14: the round side; p12, p13 tube wall 2.10 and 2.34
BORE_FLATS = 3.66            # listings (X2 Robotics, VXB); not measured
# Fins: one behind each spoke, on the spoke's flat back, tall at the barrel and low at the hub
# (measured: the shape). A fin's edge leaves the barrel's inside straight toward the axle,
# turns with a round into a long straight edge down toward the hub, and turns with a second
# round toward the axle again. There it meets the top of its bulge on the collar (derived: the
# second round ends on the bulge's top)
FIN_THICKNESS = 1.51         # p31
FIN_BARREL_END = 12.0        # measured: the edge meets the barrel, before the motor-side face
FIN_HUB_END = 19.3           # measured: the edge meets the hub, before the hub tube's end
FIN_BARREL_CORNER_R = 18.6   # choice (sketch): where the edge turns near the barrel, from the axle
FIN_ROUND = 3.0              # choice: the radius of the two rounds

# Derived positions
RIM_FACE_Z = TYRE_OUTER_FACE_Z + TYRE_BULGE   # F: the outer flange face, -13.0
RIM_INNER_FACE_Z = RIM_FACE_Z + RIM_WIDTH      # the motor-side flange face, 12.0
HUB_TUBE_END_Z = RIM_FACE_Z + RIM_LENGTH       # 15.42
TYRE_INNER_FACE_Z = RIM_INNER_FACE_Z + TYRE_BULGE  # the tyre's motor side at its widest, 12.0
TYRE_CENTRE_Z = RIM_FACE_Z + RIM_WIDTH / 2     # -0.5
FIN_BARREL_END_Z = RIM_INNER_FACE_Z - FIN_BARREL_END  # where a fin's edge meets the barrel, 0.0
FIN_HUB_END_Z = HUB_TUBE_END_Z - FIN_HUB_END    # where a fin's edge meets its bulge, -3.88
HUB_COLLAR_END_Z = HUB_TUBE_END_Z - HUB_BULGE_START  # the collar's end: the bulges start, -2.08
FACE_INNER_R = FLANGE_D / 2 - FLANGE_FACE      # the barrel's inside at the outer face, 22.41
LIP_INNER_R = LIP_INNER_D / 2                  # the lip's inner edge, 20.4
LIP_FRONT_Z = RIM_FACE_Z + LIP_START
SPOKE_FACE_Z = RIM_FACE_Z + SPOKE_FACE_DEPTH   # the spokes' face, in front of the lip's front
SPOKE_PLATE_BACK_Z = SPOKE_FACE_Z + SPOKE_PLATE  # the plate's back, in front of the lip's front
SPOKE_END_ROUND = LIP_START - SPOKE_FACE_DEPTH  # the round from the face down to the lip, 2.5
SPOKE_END_START_R = FACE_INNER_R - SPOKE_END_ROUND  # where the flat face ends, 19.91
SPOKE_MARK_RADIUS = FACE_INNER_R - SPOKE_MARK_FROM_END  # the dimples' centres, 15.01
BOSS_TOP_Z = SPOKE_FACE_Z + BOSS_TOP_DEPTH
BOWL_FLOOR_Z = SPOKE_FACE_Z + BOWL_DEPTH
CUP_BACK_Z = BOWL_FLOOR_Z + BOWL_FLOOR         # the cup's back on the motor side
BORE_END_Z = BOSS_TOP_Z + BOSS_CAP             # the blind bore's end inside the boss

# Tessellation: chordal deviation (mm) and angle (rad)
LINEAR_DEFLECTION = 0.03
ANGULAR_DEFLECTION = 0.3

MATERIALS = {
    # The rim's yellow and the tyre's black, as remo_description's wheel colours
    'rim': phong((0.95, 0.72, 0.05, 1.0), (0.25, 0.25, 0.25, 1.0), 20.0),
    'tyre': phong((0.10, 0.10, 0.10, 1.0), (0.05, 0.05, 0.05, 1.0), 5.0),
}


# ---------------------------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------------------------
def revolve_profile(points: Sequence[Tuple[float, float]]) -> cq.Workplane:
    """Revolve a closed (r, z) polygon about the z axis."""
    return cq.Workplane('XZ').polyline(list(points)).close().revolve(360, (0, 0, 0), (0, 1, 0))


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


def fillet_corner(corner: Tuple[float, float], radius: float,
                  first: Tuple[float, float], second: Tuple[float, float]) -> List:
    """Return the path pieces that round a corner between two straight edges at any angle.

    first and second are the unit directions from the corner to the previous and the next
    point of the path. The pieces are ('line', end) and ('arc', middle, end) for revolve_path.
    """
    cx, cz = corner
    angle = math.acos(max(-1.0, min(1.0, first[0] * second[0] + first[1] * second[1])))
    reach = radius / math.tan(angle / 2)
    start = (cx + first[0] * reach, cz + first[1] * reach)
    end = (cx + second[0] * reach, cz + second[1] * reach)
    bx, bz = first[0] + second[0], first[1] + second[1]
    length = math.hypot(bx, bz)
    bx, bz = bx / length, bz / length
    centre_distance = radius / math.sin(angle / 2)
    middle = (cx + bx * (centre_distance - radius), cz + bz * (centre_distance - radius))
    return [('line', start), ('arc', middle, end)]


def round_corner_by(corner: Tuple[float, float], reach: float,
                    first: Tuple[float, float], second: Tuple[float, float]) -> List:
    """Return the path pieces that round a corner so that the round takes reach off each edge.

    first and second are the unit directions from the corner to the previous and the next
    point of the path. The pieces are ('line', end) and ('arc', middle, end) for revolve_path.
    """
    angle = math.acos(max(-1.0, min(1.0, first[0] * second[0] + first[1] * second[1])))
    return fillet_corner(corner, reach * math.tan(angle / 2), first, second)


def ellipse(centre: Tuple[float, float], radius_r: float, radius_z: float,
            start: float, end: float, count: int = 12,
            directions: Sequence[Tuple[float, float]] = ()) -> Tuple:
    """Return a ('through', points) piece along a quarter ellipse, from angle start to end (deg).

    The start point (at angle start) is the path's current point and isn't repeated. With
    directions, the piece also names the curve's directions at its start and its end, so that it
    leaves and meets its neighbours without a corner.
    """
    points = []
    for k in range(1, count + 1):
        angle = math.radians(start + (end - start) * k / count)
        points.append((centre[0] + radius_r * math.cos(angle),
                       centre[1] + radius_z * math.sin(angle)))
    return ('through', points, *directions)


def unit(dx: float, dz: float) -> Tuple[float, float]:
    """Return the direction (dx, dz) with length 1."""
    length = math.hypot(dx, dz)
    return dx / length, dz / length


def trace_path(start: Tuple[float, float], pieces: Sequence, y: float = 0.0) -> cq.Workplane:
    """Return a closed (r, z) path as a wire in the plane of the x axis and the axle, moved to y.

    The pieces are ('line', end), ('arc', middle, end) or ('through', points), a spline
    through the points, all in (r, z). After its points a 'through' piece may name the
    spline's directions at its start and its end.
    """
    wp = cq.Workplane('XZ', origin=(0, y, 0)).moveTo(*start)
    for piece in pieces:
        if piece[0] == 'line':
            wp = wp.lineTo(*piece[1])
        elif piece[0] == 'arc':
            wp = wp.threePointArc(piece[1], piece[2])
        elif piece[0] == 'through':
            wp = wp.spline(list(piece[1]), tangents=list(piece[2:]) or None, includeCurrent=True)
        else:
            raise ValueError(f'unknown piece: {piece[0]}')
    return wp.close()


def revolve_path(start: Tuple[float, float], pieces: Sequence) -> cq.Workplane:
    """Revolve a closed (r, z) path (trace_path's pieces) about the z axis."""
    return trace_path(start, pieces).revolve(360, (0, 0, 0), (0, 1, 0))


def union_all(shapes: Sequence[cq.Workplane]) -> cq.Workplane:
    """Return the union of the shapes."""
    result = shapes[0]
    for shape in shapes[1:]:
        result = result.union(shape)
    return result


def groove_spans() -> List[Tuple[float, float, float, float]]:
    """Return each barrel groove's z positions: top start, bottom start, bottom end, top end.

    The grooves follow each other from the outer flange's top to the motor-side flange's top,
    with a ridge between two grooves. The top positions are where the walls reach the tops'
    level: there a ridge is RIDGE_FOOT wide, its flat top and its rounded edge.
    """
    spans = []
    z = RIM_FACE_Z + FLANGE_TOPS[0]
    for top, bottom, lead in BARREL_GROOVES:
        spans.append((z, z + lead, z + lead + bottom, z + top))
        z += top + RIDGE_FOOT
    return spans


def ridge_edges() -> List[float]:
    """Return how far each ridge's flat top starts behind its continued outer-face-side wall.

    The flat tops start at RIDGE_STARTS, or at the wall itself where RIDGE_STARTS has None (0:
    a sharp edge). The walls' positions are those of groove_spans(). A flat top ends at the
    ridge's motor-side wall, and must be at least RIDGE_TOP_MIN wide.
    """
    spans = groove_spans()
    edges = []
    for k, start in enumerate(RIDGE_STARTS):
        flat_start = spans[k][3] if start is None else RIM_FACE_Z + start
        edge, top = flat_start - spans[k][3], spans[k + 1][0] - flat_start
        if edge < 0 or top < RIDGE_TOP_MIN:
            raise ValueError("a ridge's flat top must start behind the ridge's outer-face-side "
                             'wall and be at least RIDGE_TOP_MIN wide')
        edges.append(edge)
    return edges


def crown_arc() -> Tuple[float, float]:
    """Return the crown's radius of curvature and the sine of its half angle.

    The crown is an arc through its middle (the tyre's largest radius) and its two edges, which
    lie TYRE_CROWN_DROP lower, TYRE_CROWN_WIDTH apart.
    """
    half = TYRE_CROWN_WIDTH / 2
    radius = (half * half + TYRE_CROWN_DROP * TYRE_CROWN_DROP) / (2 * TYRE_CROWN_DROP)
    return radius, half / radius


def tyre_outer_pieces(inset: float) -> List:
    """Return the tyre's outer contour from the outer foot to the motor-side foot, as path pieces.

    The contour starts at the foot on the outer flange's top (the path's start point, see
    tyre_foot). Each sidewall bulges from its foot to its widest point and rounds over the
    shoulder to the crown, quarter ellipses, so the widest point is exactly the outermost. Each
    shoulder meets the crown's arc in the arc's direction, without a corner. With an inset, the
    feet, the widest points and the crown move inwards by it (the tread's grooves are cut
    outside the inset copy).
    """
    outer = TYRE_OUTER_D / 2 - inset
    wide = TYRE_WIDEST_D / 2
    z_out = RIM_FACE_Z - TYRE_BULGE + inset
    z_in = RIM_INNER_FACE_Z + TYRE_BULGE - inset
    half = TYRE_CROWN_WIDTH / 2
    edge = outer - TYRE_CROWN_DROP
    foot_r, foot_out = tyre_foot(inset)
    foot_in = RIM_INNER_FACE_Z - TYRE_FOOT_INSET - inset
    c = TYRE_CENTRE_Z
    sidewall_r, shoulder_r = wide - foot_r, edge - wide
    _, sine = crown_arc()
    cosine = math.sqrt(1 - sine * sine)
    return [
        ellipse((wide, foot_out), sidewall_r, foot_out - z_out, 180, 270),
        ellipse((wide, c - half), shoulder_r, c - half - z_out, 270, 360,
                directions=((1.0, 0.0), (sine, cosine))),
        ('arc', (outer, c), (edge, c + half)),
        ellipse((wide, c + half), shoulder_r, z_in - c - half, 0, 90,
                directions=((-sine, cosine), (-1.0, 0.0))),
        ellipse((wide, foot_in), sidewall_r, z_in - foot_in, 90, 180)]


def tyre_foot(inset: float) -> Tuple[float, float]:
    """Return where the tyre's outer sidewall meets the outer flange's top (moved by inset)."""
    return (FLANGE_D / 2 + TYRE_SEAT_GAP + inset, RIM_FACE_Z + TYRE_FOOT_INSET + inset)


def tyre_envelope(inset: float) -> cq.Workplane:
    """Return the region inside the tyre's outer contour (moved inwards by inset)."""
    return revolve_path(tyre_foot(inset), tyre_outer_pieces(inset))


def shoulder_inside(centre_z: float, radius_z: float, start: float, end: float,
                    count: int = 12) -> List[Tuple[float, float]]:
    """Return points on a shoulder's inside: the shoulder's ellipse moved inwards by TYRE_WALL.

    The ellipse is the outer contour's (see tyre_outer_pieces), about (TYRE_WIDEST_D / 2,
    centre_z). Each point lies TYRE_WALL from the ellipse along the ellipse's normal. The points
    run from angle start to end (deg), without the two ends.
    """
    wide = TYRE_WIDEST_D / 2
    radius_r = TYRE_OUTER_D / 2 - TYRE_CROWN_DROP - wide
    points = []
    for k in range(1, count):
        angle = math.radians(start + (end - start) * k / count)
        normal = unit(math.cos(angle) / radius_r, math.sin(angle) / radius_z)
        points.append((wide + radius_r * math.cos(angle) - TYRE_WALL * normal[0],
                       centre_z + radius_z * math.sin(angle) - TYRE_WALL * normal[1]))
    return points


def tyre_body() -> cq.Workplane:
    """Return the tyre without tread grooves: beads in the rim, sidewalls, crown, hollow inside.

    The wall is TYRE_WALL thick: the inside follows the outside at that distance, under the
    crown, around the shoulders and down the sidewalls. Each bead's base sits in the groove
    next to its flange, held between the flange and the outer ridge (its diameter is less than
    both), its toe at the flange's wall. The bead is TYRE_BEAD_WIDTH wide and TYRE_BEAD_HEIGHT
    high. A round joins the bead's top to the sidewall's inside.
    """
    r_seat, r_bead = FLANGE_D / 2 + TYRE_SEAT_GAP, TYRE_BEAD_D / 2
    r_top, wide = r_bead + TYRE_BEAD_HEIGHT, TYRE_WIDEST_D / 2
    f, i, c = RIM_FACE_Z, RIM_INNER_FACE_Z, TYRE_CENTRE_Z
    half, wall = TYRE_CROWN_WIDTH / 2, TYRE_WALL
    toe_out = f + FLANGE_TOPS[0] + TYRE_SEAT_GAP
    toe_in = i - FLANGE_TOPS[1] - TYRE_SEAT_GAP
    heel_out, heel_in = toe_out + TYRE_BEAD_WIDTH, toe_in - TYRE_BEAD_WIDTH
    inside_out, inside_in = f - TYRE_BULGE + wall, i + TYRE_BULGE - wall   # the sidewalls' insides
    outer, edge = TYRE_OUTER_D / 2, TYRE_OUTER_D / 2 - TYRE_CROWN_DROP
    shoulder_r, shoulder_z = edge - wide, c - half - (f - TYRE_BULGE)
    if not GROOVE_D / 2 < r_bead < min(RIDGE_DS[0], RIDGE_DS[-1]) / 2:
        raise ValueError('the beads must sit in the grooves, held by the flanges and ridges')
    outer_groove, motor_groove = groove_spans()[0], groove_spans()[-1]
    if heel_out > outer_groove[2] or heel_in < motor_groove[1]:
        raise ValueError('the beads must fit into the grooves next to the flanges')
    if not r_seat < r_top < r_top + TYRE_BEAD_ROUND < wide:
        raise ValueError("the beads' tops must lie above the flanges, below the widest diameter")
    if not inside_out + TYRE_BEAD_ROUND < heel_out:
        raise ValueError("the bead must reach further in than the sidewall's inside and its round")
    if not wall < min(shoulder_r * shoulder_r / shoulder_z, shoulder_z * shoulder_z / shoulder_r):
        raise ValueError("the wall must be thinner than the shoulders' tightest curve")
    _, sine = crown_arc()
    cosine = math.sqrt(1 - sine * sine)
    crown_in = (edge - wall * cosine, c + half - wall * sine)      # the crown's inside, motor side
    crown_out = (edge - wall * cosine, c - half + wall * sine)     # the crown's inside, outer side
    return revolve_path(tyre_foot(0.0), [
        *tyre_outer_pieces(0.0),
        # the motor-side bead: along the flange's top, down its wall, the base, up to the top
        ('line', (r_seat, toe_in)), ('line', (r_bead, toe_in)), ('line', (r_bead, heel_in)),
        ('line', (r_top, heel_in)),
        *fillet_corner((r_top, inside_in), TYRE_BEAD_ROUND, (0.0, -1.0), (1.0, 0.0)),
        # the inside: up the sidewall, around the shoulder, under the crown, and down again
        ('line', (wide, inside_in)),
        ('through', [*shoulder_inside(c + half, shoulder_z, 90, 0), crown_in],
         (1.0, 0.0), (sine, -cosine)),
        ('arc', (outer - wall, c), crown_out),
        ('through', [*shoulder_inside(c - half, shoulder_z, 360, 270), (wide, inside_out)],
         (-sine, -cosine), (-1.0, 0.0)),
        *fillet_corner((r_top, inside_out), TYRE_BEAD_ROUND, (1.0, 0.0), (0.0, 1.0)),
        # the outer bead
        ('line', (r_top, heel_out)), ('line', (r_bead, heel_out)), ('line', (r_bead, toe_out)),
        ('line', (r_seat, toe_out))])


def tread_cutter() -> cq.Workplane:
    """Return the grooves: circumferential rings and transverse slabs, one tread depth deep."""
    reach = TYRE_OUTER_D / 2 + 5
    centre = TYRE_CENTRE_Z
    width = TYRE_INNER_FACE_Z - TYRE_OUTER_FACE_Z
    rings = union_all([
        revolve_profile([(TREAD_SHOULDER_REACH, centre + off - w / 2),
                         (reach, centre + off - w / 2), (reach, centre + off + w / 2),
                         (TREAD_SHOULDER_REACH, centre + off + w / 2)])
        for off, w in TREAD_CIRCUMFERENTIAL_GROOVES])
    length = reach - TREAD_SHOULDER_REACH
    slabs = union_all([
        cq.Workplane('XY').box(length, TREAD_TRANSVERSE_GROOVE, width + 4)
        .translate((TREAD_SHOULDER_REACH + length / 2, 0, centre))
        .rotate((0, 0, 0), (0, 0, 1), 360.0 * (k + 0.5) / TREAD_ROWS)
        for k in range(TREAD_ROWS)])
    # Only the part outside the contour's inset copy: one tread depth deep, and beyond the
    # tyre (no faces in common with the tyre's own outer surface)
    return rings.union(slabs).cut(tyre_envelope(TREAD_DEPTH))


def make_tyre() -> cq.Workplane:
    """Return the black tyre with its block tread."""
    return tyre_body().cut(tread_cutter())


def spoke_side() -> Tuple[Tuple[float, float], Tuple[float, float]]:
    """Return the two points that set a spoke's straight side, on its +y side.

    The spoke's centre line is the x axis. Each point lies half the spoke's width there from
    the centre line: the first above the lip's inner edge, where the spoke has
    SPOKE_LIP_WIDTH, the second on the barrel's inside, where it has SPOKE_END_WIDTH.
    """
    r0, w0 = LIP_INNER_R, SPOKE_LIP_WIDTH
    r1, w1 = FACE_INNER_R, SPOKE_END_WIDTH
    return ((math.sqrt(r0 * r0 - w0 * w0 / 4), w0 / 2),
            (math.sqrt(r1 * r1 - w1 * w1 / 4), w1 / 2))


def spoke_outline() -> List[Tuple[float, float]]:
    """Return a spoke's outline as a polygon: straight sides, from the cup into the barrel.

    The sides are the straight lines through spoke_side()'s points, mirrored about the centre
    line. The polygon starts inside the cup's wall and ends inside the barrel's wall.
    """
    (x0, y0), (x1, y1) = spoke_side()
    slope = (y1 - y0) / (x1 - x0)
    root, tip = SPOKE_ROOT_RADIUS, FACE_INNER_R + 0.5
    half_root, half_tip = y0 + slope * (root - x0), y0 + slope * (tip - x0)
    return [(root, -half_root), (tip, -half_tip), (tip, half_tip), (root, half_root)]


def side_blend(start: Tuple[float, float], end: Tuple[float, float], side: int) -> cq.Workplane:
    """Return the filler that rounds a window's corner between a spoke's side and the hub's web.

    start and end are the spoke side's ends (unrotated spoke, start nearer the hub), side +1
    or -1 the window's side of that edge. The window lies outside the web's edge, a circle of
    radius WEB_R about the axle. The filler is the region between the corner, the spoke's
    side and a concave arc of radius SPOKE_ROOT_BLEND tangent to side and circle, as deep as
    the spoke's plate. It reaches 0.3 mm into the web, so no gap is left at the circle.
    """
    circle, radius = WEB_R, SPOKE_ROOT_BLEND
    ax, ay = start
    length = math.hypot(end[0] - ax, end[1] - ay)
    dx, dy = (end[0] - ax) / length, (end[1] - ay) / length
    nx, ny = (-dy, dx) if side > 0 else (dy, -dx)

    def along(px: float, py: float, distance: float) -> float:
        """Return t where the line (px, py) + t (dx, dy) reaches the distance from the axle."""
        b = px * dx + py * dy
        return -b + math.sqrt(b * b - (px * px + py * py - distance * distance))

    t = along(ax + radius * nx, ay + radius * ny, circle + radius)
    cx, cy = ax + radius * nx + t * dx, ay + radius * ny + t * dy
    on_edge = (cx - radius * nx, cy - radius * ny)
    c = math.hypot(cx, cy)
    on_circle = (cx * circle / c, cy * circle / c)
    past_circle = (cx * (circle - 0.3) / c, cy * (circle - 0.3) / c)
    u = along(ax, ay, circle) - 0.3
    corner = (ax + u * dx, ay + u * dy)
    mx, my = on_edge[0] + on_circle[0] - 2 * cx, on_edge[1] + on_circle[1] - 2 * cy
    m = math.hypot(mx, my)
    middle = (cx + radius * mx / m, cy + radius * my / m)
    return (cq.Workplane('XY', origin=(0, 0, SPOKE_FACE_Z)).moveTo(*corner).lineTo(*on_edge)
            .threePointArc(middle, on_circle).lineTo(*past_circle).close()
            .extrude(SPOKE_PLATE_BACK_Z - SPOKE_FACE_Z))


def spoke_plate() -> cq.Workplane:
    """Return a spoke's plate: a flat face whose end comes down onto the lip in a round.

    The outline is extruded from the face to the lip's front. A revolved cutter takes away
    what lies in front of the round: a quarter circle of radius SPOKE_END_ROUND from the flat
    face to the corner of the lip's front and the barrel's inside. The round is tangent to
    the face where it starts and to the barrel's inside where it ends, so in front of the lip
    a wedge of air stays between the two. A second cutter takes away what lies behind the
    plate's back inside the lip's inner edge. From that edge outwards the end stays solid
    down to the lip's front: the spoke's foot on the lip. A little further out the foot
    reaches 0.1 mm into the lip, so the two solids overlap instead of touching.
    """
    face, back, lip, r = SPOKE_FACE_Z, SPOKE_PLATE_BACK_Z, LIP_FRONT_Z, SPOKE_END_ROUND
    start, wall, beyond = SPOKE_END_START_R, FACE_INNER_R, FACE_INNER_R + 1.0
    k = r / math.sqrt(2)
    plate = (cq.Workplane('XY', origin=(0, 0, face)).polyline(spoke_outline()).close()
             .extrude(lip + 0.1 - face))
    in_front = revolve_path((start, face), [
        ('arc', (start + k, lip - k), (wall, lip)), ('line', (beyond, lip)),
        ('line', (beyond, face - 1)), ('line', (start, face - 1))])
    foot, in_lip = LIP_INNER_R, LIP_INNER_R + 0.1
    behind = revolve_path((0, back), [
        ('line', (foot, back)), ('line', (foot, lip)), ('line', (in_lip, lip)),
        ('line', (in_lip, lip + 0.2)), ('line', (0, lip + 0.2))])
    return plate.cut(in_front).cut(behind)


def dimple_wall_slope() -> float:
    """Return the angle (rad) between a dimple's straight wall and the spoke's face.

    The wall, continued, meets the face half SPOKE_MARK_D from the dimple's axis. It is
    tangent to the rounded bottom, a sphere of radius SPOKE_MARK_BOTTOM whose lowest point lies
    SPOKE_MARK_DEPTH behind the face. The slope that fits both is found by halving an interval.
    """
    half, depth, bottom = SPOKE_MARK_D / 2, SPOKE_MARK_DEPTH, SPOKE_MARK_BOTTOM

    def depth_for(slope: float) -> float:
        """Return the bottom's depth for a wall of this slope."""
        return half * math.tan(slope) - bottom * (1 / math.cos(slope) - 1)

    low, high = math.atan2(depth, half), math.radians(80.0)
    if not depth_for(low) <= depth <= depth_for(high):
        raise ValueError("the dimple's bottom round doesn't fit its width and depth")
    for _ in range(60):
        middle = (low + high) / 2
        low, high = (middle, high) if depth_for(middle) < depth else (low, middle)
    return (low + high) / 2


def spoke_dimple() -> cq.Workplane:
    """Return the cutter for a spoke's dimple, its axis the z axis: a bowl with a rounded rim.

    In the (r, z) profile, from the spoke's face inwards: the rim's round, an arc of radius
    SPOKE_MARK_ROUND tangent to the face and to the wall. The straight wall. The rounded
    bottom, an arc of radius SPOKE_MARK_BOTTOM about a point on the axis, tangent to the wall,
    down to the dimple's deepest point on the axis. The rim's round makes the dimple wider at
    the face than SPOKE_MARK_D. Above the face the cutter is a short cylinder.
    """
    face, half, depth = SPOKE_FACE_Z, SPOKE_MARK_D / 2, SPOKE_MARK_DEPTH
    rim, bottom = SPOKE_MARK_ROUND, SPOKE_MARK_BOTTOM
    slope = dimple_wall_slope()
    mouth = half + rim * math.tan(slope / 2)            # where the rim's round meets the face
    wall_top = (mouth - rim * math.sin(slope), face + rim * (1 - math.cos(slope)))
    rim_middle = (mouth - rim * math.sin(slope / 2), face + rim * (1 - math.cos(slope / 2)))
    centre_z = face + depth - bottom                    # the bottom's centre, on the axis
    wall_end = (bottom * math.sin(slope), centre_z + bottom * math.cos(slope))
    if not (wall_end[0] < wall_top[0] and wall_top[1] < wall_end[1]):
        raise ValueError("the dimple needs a straight wall between its rim's and bottom's rounds")
    # The bottom is a curve through points on its arc and not an arc. A revolved arc that ends
    # on the axis becomes a spherical face, and the mesher doesn't put that face's triangles
    # in the same order every time, so the mesh files wouldn't regenerate byte for byte. The
    # curve leaves the wall in the wall's direction, reaches the axis at a right angle and
    # stays within 0.0001 mm of the arc
    steps = 8
    angles = [slope * (steps - k) / steps for k in range(1, steps)]
    bottom_points = [(bottom * math.sin(angle), centre_z + bottom * math.cos(angle))
                     for angle in angles] + [(0.0, face + depth)]
    down_wall = unit(wall_end[0] - wall_top[0], wall_end[1] - wall_top[1])
    return revolve_path((0, face - 0.01), [
        ('line', (mouth, face - 0.01)), ('line', (mouth, face)), ('arc', rim_middle, wall_top),
        ('line', wall_end), ('through', bottom_points, down_wall, (-1.0, 0.0))])


def fin() -> cq.Workplane:
    """Return one fin in the plane of the x axis and the axle: tall at the barrel, low at the hub.

    The fin stands on the spoke's flat back. Its edge leaves the barrel's inside straight
    toward the axle, turns with a round into a long straight edge down toward the hub, and
    turns with a second round toward the axle again. That round ends on the top of the fin's
    bulge on the collar. The fin starts 0.1 mm inside the spoke's plate and, where the lip is,
    0.1 mm inside the lip, so it stays out of the wedge of air in front of the lip. It ends
    inside the barrel's wall, the collar and the hub tube. At the cup it ends 0.3 mm inside
    the cup's outside, so it stays out of the bowl.
    """
    back, lip, in_lip = SPOKE_PLATE_BACK_Z - 0.1, LIP_FRONT_Z + 0.1, LIP_INNER_R + 0.2
    wall, tube, bulge_top = BARREL_LIP_D / 2 + 0.3, HUB_TUBE_D / 2 - 0.2, HUB_COLLAR_D / 2
    high, low = FIN_BARREL_END_Z, FIN_HUB_END_Z
    outer = (FIN_BARREL_CORNER_R, high)
    # The corner that the second round replaces lies a little outside the bulge's top, so that
    # the round ends on it. Found by repeating, because the long edge's direction depends on it
    inner = (bulge_top, low)
    for _ in range(5):
        along = unit(inner[0] - outer[0], inner[1] - outer[1])   # down the long straight edge
        inner = (bulge_top + FIN_ROUND * math.tan(math.acos(-along[0]) / 2), low)
    along = unit(inner[0] - outer[0], inner[1] - outer[1])
    if not (bulge_top < inner[0] < outer[0] - 2 * FIN_ROUND and outer[0] < BARREL_LIP_D / 2
            and CUP_BACK_Z < low < HUB_COLLAR_END_Z and low < high < RIM_INNER_FACE_Z):
        raise ValueError("the fin's edge must run from the barrel down to its bulge on the collar")
    # inside the cup: a quarter circle 0.3 mm inside the cup's rounded back edge
    centre = (CUP_D / 2 - CUP_BACK_ROUND, CUP_BACK_Z - CUP_BACK_ROUND)
    radius = CUP_BACK_ROUND - 0.3
    k = radius / math.sqrt(2)
    return trace_path((centre[0] + radius, back), [
        ('line', (in_lip, back)), ('line', (in_lip, lip)), ('line', (wall, lip)),
        ('line', (wall, high)),
        *fillet_corner(outer, FIN_ROUND, (1.0, 0.0), along),
        *fillet_corner(inner, FIN_ROUND, (-along[0], -along[1]), (-1.0, 0.0)),
        ('line', (tube, low)), ('line', (tube, centre[1] + radius)),
        ('line', (centre[0], centre[1] + radius)),
        ('arc', (centre[0] + k, centre[1] + k), (centre[0] + radius, centre[1]))],
        FIN_THICKNESS / 2).extrude(FIN_THICKNESS)


def bulge() -> cq.Workplane:
    """Return one of the collar's five bulges, along the x axis: the bulge where a fin starts.

    Seen along the axle, the bulge has two parallel flanks, HUB_BULGE_WIDTH apart, and a top on
    the circle of HUB_COLLAR_D. Rounds join the flanks to the top. The bulge starts 0.2 mm
    inside the cup's back and ends at HUB_COLLAR_END_Z, where its edges are rounded too. Toward
    the axle it reaches into the hub tube.
    """
    top, half, edge = HUB_COLLAR_D / 2, HUB_BULGE_WIDTH / 2, HUB_BULGE_ROUND
    inside = HUB_TUBE_D / 2 - 0.5
    if not (HUB_BULGE_END_ROUND < edge < half and HUB_TUBE_D / 2 < top - HUB_BULGE_HEIGHT):
        raise ValueError("the collar's bulges need room for their rounds")
    # the centre of the round on the bulge's +y side, and where that round meets the top
    centre = (math.sqrt((top - edge) ** 2 - (half - edge) ** 2), half - edge)
    angle = math.atan2(centre[1], centre[0])
    turn = (angle + math.pi / 2) / 2                    # the middle of that round, from its centre
    middle = (centre[0] + edge * math.cos(turn), centre[1] + edge * math.sin(turn))
    on_top = (top * math.cos(angle), top * math.sin(angle))
    start_z = CUP_BACK_Z - 0.2
    outline = (cq.Workplane('XY', origin=(0, 0, start_z)).moveTo(inside, -half)
               .lineTo(centre[0], -half)
               .threePointArc((middle[0], -middle[1]), (on_top[0], -on_top[1]))
               .threePointArc((top, 0.0), on_top)
               .threePointArc(middle, (centre[0], half))
               .lineTo(inside, half).close())
    return (outline.extrude(HUB_COLLAR_END_Z - start_z)
            .faces('>Z').edges('not <X').fillet(HUB_BULGE_END_ROUND))


def spoke() -> cq.Workplane:
    """Return one spoke with its fin, its centre line along the x axis.

    The plate is flat, with straight tapering sides, and its end comes down onto the lip in a
    round. Curved fillers round the windows' corners at the hub web. At the lip the corners
    are sharp. The plate's back is flat. A fin stands on it, along the centre line.
    """
    outline = spoke_outline()
    blends = [side_blend(outline[3], outline[2], 1), side_blend(outline[0], outline[1], -1)]
    return union_all([spoke_plate(), fin(), *blends])


def face_groove() -> List:
    """Return the path pieces of the fine groove in the outer flange's face.

    The pieces run from the groove's edge toward the tyre to its edge toward the axle. The
    groove's mouth lies in the middle of the face. From the tyre's side: a rounded edge, a
    chamfer down to the flat bottom, the bottom, a round back up toward the face, a rounded
    edge. The round's radius is the one that makes the mouth FACE_GROOVE_MOUTH wide: the
    chamfer, the bottom and the two edges' rounds take the rest.
    """
    f, depth, edge = RIM_FACE_Z, FACE_GROOVE_DEPTH, FACE_GROOVE_EDGE
    slope = math.radians(FACE_GROOVE_CHAMFER)
    # how much of the mouth's width the chamfer's side and the round's side take
    chamfer = depth / math.tan(slope) + edge * math.tan(slope / 2)
    reach = FACE_GROOVE_MOUTH - FACE_GROOVE_BOTTOM - chamfer
    radius = (reach * reach / depth + depth) / 2 - edge
    if not (reach > 0 and radius > depth - edge):
        raise ValueError("the fine groove's mouth is too narrow for its bottom, chamfer and edges")
    middle = FLANGE_D / 2 - FLANGE_FACE / 2
    mouth_outer = middle + FACE_GROOVE_MOUTH / 2        # the mouth's edge toward the tyre
    bottom_outer = mouth_outer - chamfer
    bottom_inner = bottom_outer - FACE_GROOVE_BOTTOM
    corner = (bottom_outer + depth / math.tan(slope), f)   # where the chamfer would meet the face
    # the round about (bottom_inner, depth - radius) and the edge's round about (mouth_inner,
    # edge) touch on the line between their centres
    mouth_inner = bottom_inner - reach
    turn = math.atan2(reach, radius + edge - depth)
    touch = (bottom_inner - radius * math.sin(turn), f + depth - radius + radius * math.cos(turn))
    return [
        *fillet_corner(corner, edge, (1.0, 0.0), (-math.cos(slope), math.sin(slope))),
        ('line', (bottom_outer, f + depth)), ('line', (bottom_inner, f + depth)),
        ('arc', (bottom_inner - radius * math.sin(turn / 2),
                 f + depth - radius + radius * math.cos(turn / 2)), touch),
        ('arc', (mouth_inner + edge * math.sin(turn / 2), f + edge - edge * math.cos(turn / 2)),
         (mouth_inner, f))]


def barrel() -> cq.Workplane:
    """Return the barrel: flanges, grooves and ridges, the outer face and the lip inside.

    The profile runs from the motor-side flange over the four grooves to the outer flange,
    down the outer face with its fine groove, and back along the inside: plain to the lip,
    the lip, and a cone that widens to the motor-side end. The top edge on the outer-face side
    is sharp on ridge 1 and rounded on ridges 2 and 3. The flanges' outer corners are sharp,
    their inner corners round.
    """
    f, i = RIM_FACE_Z, RIM_INNER_FACE_Z
    r_flange, r_bottom = FLANGE_D / 2, GROOVE_D / 2
    r_face, r_lip, r_in, r_end = FACE_INNER_R, LIP_INNER_R, BARREL_LIP_D / 2, BARREL_END_D / 2
    if not (r_lip < min(r_face, r_in) and r_in < r_end < r_bottom < min(RIDGE_DS) / 2
            and max(RIDGE_DS) / 2 <= r_flange and r_face < r_bottom):
        raise ValueError('the barrel needs lip < inside < grooves < ridges <= flanges')
    spans = groove_spans()
    edges = ridge_edges()
    tops = [r_flange] + [d / 2 for d in RIDGE_DS] + [r_flange]
    grooves: List = []
    for k in reversed(range(len(spans))):
        top_start, bottom_start, bottom_end, top_end = spans[k]
        motor_corner, outer_corner = (tops[k + 1], top_end), (tops[k], top_start)
        down = unit(r_bottom - tops[k + 1], bottom_end - top_end)   # down the motor-side wall
        # groove k's motor-side wall is the outer-face-side wall of the ridge behind it
        grooves += (round_corner_by(motor_corner, edges[k], (0, 1), down)
                    if k + 1 < len(spans) and edges[k] > 0 else [('line', motor_corner)])
        grooves += [('line', (r_bottom, bottom_end)), ('line', (r_bottom, bottom_start)),
                    ('line', outer_corner)]
    lip_front = f + LIP_START
    lip_back = lip_front + LIP_WIDTH
    # The cone from the lip's back ends where the round at the motor-side end begins, at
    # r_end. The round's corner on the motor-side face lies a little further out: found by
    # repeating, because the cone's slope depends on it
    corner_r = r_end
    for _ in range(4):
        slope = math.atan2(corner_r - r_in, i - lip_back)
        reach = FLANGE_INNER_ROUND / math.tan(math.pi / 4 + slope / 2)
        corner_r = r_end + reach * math.sin(slope)
    along_cone = (-math.sin(slope), -math.cos(slope))   # from the corner back up the cone
    return revolve_path((r_flange, i), [
        *grooves,
        ('line', (r_flange, f)),
        *face_groove(),
        *round_corner((r_face, f), FLANGE_INNER_ROUND, (1, 0), (0, 1)),
        ('line', (r_face, lip_front)), ('line', (r_lip, lip_front)),
        ('line', (r_lip, lip_back)), ('line', (r_in, lip_back)),
        *fillet_corner((corner_r, i), FLANGE_INNER_ROUND, along_cone, (1.0, 0.0))])


def ribs() -> cq.Workplane:
    """Return the axial ribs across the two middle grooves, at the height of ridges 2 and 3.

    Each rib runs from the middle of the first ridge's top to the middle of the third's.
    """
    spans = groove_spans()
    z0 = (spans[0][3] + spans[1][0]) / 2
    z1 = (spans[-2][3] + spans[-1][0]) / 2
    r0, r1 = GROOVE_D / 2 - 0.2, RIB_D / 2
    rib = (cq.Workplane('XY', origin=(0, 0, z0)).center((r0 + r1) / 2, 0)
           .rect(r1 - r0, RIB_WIDTH).extrude(z1 - z0))
    return union_all([rib.rotate((0, 0, 0), (0, 0, 1), 360.0 * (k + 0.5) / RIBS)
                      for k in range(RIBS)])


def hub() -> cq.Workplane:
    """Return the hub: the bowl in the spokes' face with the boss in it, the cup and the tube.

    The bowl's edge lies in the spokes' face with a small round. Its wall runs almost straight
    down to the depth of the boss's top, then rounds into the flat floor. The boss stands on
    the floor, its top below the face. Behind the face the bowl's outside (CUP_D) runs back to
    the cup's back, whose edge is rounded. The cup's back carries the collar and the tube.
    Between the collar's bulges (bulge()) the collar runs level from the cup's back for
    HUB_COLLAR_FLAT and then curves down to the tube, which it reaches at HUB_COLLAR_END_Z.
    """
    face = SPOKE_FACE_Z
    boss, edge, bowl, floor_r = BOSS_D / 2, BOWL_EDGE_D / 2, BOWL_D / 2, BOWL_FLOOR_D / 2
    boss_top, floor_z = BOSS_TOP_Z, BOWL_FLOOR_Z
    cup, tube = CUP_D / 2, HUB_TUBE_D / 2
    ring = HUB_COLLAR_D / 2 - HUB_BULGE_HEIGHT           # the collar's level part in a notch
    level_end = CUP_BACK_Z + HUB_COLLAR_FLAT
    if not boss + BOSS_FILLET < floor_r < bowl <= edge < cup:
        raise ValueError('the bowl needs boss < floor < wall <= edge < cup')
    if not face < boss_top < floor_z < CUP_BACK_Z:
        raise ValueError("the boss's top must lie between the face and the floor")
    if not boss_top < BORE_END_Z < CUP_BACK_Z:
        raise ValueError('the bore must end inside the boss, behind its top')
    if not (SPOKE_PLATE_BACK_Z <= CUP_BACK_Z - CUP_BACK_ROUND and floor_r < cup - CUP_BACK_ROUND
            and tube < ring < cup - CUP_BACK_ROUND
            and level_end < HUB_COLLAR_END_Z < HUB_TUBE_END_Z - HUB_TUBE_CHAMFER):
        raise ValueError("the cup's rounded back edge and the collar need room behind the spokes")
    # The collar's curve down to the tube is a piece of an ellipse about a point 0.15 mm inside
    # the tube. So it reaches the tube a little short of a right angle, and its surface crosses
    # the plane of the bulges' ends instead of touching it
    curve_r = tube - 0.15
    reach = math.degrees(math.acos((tube - curve_r) / (ring - curve_r)))
    curve_z = (HUB_COLLAR_END_Z - level_end) / math.sin(math.radians(reach))
    arrive = unit(-(ring - curve_r) * math.sin(math.radians(reach)),
                  curve_z * math.cos(math.radians(reach)))
    wall = unit(edge - bowl, face - boss_top)            # up the wall's straight part
    return revolve_path((0, boss_top), [
        ('line', (boss - BOSS_CHAMFER, boss_top)), ('line', (boss, boss_top + BOSS_CHAMFER)),
        *round_corner((boss, floor_z), BOSS_FILLET, (0, -1), (1, 0)),
        ('line', (floor_r, floor_z)),
        ellipse((floor_r, boss_top), bowl - floor_r, floor_z - boss_top, 90, 0,
                directions=((1.0, 0.0), wall)),
        *fillet_corner((edge, face), BOWL_EDGE_ROUND, (-wall[0], -wall[1]), (1.0, 0.0)),
        ('line', (cup, face)),
        *round_corner((cup, CUP_BACK_Z), CUP_BACK_ROUND, (0, -1), (-1, 0)),
        ('line', (ring, CUP_BACK_Z)), ('line', (ring, level_end)),
        ellipse((curve_r, level_end), ring - curve_r, curve_z, 0, reach,
                directions=((0.0, 1.0), arrive)),
        ('line', (tube, HUB_TUBE_END_Z - HUB_TUBE_CHAMFER)),
        ('line', (tube - HUB_TUBE_CHAMFER, HUB_TUBE_END_Z)), ('line', (0, HUB_TUBE_END_Z))])


def make_rim() -> cq.Workplane:
    """Return the yellow rim: barrel, ribs, hub, web, spokes, fins, bulges, bore, pins, dimples."""
    if not FLANGE_INNER_ROUND <= SPOKE_FACE_DEPTH < SPOKE_FACE_DEPTH + SPOKE_PLATE <= LIP_START:
        raise ValueError("the spokes' plate must lie between the face's round and the lip's front")
    if not SPOKE_MARK_DEPTH < SPOKE_PLATE:
        raise ValueError("the spokes' dimples must end inside the plate")
    if not WEB_R + SPOKE_ROOT_BLEND < SPOKE_END_START_R:
        raise ValueError("the spokes need a flat face between the hub's web and their end's round")
    if not BOSS_TOP_DEPTH < BOWL_DEPTH - HUB_PIN_HEIGHT:
        raise ValueError("the pins' tops must lie below the boss's top")
    # The spokes' central face: an annulus from the cup to WEB_R (the bowl stays open)
    web = (cq.Workplane('XY', origin=(0, 0, SPOKE_FACE_Z)).circle(WEB_R)
           .circle(CUP_D / 2 - 0.2).extrude(SPOKE_PLATE_BACK_Z - SPOKE_FACE_Z))
    one_spoke = spoke()
    rim = union_all([barrel(), ribs(), hub(), web,
                     *[part.rotate((0, 0, 0), (0, 0, 1), 360.0 * k / SPOKES)
                       for part in (one_spoke, bulge()) for k in range(SPOKES)]])
    bore_length = HUB_TUBE_END_Z - BORE_END_Z + 1
    bore = (cq.Workplane('XY', origin=(0, 0, BORE_END_Z)).circle(BORE_D / 2).extrude(bore_length)
            .intersect(cq.Workplane('XY', origin=(0, 0, BORE_END_Z))
                       .rect(BORE_FLATS, BORE_D + 1).extrude(bore_length)))
    rim = rim.cut(bore)
    # Small pins on the bowl's floor, a dimple with a rounded rim and bottom on each spoke
    pads = union_all([cq.Workplane('XY', origin=(HUB_PIN_R, 0, BOWL_FLOOR_Z - HUB_PIN_HEIGHT))
                      .circle(HUB_PIN_D / 2).extrude(HUB_PIN_HEIGHT + 0.01)
                      .rotate((0, 0, 0), (0, 0, 1), HUB_PIN_ANGLE + 360.0 * k / HUB_PINS)
                      for k in range(HUB_PINS)])
    dimple = spoke_dimple()
    marks = union_all([dimple.translate((SPOKE_MARK_RADIUS, 0, 0))
                       .rotate((0, 0, 0), (0, 0, 1), 360.0 * k / SPOKES)
                       for k in range(SPOKES)])
    return rim.union(pads).cut(marks)


# ---------------------------------------------------------------------------------------------
# Mesh
# ---------------------------------------------------------------------------------------------
FRAME = ("The axle along z at x = y = 0, the wheel's outer face (the rim's outer flange face, "
         "flush with the tyre's sidewall at its widest) at z = -13 mm, the rim's motor-side "
         "flange face at z = +12.00 mm and the hub tube's end at z = +15.42 mm.")
SUB_PARTS = {'rim': 'rim', 'tyre': 'tyre'}


def for_meshing(solid: cq.Workplane) -> cq.Workplane:
    """Return a solid with every face as a NURBS surface, for the mesher.

    On the faces as built, the mesher's triangles cut across the profile's small rounds: a cut
    through the axle then shows a straight piece where the round is, up to 0.09 mm inside a
    round of 0.5 mm, three times LINEAR_DEFLECTION. On NURBS surfaces the mesher keeps its
    triangles within LINEAR_DEFLECTION of the rounds.
    """
    converted = BRepBuilderAPI_NurbsConvert(solid.val().wrapped, True).Shape()
    return cq.Workplane('XY').newObject([cq.Shape.cast(converted)])


def build() -> List[Model]:
    """Return the wheel's mesh: the yellow rim and the black tyre."""
    solids = {'rim': make_rim(), 'tyre': make_tyre()}
    sub_parts = [SubPart(part, {material: tessellate_faces(for_meshing(solids[part]),
                                                           LINEAR_DEFLECTION, ANGULAR_DEFLECTION)})
                 for part, material in SUB_PARTS.items()]
    return [Model('tt_motor_wheel_65mm', TITLE, FRAME, MATERIALS, sub_parts)]
