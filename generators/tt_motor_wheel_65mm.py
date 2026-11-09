# SPDX-FileCopyrightText: 2026 ROS Mobile Robots
# SPDX-License-Identifier: BSD-3-Clause
"""Generate the 65 mm wheel for TT gear motors: yellow five-spoke rim, black block-tread tyre.

Modelled from caliper measurements of the real wheel (tyre taken off the rim), photos and video
frames of DiffBot's and Remo's wheels, shop photos and product listings. The rim is a barrel with
a flange at each end and three ridges between them, axial ribs across the two pockets, a ring on
the outer face that curves into the windows, five tapered swept spokes with a narrow web and a fin
behind each, a bowl recessed into the spokes' central face with the centre boss and four small
pins in it, and a hub tube on a collar on the motor side. The tyre is a hollow section like a car
tyre's: its beads sit inside the rim between the flanges and the outer ridges, its sidewalls rise
over the flanges and are widest halfway up, and an arched channel inside clears the barrel's
ridges. Sources in provenance/tt_motor_wheel_65mm.md.

Frame (positions in mm, the files are in metres): the axle along z. The wheel's outer face is
at z = -13 mm: the rim's outer flange face, flush with the tyre's sidewall at its widest. The rim's
motor-side flange face is at +12.00 mm and the hub tube's end at +15.42 mm.
"""
import math
from typing import List
from typing import Sequence
from typing import Tuple

import cadquery as cq

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
#   video     video frames of DiffBot's wheel
#   choice    not measured: an estimate that looks like the photos
# z is along the axle (see the module docstring). Diameters as D_, radii are derived.
# ---------------------------------------------------------------------------------------------
TYRE_OUTER_FACE_Z = -13.0    # the outer face: 13 mm outboard of both robots' wheel joints

# Tyre: a hollow section like a car tyre's (the real tyre, p03-p09). The beads at its inner
# diameter sit inside the rim, in the plain bands between the flanges and the outer ridges. The
# sidewalls rise over the flanges and are widest halfway up, flush with the flanges' faces. The
# crown is a little narrower. Inside, an arched channel spans the barrel's ridges and ribs
TYRE_OUTER_D = 66.6          # measured on DiffBot: 66.3 and 66.8 (front_wheel.yaml)
TYRE_BEAD_D = 49.4           # p07, p08: bead to tread 8.38 and 8.82 -> 66.6 - 2 x 8.6
TYRE_BEAD_WIDTH = 1.8        # choice: the bead's base in the plain band (p37 sil.: band 2.0)
TYRE_BEAD_LIP = 0.74         # p03: the bead lip's thickness where it touches the rim
# The sidewalls stand this far past the flanges' faces at their widest: flush on the real
# tyre, not measured (provenance: measurement 1)
TYRE_BULGE = 0.0             # the real tyre
TYRE_WIDEST_D = 59.2         # the real tyre: widest halfway between flange (51.22) and tread
TYRE_CROWN_WIDTH = 18.6      # choice (shop photos): the tread between the shoulders' rounds
TYRE_CROWN_DROP = 0.25       # choice (shop photos): the crown's edges below its middle
# The channel's ceiling, from the tread's top (radial, tyre off the rim). Three readings, 5.77
# (p04), 6.88 (p05) and 7.46 (p06), at places across the tyre that aren't recorded: assumed
# here in the middle (p04) and near a bead (p06) (provenance: measurement 2)
TYRE_WALL_SIDE = 7.46        # p06, near a bead: assumed
TYRE_WALL_MIDDLE = 5.77      # p04, in the middle: assumed
TYRE_CHANNEL_WIDTH = 15.6    # choice: between the beads' inner flanks, over all three ridges
TYRE_SEAT_GAP = 0.02         # choice: the tyre stands this far off the rim
TREAD_DEPTH = 0.6            # choice (shop photos; provenance: measurement 4)
TREAD_ROWS = 42              # p07, p08: about 5 mm row pitch, blocks about 4.3 mm long
TREAD_TRANSVERSE_GROOVE = 0.7  # p07, p08: narrow grooves between the blocks (width: choice)
# Circumferential grooves (offset from the tyre's centre plane, width): six block columns
# (p07, p08, video); positions and widths: choice
TREAD_CIRCUMFERENTIAL_GROOVES = ((0.0, 0.7), (4.6, 0.7), (-4.6, 0.7), (9.0, 0.7), (-9.0, 0.7))
TREAD_SHOULDER_REACH = 30.0  # radius down to which the shoulder blocks wrap onto the sidewall

# Rim barrel (from the outer flange face F towards the motor side)
RIM_WIDTH = 25.00            # p40: flange to flange
RIM_LENGTH = 28.42           # p33: the outer flange face to the hub tube's end
FLANGE_D = 51.22             # p01, p41 (51.25)
RIDGE_D = 50.9               # p37 sil.: the ridges stand 0.1 to 0.2 inside the flanges
BAND_D = 47.91               # p02: the plain bands and the pockets
BARREL_INNER_D = 45.1        # p28: 51.22 - 2 x 3.06
FLANGE_TOP = 1.8             # p37 sil.: flange 0 to 2.7 with its slope
FLANGE_SLOPE_END = 2.7       # p37 sil.
FLANGE_EDGE_ROUND = 0.4      # choice
RIDGE_CENTRES = (6.0, 12.5, 19.0)  # p37 sil.: 6.0, 12.75, 18.7; modelled symmetric
RIDGE_TOP = 0.8              # p37 sil.: top width
RIDGE_BASE = 2.7             # p37 sil.: base width
RIBS = 7                     # p33, p35: about 50 deg apart
RIB_WIDTH = 1.0              # choice (photos: thin)

# Outer face: the ring between the tyre and the windows
FACE_LINE_R = 24.0           # p41 img.: the fine line on the ring
FACE_LINE = (0.3, 0.2)       # choice: its width and depth
WINDOW_OUTER_R = 20.0        # p41 img.: where the windows start
LIP_CURL = (21.6, 0.45, 1.6)  # choice: the face curves inward: mid r, mid depth, depth at r 20
LIP_END = 2.4                # choice: depth of the ring's inner edge behind the face
LIP_BACK = 3.2               # choice: where the ring's back meets the barrel's inside

# Hub, outer face: a bowl recessed into the spokes' flat central face, its edge in that face
# (no raised ring), the boss standing in it below the face (p21, p23, p24, p41). The depths are
# estimated on the oblique photo p24, scaled with the bowl's 17.62 (provenance: measurement 3)
BOSS_D = 9.69                # p23
BOSS_CHAMFER = 0.3           # choice
BOWL_EDGE_D = 17.9           # p41 img.: the bowl's edge in the spokes' face
BOWL_D = 17.62               # p21: inside jaws in the bowl at about the boss's top
# Below that the wall rounds into the floor: p20 reads 14.07 with the jaws deeper in the bowl
BOWL_FLOOR_D = 13.8          # p41 img.: the wall's foot at r 6.9
BOWL_DEPTH = 2.8             # p24 img. (2.7 to 3.4): the floor behind the spokes' face
BOSS_TOP_DEPTH = 1.2         # p24 img. (1.0 to 2.2): the boss's top behind the spokes' face
BOWL_EDGE_ROUND = 0.2        # choice
BOSS_FILLET = 0.3            # choice: where the boss meets the floor
CUP_D = 20.64                # p24: the bowl's outside behind the face, between the spokes
CUP_BACK_DEPTH = 5.2         # choice: the cup's back on the motor side, the floor 1.2 thick
HUB_PINS = 4                 # p41 img.: four small pins on the bowl's floor, 90 deg apart
HUB_PIN_R = 6.15             # p41 img.
HUB_PIN_D = 0.9              # p41 img.
HUB_PIN_HEIGHT = 0.4         # choice
HUB_PIN_ANGLE = 42.0         # p41 img.: the first pin 42 deg after a spoke
WEB_R = 11.2                 # p41 img.: windows start at r 11.4 between the spokes; cup 10.32

# Spokes: five, tapered and swept
SPOKES = 5
SPOKE_FACE_DEPTH = 1.2       # video: behind the face
SPOKE_PLATE = 1.5            # choice: thickness of the spoke's front plate
SPOKE_WEB_WIDTH = 3.47       # p18, p19: the spoke's back, near the hub
HUB_BACK_DEPTH = 4.5         # choice: the spokes' back plane behind the face
SPOKE_ROOT_RADIUS = 9.0      # choice: the loft starts just outside the bowl's edge
SPOKE_TIP_RADIUS = 21.5      # choice: ends inside the ring's curl
# Spoke widths, a linear taper through two of p41 img.'s estimates (9.3 at r 11.7, 5.4 at r 19.5);
# the model then has 9.31, 7.35 (p41: 7.7) at r 15.6, 5.40, and 5.15 at r 20 (p25: 5.35 across the
# tip at the ring), before the bevel narrows the flat face. The sweep: video
SPOKE_ROOT_EDGES = (-5.33, 5.33)  # 10.65 at r 9
SPOKE_TIP_EDGES = (-0.6, 3.8)     # 4.4 at r 21.5, the centre swept 1.6 mm
SPOKE_BEVEL = (1.2, 0.8)     # video: trailing edge bevel at the tip: width, depth
SPOKE_EDGE_BREAK = 0.35      # video: leading edge chamfer at the tip
SPOKE_BEVEL_START = 15.0     # the bevel starts past the root blends
SPOKE_BEVEL_MIN = 0.03       # bevel scale up to there (keeps the loft's corners distinct)
SPOKE_ROOT_BLEND = 3.0       # video, p41: curved sides into the web; radius: choice
SPOKE_MARK_RADIUS = 15.5     # video, p41: one round mark per spoke
SPOKE_MARK_D = 1.1           # choice
SPOKE_MARK_DEPTH = 0.4       # choice

# Motor side
HUB_COLLAR_D = 15.27         # p15: the collar at the tube's root
HUB_COLLAR_HEIGHT = 2.0      # choice
HUB_COLLAR_CHAMFER = 0.5     # choice
HUB_TUBE_D = 9.95            # p29, p30: 9.97 and 9.92
HUB_TUBE_FILLET = 1.0        # choice
HUB_TUBE_CHAMFER = 0.5       # choice
BORE_D = 5.44                # p14: the round side; p12, p13 tube wall 2.10 and 2.34
BORE_FLATS = 3.66            # listings (X2 Robotics, VXB); not measured
BORE_BOTTOM_DEPTH = 3.2      # choice: blind, the boss's top is closed (photos)
FIN_THICKNESS = 1.51         # p31
FIN_TOP_DEPTH = 15.26        # video, p32: the fins reach about half way up the tube
FIN_OUTER_R = 20.0           # p32: the fins run out along the spokes

# Derived positions
RIM_FACE_Z = TYRE_OUTER_FACE_Z + TYRE_BULGE   # F: the outer flange face, -13.0
RIM_INNER_FACE_Z = RIM_FACE_Z + RIM_WIDTH      # the motor-side flange face, 12.0
HUB_TUBE_END_Z = RIM_FACE_Z + RIM_LENGTH       # 15.42
TYRE_INNER_FACE_Z = RIM_INNER_FACE_Z + TYRE_BULGE  # the tyre's motor side at its widest, 12.0
TYRE_CENTRE_Z = RIM_FACE_Z + RIM_WIDTH / 2     # -0.5
FIN_TOP_Z = RIM_FACE_Z + FIN_TOP_DEPTH
SPOKE_FACE_Z = RIM_FACE_Z + SPOKE_FACE_DEPTH
SPOKE_PLATE_BACK_Z = SPOKE_FACE_Z + SPOKE_PLATE
HUB_BACK_Z = RIM_FACE_Z + HUB_BACK_DEPTH
CUP_BACK_Z = RIM_FACE_Z + CUP_BACK_DEPTH

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


def ellipse(centre: Tuple[float, float], radius_r: float, radius_z: float,
            start: float, end: float, count: int = 12) -> Tuple:
    """Return a ('through', points) piece along a quarter ellipse, from angle start to end (deg).

    The start point (at angle start) is the path's current point and isn't repeated.
    """
    points = []
    for k in range(1, count + 1):
        angle = math.radians(start + (end - start) * k / count)
        points.append((centre[0] + radius_r * math.cos(angle),
                       centre[1] + radius_z * math.sin(angle)))
    return ('through', points)


def unit(dx: float, dz: float) -> Tuple[float, float]:
    """Return the direction (dx, dz) with length 1."""
    length = math.hypot(dx, dz)
    return dx / length, dz / length


def revolve_path(start: Tuple[float, float], pieces: Sequence) -> cq.Workplane:
    """Revolve a closed (r, z) path about the z axis.

    The pieces are ('line', end), ('arc', middle, end), ('spline', end, start_tangent,
    end_tangent) or ('through', points), a spline through the points, all in (r, z).
    """
    wp = cq.Workplane('XZ').moveTo(*start)
    for piece in pieces:
        if piece[0] == 'line':
            wp = wp.lineTo(*piece[1])
        elif piece[0] == 'arc':
            wp = wp.threePointArc(piece[1], piece[2])
        elif piece[0] == 'through':
            wp = wp.spline(list(piece[1]), includeCurrent=True)
        else:
            wp = wp.spline([piece[1]], tangents=[piece[2], piece[3]], includeCurrent=True)
    return wp.close().revolve(360, (0, 0, 0), (0, 1, 0))


def union_all(shapes: Sequence[cq.Workplane]) -> cq.Workplane:
    """Return the union of the shapes."""
    result = shapes[0]
    for shape in shapes[1:]:
        result = result.union(shape)
    return result


def plain_bands() -> List[Tuple[float, float]]:
    """Return the rim's two plain bands (z from, z to): between each flange and its ridge."""
    f = RIM_FACE_Z
    half_base = RIDGE_BASE / 2
    return [(f + FLANGE_SLOPE_END, f + RIDGE_CENTRES[0] - half_base),
            (f + RIDGE_CENTRES[-1] + half_base, f + RIM_WIDTH - FLANGE_SLOPE_END)]


def tyre_outer_pieces(inset: float) -> List:
    """Return the tyre's outer contour from the outer foot to the motor-side foot, as path pieces.

    The contour starts at the foot on the outer flange's top (the path's start point, see
    tyre_foot). Each sidewall bulges from its foot to its widest point and rounds over the
    shoulder to the crown, quarter ellipses, so the widest point is exactly the outermost.
    With an inset, the feet, the widest points and the crown move inwards by it (the tread's
    grooves are cut outside the inset copy).
    """
    outer = TYRE_OUTER_D / 2 - inset
    wide = TYRE_WIDEST_D / 2
    z_out = RIM_FACE_Z - TYRE_BULGE + inset
    z_in = RIM_INNER_FACE_Z + TYRE_BULGE - inset
    half = TYRE_CROWN_WIDTH / 2
    edge = outer - TYRE_CROWN_DROP
    foot_r, foot_out = tyre_foot(inset)
    foot_in = RIM_INNER_FACE_Z - FLANGE_EDGE_ROUND - inset
    c = TYRE_CENTRE_Z
    sidewall_r, shoulder_r = wide - foot_r, edge - wide
    return [
        ellipse((wide, foot_out), sidewall_r, foot_out - z_out, 180, 270),
        ellipse((wide, c - half), shoulder_r, c - half - z_out, 270, 360),
        ('arc', (outer, c), (edge, c + half)),
        ellipse((wide, c + half), shoulder_r, z_in - c - half, 0, 90),
        ellipse((wide, foot_in), sidewall_r, z_in - foot_in, 90, 180)]


def tyre_foot(inset: float) -> Tuple[float, float]:
    """Return where the tyre's outer sidewall meets the outer flange's top (moved by inset)."""
    return (FLANGE_D / 2 + TYRE_SEAT_GAP + inset, RIM_FACE_Z + FLANGE_EDGE_ROUND + inset)


def tyre_envelope(inset: float) -> cq.Workplane:
    """Return the region inside the tyre's outer contour (moved inwards by inset)."""
    return revolve_path(tyre_foot(inset), tyre_outer_pieces(inset))


def bead_toe_z(face_z: float, towards: int) -> float:
    """Return z where a bead's toe meets the flange's sloping inside (towards: +1 or -1)."""
    r_flange, r_band, r_bead = FLANGE_D / 2, BAND_D / 2, TYRE_BEAD_D / 2
    along = (r_flange - r_bead) / (r_flange - r_band) * (FLANGE_SLOPE_END - FLANGE_TOP)
    return face_z + towards * (FLANGE_TOP + along + TYRE_SEAT_GAP)


def tyre_body() -> cq.Workplane:
    """Return the tyre without tread grooves: beads in the rim, sidewalls, crown and channel.

    Each bead's base sits in a plain band next to its flange, held between the flange and the
    outer ridge (its diameter is less than both). At the base's inner end the bead lip, as thick
    as measured, rises to the channel's side, and the channel's ceiling arches over the three
    ridges.
    """
    r_seat, r_bead = FLANGE_D / 2 + TYRE_SEAT_GAP, TYRE_BEAD_D / 2
    f, i, c = RIM_FACE_Z, RIM_INNER_FACE_Z, TYRE_CENTRE_Z
    toe_out, toe_in = bead_toe_z(f, 1), bead_toe_z(i, -1)
    heel_out, heel_in = toe_out + TYRE_BEAD_WIDTH, toe_in - TYRE_BEAD_WIDTH
    side = TYRE_OUTER_D / 2 - TYRE_WALL_SIDE
    apex = TYRE_OUTER_D / 2 - TYRE_WALL_MIDDLE
    corner_out = (side, c - TYRE_CHANNEL_WIDTH / 2)
    corner_in = (side, c + TYRE_CHANNEL_WIDTH / 2)
    if not RIDGE_D / 2 + TYRE_SEAT_GAP < side < apex:
        raise ValueError("the channel's ceiling must clear the ridges and rise to the middle")
    lip = r_bead + TYRE_BEAD_LIP
    if not lip < side:
        raise ValueError("the bead lip must stay below the channel's ceiling")
    ridge_flank = f + RIDGE_CENTRES[0] - RIDGE_BASE / 2
    if heel_out > ridge_flank or corner_out[1] > f + RIDGE_CENTRES[0] - RIDGE_TOP / 2:
        raise ValueError('the bead and the channel must stay outside the outer ridge')
    return revolve_path(tyre_foot(0.0), [
        *tyre_outer_pieces(0.0),
        ('line', (r_seat, i - FLANGE_TOP)),
        ('line', (r_bead, toe_in)), ('line', (r_bead, heel_in)), ('line', (lip, heel_in)),
        ('line', corner_in),
        ellipse((side, c), apex - side, TYRE_CHANNEL_WIDTH / 2, 90, -90, 24),
        ('line', (lip, heel_out)), ('line', (r_bead, heel_out)), ('line', (r_bead, toe_out)),
        ('line', (r_seat, f + FLANGE_TOP))])


def tread_cutter() -> cq.Workplane:
    """Return the grooves: circumferential rings and transverse slabs, one tread depth deep."""
    reach = TYRE_OUTER_D / 2 + 5
    centre = TYRE_CENTRE_Z
    width = TYRE_INNER_FACE_Z - TYRE_OUTER_FACE_Z
    rings = union_all([
        revolve_profile([(TREAD_SHOULDER_REACH + 1.0, centre + off - w / 2),
                         (reach, centre + off - w / 2), (reach, centre + off + w / 2),
                         (TREAD_SHOULDER_REACH + 1.0, centre + off + w / 2)])
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


def lerp(a: float, b: float, t: float) -> float:
    """Return the value a fraction t of the way from a to b."""
    return a + (b - a) * t


def spoke_edges(radius: float) -> Tuple[float, float]:
    """Return a spoke's side edges (tangential positions) at a radius (unrotated spoke)."""
    t = (radius - SPOKE_ROOT_RADIUS) / (SPOKE_TIP_RADIUS - SPOKE_ROOT_RADIUS)
    return (lerp(SPOKE_ROOT_EDGES[0], SPOKE_TIP_EDGES[0], t),
            lerp(SPOKE_ROOT_EDGES[1], SPOKE_TIP_EDGES[1], t))


def bevel_scale(radius: float) -> float:
    """Return the size of the spoke's bevel and chamfer at a radius, 1 at the tip.

    The bevel starts after the root blends, so the blends meet a flat spoke face.
    """
    if radius <= SPOKE_BEVEL_START:
        return SPOKE_BEVEL_MIN
    t = (radius - SPOKE_BEVEL_START) / (SPOKE_TIP_RADIUS - SPOKE_BEVEL_START)
    return lerp(SPOKE_BEVEL_MIN, 1.0, t)


def spoke_centre(radius: float) -> float:
    """Return the tangential position of a spoke's face centre at a radius (unrotated spoke)."""
    lo, hi = spoke_edges(radius)
    s = bevel_scale(radius)
    return (lo + SPOKE_BEVEL[0] * s + hi - SPOKE_EDGE_BREAK * s) / 2


def root_blend(start: Tuple[float, float], end: Tuple[float, float], side: int) -> cq.Workplane:
    """Return the filler that rounds the corner between a spoke's side and the hub web.

    start and end are the spoke side's ends (unrotated spoke), side +1 or -1 the window's side
    of that edge. The filler is the region between the corner, the edge, a concave arc of
    radius SPOKE_ROOT_BLEND tangent to edge and web, and the web, as deep as the spoke's plate.
    Its top is the spoke's face, so the arc must end before the bevel starts.
    """
    ax, ay = start
    length = math.hypot(end[0] - ax, end[1] - ay)
    dx, dy = (end[0] - ax) / length, (end[1] - ay) / length
    nx, ny = (-dy, dx) if side > 0 else (dy, -dx)
    r, web = SPOKE_ROOT_BLEND, WEB_R

    def along(px: float, py: float, radius: float) -> float:
        """Return t where the line (px, py) + t (dx, dy) reaches the radius (outer root)."""
        b = px * dx + py * dy
        return -b + math.sqrt(b * b - (px * px + py * py - radius * radius))

    t = along(ax + r * nx, ay + r * ny, web + r)
    cx, cy = ax + r * nx + t * dx, ay + r * ny + t * dy
    t1 = (cx - r * nx, cy - r * ny)
    c = math.hypot(cx, cy)
    t2 = (cx * web / c, cy * web / c)
    u = along(ax, ay, web)
    corner = (ax + u * dx, ay + u * dy)
    mx, my = t1[0] + t2[0] - 2 * cx, t1[1] + t2[1] - 2 * cy
    m = math.hypot(mx, my)
    middle = (cx + r * mx / m, cy + r * my / m)
    if math.hypot(*t1) > SPOKE_BEVEL_START:
        raise ValueError('the root blend reaches past SPOKE_BEVEL_START')
    return (cq.Workplane('XY', origin=(0, 0, SPOKE_FACE_Z)).moveTo(*corner).lineTo(*t1)
            .threePointArc(middle, t2).close().extrude(SPOKE_PLATE_BACK_Z - SPOKE_FACE_Z))


def spoke(angle: float) -> cq.Workplane:
    """Return one spoke with its web and fin, rotated by angle (deg) about the axle.

    The spoke's front plate is lofted through its cross-sections at the hub, where the bevel
    starts and at the tip: a trapezoid with a bevel along the trailing edge and a small chamfer
    along the leading edge, both growing from the bevel's start to the tip. Behind the plate a
    narrow web runs along the spoke's centre line, and a fin from the web up the hub tube.
    """
    def section(radius: float) -> List[Tuple[float, float]]:
        lo, hi = spoke_edges(radius)
        s = bevel_scale(radius)
        width, depth = SPOKE_BEVEL[0] * s, SPOKE_BEVEL[1] * s
        e = SPOKE_EDGE_BREAK * s
        back = SPOKE_PLATE_BACK_Z
        return [(lo, back), (hi, back), (hi, SPOKE_FACE_Z + e),
                (hi - e, SPOKE_FACE_Z), (lo + width, SPOKE_FACE_Z), (lo, SPOKE_FACE_Z + depth)]

    plate = (cq.Workplane('YZ', origin=(SPOKE_ROOT_RADIUS, 0, 0))
             .polyline(section(SPOKE_ROOT_RADIUS)).close()
             .workplane(offset=SPOKE_BEVEL_START - SPOKE_ROOT_RADIUS)
             .polyline(section(SPOKE_BEVEL_START)).close()
             .workplane(offset=SPOKE_TIP_RADIUS - SPOKE_BEVEL_START)
             .polyline(section(SPOKE_TIP_RADIUS)).close()
             .loft(ruled=True))
    # The web and the fin follow the spoke's centre line, which leans with the sweep
    tip_centre = sum(SPOKE_TIP_EDGES) / 2
    lean = math.degrees(math.atan2(tip_centre, SPOKE_TIP_RADIUS - SPOKE_ROOT_RADIUS))
    web = (cq.Workplane('XY', origin=(0, 0, SPOKE_PLATE_BACK_Z - 0.1))
           .center((SPOKE_ROOT_RADIUS + FIN_OUTER_R) / 2, 0)
           .rect(FIN_OUTER_R - SPOKE_ROOT_RADIUS, SPOKE_WEB_WIDTH)
           .extrude(HUB_BACK_Z - SPOKE_PLATE_BACK_Z + 0.1))
    fin = (cq.Workplane('XZ', origin=(0, FIN_THICKNESS / 2, 0))
           .polyline([(HUB_TUBE_D / 2 - 0.2, HUB_BACK_Z - 0.2),
                      (FIN_OUTER_R, HUB_BACK_Z - 0.2),
                      (HUB_TUBE_D / 2 - 0.2, FIN_TOP_Z)]).close().extrude(FIN_THICKNESS))
    web = web.union(fin).rotate((SPOKE_ROOT_RADIUS, 0, 0), (SPOKE_ROOT_RADIUS, 0, 1), lean)
    blends = [root_blend((SPOKE_ROOT_RADIUS, SPOKE_ROOT_EDGES[i]),
                         (SPOKE_TIP_RADIUS, SPOKE_TIP_EDGES[i]), side)
              for i, side in ((0, -1), (1, 1))]
    return union_all([plate, web, *blends]).rotate((0, 0, 0), (0, 0, 1), angle)


def barrel() -> cq.Workplane:
    """Return the barrel with its flanges, three ridges and the ring on the outer face."""
    f, i = RIM_FACE_Z, RIM_INNER_FACE_Z
    r_in, r_band = BARREL_INNER_D / 2, BAND_D / 2
    r_flange, r_ridge = FLANGE_D / 2, RIDGE_D / 2
    e = FLANGE_EDGE_ROUND
    ridges = []
    for c in reversed(RIDGE_CENTRES):
        ridges += [('line', (r_band, f + c + RIDGE_BASE / 2)),
                   ('line', (r_ridge, f + c + RIDGE_TOP / 2)),
                   ('line', (r_ridge, f + c - RIDGE_TOP / 2)),
                   ('line', (r_band, f + c - RIDGE_BASE / 2))]
    line_mid, (line_w, line_d) = FACE_LINE_R, FACE_LINE
    mid_r, mid_depth, end_depth = LIP_CURL
    return revolve_path((r_in, i), [
        *round_corner((r_flange, i), e, (-1, 0), (0, -1)),
        ('line', (r_flange, i - FLANGE_TOP)), ('line', (r_band, i - FLANGE_SLOPE_END)),
        *ridges,
        ('line', (r_band, f + FLANGE_SLOPE_END)), ('line', (r_flange, f + FLANGE_TOP)),
        *round_corner((r_flange, f), e, (0, 1), (-1, 0)),
        ('line', (line_mid + line_w / 2, f)), ('line', (line_mid + line_w / 3, f + line_d)),
        ('line', (line_mid - line_w / 3, f + line_d)), ('line', (line_mid - line_w / 2, f)),
        ('arc', (mid_r, f + mid_depth), (WINDOW_OUTER_R, f + end_depth)),
        ('line', (WINDOW_OUTER_R, f + LIP_END)), ('line', (r_in, f + LIP_BACK))])


def ribs() -> cq.Workplane:
    """Return the axial ribs across the pockets, from ridge 1 to ridge 3, at the ridges' height."""
    z0 = RIM_FACE_Z + RIDGE_CENTRES[0]
    z1 = RIM_FACE_Z + RIDGE_CENTRES[-1]
    r0, r1 = BAND_D / 2 - 0.2, RIDGE_D / 2
    rib = (cq.Workplane('XY', origin=(0, 0, z0)).center((r0 + r1) / 2, 0)
           .rect(r1 - r0, RIB_WIDTH).extrude(z1 - z0))
    return union_all([rib.rotate((0, 0, 0), (0, 0, 1), 360.0 * (k + 0.5) / RIBS)
                      for k in range(RIBS)])


def hub() -> cq.Workplane:
    """Return the hub: the bowl in the spokes' face with the boss in it, the cup, collar and tube.

    The bowl's edge lies in the spokes' face with a small round. Its wall runs almost straight
    down to the depth of the boss's top, then rounds into the flat floor. The boss stands on
    the floor, its top below the face. Behind the face the bowl's outside (CUP_D) runs back to
    the cup's back, which carries the collar and the tube.
    """
    face = SPOKE_FACE_Z
    boss, edge, bowl, floor_r = BOSS_D / 2, BOWL_EDGE_D / 2, BOWL_D / 2, BOWL_FLOOR_D / 2
    boss_top, floor_z = face + BOSS_TOP_DEPTH, face + BOWL_DEPTH
    cup, collar, tube = CUP_D / 2, HUB_COLLAR_D / 2, HUB_TUBE_D / 2
    collar_top = CUP_BACK_Z + HUB_COLLAR_HEIGHT
    if not boss + BOSS_FILLET < floor_r < bowl <= edge < cup:
        raise ValueError('the bowl needs boss < floor < wall <= edge < cup')
    if not face < boss_top < floor_z < CUP_BACK_Z:
        raise ValueError("the boss's top must lie between the face and the floor")
    if not boss_top + 0.5 < RIM_FACE_Z + BORE_BOTTOM_DEPTH < CUP_BACK_Z:
        raise ValueError('the bore must end inside the boss, behind its top')
    wall = unit(edge - bowl, face - boss_top)            # up the wall's straight part
    return revolve_path((0, boss_top), [
        ('line', (boss - BOSS_CHAMFER, boss_top)), ('line', (boss, boss_top + BOSS_CHAMFER)),
        *round_corner((boss, floor_z), BOSS_FILLET, (0, -1), (1, 0)),
        ('line', (floor_r, floor_z)),
        ellipse((floor_r, boss_top), bowl - floor_r, floor_z - boss_top, 90, 0),
        *fillet_corner((edge, face), BOWL_EDGE_ROUND, (-wall[0], -wall[1]), (1.0, 0.0)),
        ('line', (cup, face)), ('line', (cup, CUP_BACK_Z)), ('line', (collar, CUP_BACK_Z)),
        ('line', (collar, collar_top - HUB_COLLAR_CHAMFER)),
        ('line', (collar - HUB_COLLAR_CHAMFER, collar_top)),
        *round_corner((tube, collar_top), HUB_TUBE_FILLET, (1, 0), (0, 1)),
        ('line', (tube, HUB_TUBE_END_Z - HUB_TUBE_CHAMFER)),
        ('line', (tube - HUB_TUBE_CHAMFER, HUB_TUBE_END_Z)), ('line', (0, HUB_TUBE_END_Z))])


def make_rim() -> cq.Workplane:
    """Return the yellow rim: barrel, ribs, ring, hub, web, spokes, fins, bore and marks."""
    if SPOKE_TIP_RADIUS >= BARREL_INNER_D / 2 or SPOKE_TIP_RADIUS <= WINDOW_OUTER_R:
        raise ValueError("the spokes' tips must end inside the ring's curl")
    # The spokes' central face: an annulus from the cup to WEB_R (the bowl stays open)
    web = (cq.Workplane('XY', origin=(0, 0, SPOKE_FACE_Z)).circle(WEB_R)
           .circle(CUP_D / 2 - 0.2).extrude(SPOKE_PLATE_BACK_Z - SPOKE_FACE_Z))
    rim = union_all([barrel(), ribs(), hub(), web,
                     *[spoke(360.0 * k / SPOKES) for k in range(SPOKES)]])
    bottom = RIM_FACE_Z + BORE_BOTTOM_DEPTH
    bore = (cq.Workplane('XY', origin=(0, 0, bottom)).circle(BORE_D / 2)
            .extrude(HUB_TUBE_END_Z - bottom + 1)
            .intersect(cq.Workplane('XY', origin=(0, 0, bottom))
                       .rect(BORE_FLATS, BORE_D + 1).extrude(HUB_TUBE_END_Z - bottom + 1)))
    rim = rim.cut(bore)
    # Small pins on the bowl's floor, a recessed mark on each spoke
    floor_z = SPOKE_FACE_Z + BOWL_DEPTH
    pads = union_all([cq.Workplane('XY', origin=(HUB_PIN_R, 0, floor_z - HUB_PIN_HEIGHT))
                      .circle(HUB_PIN_D / 2).extrude(HUB_PIN_HEIGHT + 0.01)
                      .rotate((0, 0, 0), (0, 0, 1), HUB_PIN_ANGLE + 360.0 * k / HUB_PINS)
                      for k in range(HUB_PINS)])
    marks = union_all([cq.Workplane('XY', origin=(SPOKE_MARK_RADIUS,
                                                  spoke_centre(SPOKE_MARK_RADIUS),
                                                  SPOKE_FACE_Z - 0.01))
                       .circle(SPOKE_MARK_D / 2).extrude(SPOKE_MARK_DEPTH + 0.01)
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


def build() -> List[Model]:
    """Return the wheel's mesh: the yellow rim and the black tyre."""
    solids = {'rim': make_rim(), 'tyre': make_tyre()}
    sub_parts = [SubPart(part, {material: tessellate_faces(solids[part], LINEAR_DEFLECTION,
                                                           ANGULAR_DEFLECTION)})
                 for part, material in SUB_PARTS.items()]
    return [Model('tt_motor_wheel_65mm', TITLE, FRAME, MATERIALS, sub_parts)]
