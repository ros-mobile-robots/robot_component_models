# SPDX-FileCopyrightText: 2026 ROS Mobile Robots
# SPDX-License-Identifier: BSD-3-Clause
"""Generate the 65 mm wheel for TT gear motors: yellow five-spoke rim, black block-tread tyre.

Modelled from caliper measurements of the real wheel (tyre taken off the rim), photos and video
frames of DiffBot's and Remo's wheels, shop photos and product listings. The rim is a barrel with
a flange at each end, four grooves and three ridges between them, and axial ribs across the two
middle grooves. The outer flange's face is flat with a fine groove. Inside, a lip stands inward
behind the outer face. The lip, five flat tapered spokes and the hub's face are one plate: the
spokes' face is the lip's front, and each spoke's sides curve into the lip. Behind each spoke
are a narrow web and a fin. A bowl is recessed into the spokes' central face, with the centre
boss and four small pins in it, and a hub tube stands on a collar on the motor side. The tyre
is a hollow section like a car tyre's: its beads sit inside the rim in the grooves next to the
flanges, its sidewalls rise over the flanges and are widest halfway up, and an arched channel
inside clears the barrel's ridges. Sources in provenance/tt_motor_wheel_65mm.md.

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
#   tip photo a close photo across the outer face: the spokes' tips and the lip
#   video     video frames of DiffBot's wheel
#   measured  caliper measurement of the real wheel, without a photo
#   derived   follows from other values in this file
#   choice    not measured: an estimate that looks like the photos
# z is along the axle (see the module docstring). Diameters as D_, radii are derived.
# ---------------------------------------------------------------------------------------------
TYRE_OUTER_FACE_Z = -13.0    # the outer face: 13 mm outboard of both robots' wheel joints

# Tyre: a hollow section like a car tyre's (the real tyre, p03-p09). The beads at its inner
# diameter sit inside the rim, in the grooves between the flanges and the outer ridges. The
# sidewalls rise over the flanges and are widest halfway up, flush with the flanges' faces. The
# crown is a little narrower. Inside, an arched channel spans the barrel's ridges and ribs
TYRE_OUTER_D = 66.6          # measured on DiffBot: 66.3 and 66.8 (front_wheel.yaml)
TYRE_BEAD_D = 49.4           # p07, p08: bead to tread 8.38 and 8.82 -> 66.6 - 2 x 8.6
TYRE_BEAD_WIDTH = 1.8        # choice: the bead's base, in a groove 3.9 wide
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
TYRE_FOOT_INSET = 0.4        # choice: the sidewalls leave the flanges' tops behind the faces
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
GROOVE_D = 47.91             # p02: the grooves' bottoms
BARREL_INNER_D = 45.1        # p28: 51.22 - 2 x 3.06, at the motor-side end
# The four grooves between the flanges and the three ridges, from the outer flange: the opening
# between the tops, the bottom's width, and how far the bottom starts behind the opening on the
# outer-face side (0: that wall is vertical). The motor-side wall takes the rest of the slope
BARREL_GROOVES = (
    (3.9, 3.9, 0.0),         # measured: a rectangle
    (5.8, 4.1, 0.85),        # measured: the widths; choice: both walls slope the same
    (4.8, 3.9, 0.0),         # measured: the outer-face wall vertical, the other one sloped
    (3.9, 3.9, 0.0))         # measured: a rectangle
FLANGE_TOPS = (1.8, 2.2)     # p37 sil.: the outer and the motor-side flange's top widths
# derived: the three ridges' tops share what the grooves and the flanges leave of the width
RIDGE_TOP = (RIM_WIDTH - sum(FLANGE_TOPS) - sum(groove[0] for groove in BARREL_GROOVES)) / 3
RIBS = 7                     # p33, p35: about 50 deg apart
RIB_WIDTH = 1.0              # choice (photos: thin)

# Outer face: the flange's flat face with a fine groove. Both flanges' outer corners are sharp,
# their inner corners are round (measured: the shapes; the radii: choice)
FLANGE_FACE = 3.2            # measured: the face's radial height, so 44.82 inside at the face
FACE_GROOVE = (1.3, 0.6)     # measured: its distance from the outer edge and its width
FACE_GROOVE_DEPTH = 0.2      # choice
FLANGE_INNER_ROUND = 0.5     # choice: the round into the barrel's inside, at both ends

# Barrel inside, from the outer face: plain, a lip that stands inward, plain to the motor side
LIP_START = 3.5              # measured: the lip's front behind the outer face
LIP_WIDTH = 1.7              # measured: along the axle (19.8 remain to the motor-side face)
LIP_INNER_D = 40.5           # p41 img., p25: the windows' outer edge at r 20.2 to 20.4

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
HUB_PIN_R = 6.15             # p41 img.
HUB_PIN_D = 0.9              # p41 img.
HUB_PIN_HEIGHT = 0.4         # choice
HUB_PIN_ANGLE = 42.0         # p41 img.: the first pin 42 deg after a spoke
WEB_R = 11.2                 # p41 img.: windows start at r 11.4 between the spokes; cup 10.32

# Spokes: five, flat and tapered, each symmetric about a radial centre line (p41 img.). The
# spokes, the lip and the hub's face are one plate: the spokes' flat face lies in the lip's
# front, and the plate is as thick as the lip is wide (tip photo). Behind each spoke a narrow web
SPOKES = 5
SPOKE_WEB_WIDTH = 3.47       # p18, p19: the spoke's back, near the hub
SPOKE_WEB_DEPTH = 1.8        # choice: how far the web stands behind the plate
SPOKE_ROOT_RADIUS = 9.6      # choice: the plate starts inside the cup's wall
# Spoke widths across the flat face: a linear taper between two places
SPOKE_ROOT_WIDTH = (13.2, 8.9)  # p41 img.: the radius and the width there, near the hub
SPOKE_TIP_WIDTH = 5.35       # p25: across the tip; choice: where the spoke runs into the lip
SPOKE_TIP_BLEND = 0.5        # tip photo: the sides curve into the lip; radius: choice
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
FIN_THICKNESS = 1.51         # p31
FIN_TOP_DEPTH = 18.0         # choice (p32): the fins reach about 40 % up the tube

# Derived positions
RIM_FACE_Z = TYRE_OUTER_FACE_Z + TYRE_BULGE   # F: the outer flange face, -13.0
RIM_INNER_FACE_Z = RIM_FACE_Z + RIM_WIDTH      # the motor-side flange face, 12.0
HUB_TUBE_END_Z = RIM_FACE_Z + RIM_LENGTH       # 15.42
TYRE_INNER_FACE_Z = RIM_INNER_FACE_Z + TYRE_BULGE  # the tyre's motor side at its widest, 12.0
TYRE_CENTRE_Z = RIM_FACE_Z + RIM_WIDTH / 2     # -0.5
FIN_TOP_Z = RIM_FACE_Z + FIN_TOP_DEPTH
FACE_INNER_R = FLANGE_D / 2 - FLANGE_FACE      # the barrel's inside at the outer face, 22.41
SPOKE_FACE_Z = RIM_FACE_Z + LIP_START          # the spokes' face: the lip's front
SPOKE_PLATE_BACK_Z = SPOKE_FACE_Z + LIP_WIDTH  # the plate's back: the lip's back
HUB_BACK_Z = SPOKE_PLATE_BACK_Z + SPOKE_WEB_DEPTH  # the webs' back plane
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


def groove_spans() -> List[Tuple[float, float, float, float]]:
    """Return each barrel groove's z positions: top start, bottom start, bottom end, top end.

    The grooves follow each other from the outer flange's top to the motor-side flange's top,
    with a ridge's top between two grooves.
    """
    spans = []
    z = RIM_FACE_Z + FLANGE_TOPS[0]
    for top, bottom, lead in BARREL_GROOVES:
        spans.append((z, z + lead, z + lead + bottom, z + top))
        z += top + RIDGE_TOP
    return spans


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
    foot_in = RIM_INNER_FACE_Z - TYRE_FOOT_INSET - inset
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
    return (FLANGE_D / 2 + TYRE_SEAT_GAP + inset, RIM_FACE_Z + TYRE_FOOT_INSET + inset)


def tyre_envelope(inset: float) -> cq.Workplane:
    """Return the region inside the tyre's outer contour (moved inwards by inset)."""
    return revolve_path(tyre_foot(inset), tyre_outer_pieces(inset))


def tyre_body() -> cq.Workplane:
    """Return the tyre without tread grooves: beads in the rim, sidewalls, crown and channel.

    Each bead's base sits in the groove next to its flange, held between the flange and the
    outer ridge (its diameter is less than both), its toe at the flange's wall. At the base's
    inner end the bead lip, as thick as measured, rises to the channel's side, and the
    channel's ceiling arches over the three ridges.
    """
    r_seat, r_bead = FLANGE_D / 2 + TYRE_SEAT_GAP, TYRE_BEAD_D / 2
    f, i, c = RIM_FACE_Z, RIM_INNER_FACE_Z, TYRE_CENTRE_Z
    toe_out = f + FLANGE_TOPS[0] + TYRE_SEAT_GAP
    toe_in = i - FLANGE_TOPS[1] - TYRE_SEAT_GAP
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
    if not GROOVE_D / 2 < r_bead < RIDGE_D / 2:
        raise ValueError('the beads must sit in the grooves, held by the flanges and ridges')
    outer_groove, motor_groove = groove_spans()[0], groove_spans()[-1]
    if heel_out > outer_groove[2] or heel_in < motor_groove[1]:
        raise ValueError("the beads must fit on their grooves' bottoms")
    if corner_out[1] > outer_groove[3] or corner_in[1] < motor_groove[0]:
        raise ValueError("the channel's sides must stay over the grooves next to the flanges")
    return revolve_path(tyre_foot(0.0), [
        *tyre_outer_pieces(0.0),
        ('line', (r_seat, toe_in)),
        ('line', (r_bead, toe_in)), ('line', (r_bead, heel_in)), ('line', (lip, heel_in)),
        ('line', corner_in),
        ellipse((side, c), apex - side, TYRE_CHANNEL_WIDTH / 2, 90, -90, 24),
        ('line', (lip, heel_out)), ('line', (r_bead, heel_out)), ('line', (r_bead, toe_out)),
        ('line', (r_seat, toe_out))])


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


def spoke_outline() -> List[Tuple[float, float]]:
    """Return a spoke's flat face as a polygon: straight sides, from the cup into the lip.

    The spoke's centre line is the x axis. Each side is the straight line through the points
    where the spoke has its two measured widths: on the circle of SPOKE_ROOT_WIDTH's radius
    and on the lip's inner edge. The polygon starts inside the cup's wall and ends inside the
    lip.
    """
    (r0, w0), (r1, w1) = SPOKE_ROOT_WIDTH, (LIP_INNER_D / 2, SPOKE_TIP_WIDTH)
    x0, y0 = math.sqrt(r0 * r0 - w0 * w0 / 4), w0 / 2
    x1, y1 = math.sqrt(r1 * r1 - w1 * w1 / 4), w1 / 2
    slope = (y1 - y0) / (x1 - x0)
    root, tip = SPOKE_ROOT_RADIUS, (LIP_INNER_D / 2 + FACE_INNER_R) / 2
    half_root, half_tip = y0 + slope * (root - x0), y0 + slope * (tip - x0)
    return [(root, -half_root), (tip, -half_tip), (tip, half_tip), (root, half_root)]


def side_blend(start: Tuple[float, float], end: Tuple[float, float], side: int,
               circle: float, radius: float, at_lip: bool) -> cq.Workplane:
    """Return the filler that rounds the corner between a spoke's side and a circle.

    start and end are the spoke side's ends (unrotated spoke, start nearer the hub), side +1
    or -1 the window's side of that edge. The circle (its radius about the axle) is the lip's
    inner edge (at_lip), where the spoke's tip widens, or the hub web's edge, where its root
    does: the window lies inside the lip and outside the web. The filler is the region between
    the corner, the edge and a concave arc of the given radius tangent to edge and circle, as
    deep as the plate. It reaches 0.3 mm into the lip or the web, so no gap is left at the
    circle.
    """
    ax, ay = start
    length = math.hypot(end[0] - ax, end[1] - ay)
    dx, dy = (end[0] - ax) / length, (end[1] - ay) / length
    nx, ny = (-dy, dx) if side > 0 else (dy, -dx)
    reach = circle - radius if at_lip else circle + radius
    into = 0.3 if at_lip else -0.3

    def along(px: float, py: float, distance: float) -> float:
        """Return t where the line (px, py) + t (dx, dy) reaches the distance from the axle."""
        b = px * dx + py * dy
        return -b + math.sqrt(b * b - (px * px + py * py - distance * distance))

    t = along(ax + radius * nx, ay + radius * ny, reach)
    cx, cy = ax + radius * nx + t * dx, ay + radius * ny + t * dy
    on_edge = (cx - radius * nx, cy - radius * ny)
    c = math.hypot(cx, cy)
    on_circle = (cx * circle / c, cy * circle / c)
    past_circle = (cx * (circle + into) / c, cy * (circle + into) / c)
    u = along(ax, ay, circle) + into
    corner = (ax + u * dx, ay + u * dy)
    mx, my = on_edge[0] + on_circle[0] - 2 * cx, on_edge[1] + on_circle[1] - 2 * cy
    m = math.hypot(mx, my)
    middle = (cx + radius * mx / m, cy + radius * my / m)
    return (cq.Workplane('XY', origin=(0, 0, SPOKE_FACE_Z)).moveTo(*corner).lineTo(*on_edge)
            .threePointArc(middle, on_circle).lineTo(*past_circle).close()
            .extrude(SPOKE_PLATE_BACK_Z - SPOKE_FACE_Z))


def spoke() -> cq.Workplane:
    """Return one spoke with its web and fin, its centre line along the x axis.

    The front plate is flat, with straight tapering sides. Curved blends widen it into the hub
    web at its root and into the lip at its tip. Behind the plate a narrow web runs along the
    centre line out to the barrel, and a fin from the web up the hub tube.
    """
    outline = spoke_outline()
    plate = (cq.Workplane('XY', origin=(0, 0, SPOKE_FACE_Z)).polyline(outline).close()
             .extrude(SPOKE_PLATE_BACK_Z - SPOKE_FACE_Z))
    wall = BARREL_INNER_D / 2 + 0.3                     # the web and the fin end in the barrel
    web = (cq.Workplane('XY', origin=(0, 0, SPOKE_PLATE_BACK_Z - 0.1))
           .center((SPOKE_ROOT_RADIUS + wall) / 2, 0)
           .rect(wall - SPOKE_ROOT_RADIUS, SPOKE_WEB_WIDTH)
           .extrude(HUB_BACK_Z - SPOKE_PLATE_BACK_Z + 0.1))
    # The fin starts inside the cup's wall and behind the bowl's floor, so it stays out of the bowl
    cup_wall, tube = CUP_D / 2 - 0.2, HUB_TUBE_D / 2 - 0.2
    fin = (cq.Workplane('XZ', origin=(0, FIN_THICKNESS / 2, 0))
           .polyline([(cup_wall, HUB_BACK_Z - 0.2), (wall, HUB_BACK_Z - 0.2),
                      (tube, FIN_TOP_Z), (tube, CUP_BACK_Z - 0.2),
                      (cup_wall, CUP_BACK_Z - 0.2)]).close().extrude(FIN_THICKNESS))
    corners = ((WEB_R, SPOKE_ROOT_BLEND, False), (LIP_INNER_D / 2, SPOKE_TIP_BLEND, True))
    blends = [side_blend(outline[3], outline[2], 1, *corner) for corner in corners]
    blends += [side_blend(outline[0], outline[1], -1, *corner) for corner in corners]
    return union_all([plate, web, fin, *blends])


def barrel() -> cq.Workplane:
    """Return the barrel: flanges, grooves and ridges, the outer face and the lip inside.

    The profile runs from the motor-side flange over the four grooves to the outer flange,
    down the outer face with its fine groove, and back along the inside: plain to the lip,
    the lip, and plain to the motor-side end. The flanges' outer corners are sharp, their
    inner corners round.
    """
    f, i = RIM_FACE_Z, RIM_INNER_FACE_Z
    r_flange, r_ridge, r_bottom = FLANGE_D / 2, RIDGE_D / 2, GROOVE_D / 2
    r_face, r_lip, r_in = FACE_INNER_R, LIP_INNER_D / 2, BARREL_INNER_D / 2
    if not r_lip < r_face <= r_in < r_bottom < r_ridge <= r_flange:
        raise ValueError('the barrel needs lip < inside < grooves < ridges <= flanges')
    spans = groove_spans()
    tops = [r_flange] + [r_ridge] * (len(spans) - 1) + [r_flange]
    grooves: List = []
    for k in reversed(range(len(spans))):
        top_start, bottom_start, bottom_end, top_end = spans[k]
        grooves += [('line', (tops[k + 1], top_end)), ('line', (r_bottom, bottom_end)),
                    ('line', (r_bottom, bottom_start)), ('line', (tops[k], top_start))]
    groove_outer = r_flange - FACE_GROOVE[0]
    groove_inner = groove_outer - FACE_GROOVE[1]
    groove_z, flank = f + FACE_GROOVE_DEPTH, FACE_GROOVE[1] / 6
    lip_front = f + LIP_START
    lip_back = lip_front + LIP_WIDTH
    return revolve_path((r_flange, i), [
        *grooves,
        ('line', (r_flange, f)),
        ('line', (groove_outer, f)), ('line', (groove_outer - flank, groove_z)),
        ('line', (groove_inner + flank, groove_z)), ('line', (groove_inner, f)),
        *round_corner((r_face, f), FLANGE_INNER_ROUND, (1, 0), (0, 1)),
        ('line', (r_face, lip_front)), ('line', (r_lip, lip_front)),
        ('line', (r_lip, lip_back)), ('line', (r_in, lip_back)),
        *round_corner((r_in, i), FLANGE_INNER_ROUND, (0, -1), (1, 0))])


def ribs() -> cq.Workplane:
    """Return the axial ribs across the two middle grooves, at the ridges' height.

    Each rib runs from the middle of the first ridge's top to the middle of the third's.
    """
    spans = groove_spans()
    z0 = (spans[0][3] + spans[1][0]) / 2
    z1 = (spans[-2][3] + spans[-1][0]) / 2
    r0, r1 = GROOVE_D / 2 - 0.2, RIDGE_D / 2
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
    boss_top, floor_z = BOSS_TOP_Z, BOWL_FLOOR_Z
    cup, collar, tube = CUP_D / 2, HUB_COLLAR_D / 2, HUB_TUBE_D / 2
    collar_top = CUP_BACK_Z + HUB_COLLAR_HEIGHT
    if not boss + BOSS_FILLET < floor_r < bowl <= edge < cup:
        raise ValueError('the bowl needs boss < floor < wall <= edge < cup')
    if not face < boss_top < floor_z < CUP_BACK_Z:
        raise ValueError("the boss's top must lie between the face and the floor")
    if not boss_top < BORE_END_Z < CUP_BACK_Z:
        raise ValueError('the bore must end inside the boss, behind its top')
    if not collar_top < FIN_TOP_Z:
        raise ValueError("the fins must reach up the tube, past the collar's top")
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
    """Return the yellow rim: barrel, ribs, lip, hub, web, spokes, fins, bore and marks."""
    if not WEB_R + SPOKE_ROOT_BLEND < LIP_INNER_D / 2 - SPOKE_TIP_BLEND:
        raise ValueError('the spokes need straight sides between their root and tip blends')
    # The spokes' central face: an annulus from the cup to WEB_R (the bowl stays open)
    web = (cq.Workplane('XY', origin=(0, 0, SPOKE_FACE_Z)).circle(WEB_R)
           .circle(CUP_D / 2 - 0.2).extrude(SPOKE_PLATE_BACK_Z - SPOKE_FACE_Z))
    one_spoke = spoke()
    rim = union_all([barrel(), ribs(), hub(), web,
                     *[one_spoke.rotate((0, 0, 0), (0, 0, 1), 360.0 * k / SPOKES)
                       for k in range(SPOKES)]])
    bore_length = HUB_TUBE_END_Z - BORE_END_Z + 1
    bore = (cq.Workplane('XY', origin=(0, 0, BORE_END_Z)).circle(BORE_D / 2).extrude(bore_length)
            .intersect(cq.Workplane('XY', origin=(0, 0, BORE_END_Z))
                       .rect(BORE_FLATS, BORE_D + 1).extrude(bore_length)))
    rim = rim.cut(bore)
    # Small pins on the bowl's floor, a recessed mark on each spoke
    pads = union_all([cq.Workplane('XY', origin=(HUB_PIN_R, 0, BOWL_FLOOR_Z - HUB_PIN_HEIGHT))
                      .circle(HUB_PIN_D / 2).extrude(HUB_PIN_HEIGHT + 0.01)
                      .rotate((0, 0, 0), (0, 0, 1), HUB_PIN_ANGLE + 360.0 * k / HUB_PINS)
                      for k in range(HUB_PINS)])
    marks = union_all([cq.Workplane('XY', origin=(SPOKE_MARK_RADIUS, 0, SPOKE_FACE_Z - 0.01))
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
