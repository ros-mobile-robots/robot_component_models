# SPDX-FileCopyrightText: 2026 ROS Mobile Robots
# SPDX-License-Identifier: BSD-3-Clause
"""Generate the 65 mm wheel for TT gear motors: yellow five-spoke rim, black block-tread tyre.

Modelled from caliper measurements of the real wheel (tyre taken off the rim), photos and video
frames of DiffBot's and Remo's wheels, shop photos and product listings. The rim is a barrel with
a flange at each end and three ridges between them, axial ribs across the two pockets, a ring on
the outer face that curves into the windows, five tapered swept spokes with a narrow web and a fin
behind each, a recessed bowl around the centre boss with five small pads, and a hub tube on a
collar on the motor side. The tyre's thin edge lips hook over the flanges and two inner rings sit
in the plain bands. Sources in provenance/tt_motor_wheel_65mm.md.

Frame (positions in mm, the files are in metres): the axle along z. The
tyre's outer face is at z = -13 mm, the rim's outer flange face and the boss 0.74 mm behind it
(the tyre's lip), the rim's motor-side flange face at +12.74 mm and the hub tube's end at
+16.16 mm.
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

# Tyre
TYRE_OUTER_D = 66.6          # measured on DiffBot: 66.3 and 66.8 (front_wheel.yaml)
# The edge lips' axial thickness isn't measured: p03 reads 0.74 radially (provenance:
# measurement 1). It sets the tyre's width and the rim's z position, so the tube's end too
TYRE_LIP_THICKNESS = 0.74    # choice
TYRE_LIP_INNER_D = 49.4      # p07, p08: edge lip to tread 8.38 and 8.82 -> 66.6 - 2 x 8.6
TYRE_SHOULDER_RADIUS = 3.0   # choice (photos: rounded shoulders)
TYRE_WALL = 7.46             # p06: inner surface to a tread block's top, off the rim
TYRE_SEAT_GAP = 0.02         # choice: the tyre's lips and rings stand this far off the rim
# p05/p06: 7.46 - 6.88, conditional: if p05 is at a groove (provenance: measurement 3)
TREAD_DEPTH = 0.6
TREAD_ROWS = 42              # p07, p08: about 5 mm row pitch, blocks about 4.3 mm long
TREAD_TRANSVERSE_GROOVE = 0.7  # p07, p08: narrow grooves between the blocks (width: choice)
# Circumferential grooves (offset from the tyre's centre plane, width): six block columns
# (p07, p08, video); positions and widths: choice
TREAD_CIRCUMFERENTIAL_GROOVES = ((0.0, 0.7), (4.6, 0.7), (-4.6, 0.7), (9.0, 0.7), (-9.0, 0.7))
TREAD_SHOULDER_REACH = 30.0  # radius down to which the shoulder blocks wrap onto the sidewall

# Rim barrel (from the outer flange face F towards the motor side)
RIM_WIDTH = 25.00            # p40: flange to flange
RIM_LENGTH = 28.42           # p33: outer face (flange and boss flush) to the hub tube's end
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

# Hub, outer face
BOSS_D = 9.69                # p23
BOSS_CHAMFER = 0.4           # choice
# The hub cup is a bowl recessed into the spokes' central face, the boss standing in it, flush
# with the flange (p20-p25, p33, video). Its depths aren't measured (provenance: measurement 2)
CUP_D = 20.64                # p24: the ring's outside (p16/p17 from behind: 20.49)
CUP_INNER_D = 17.62          # p21: inside jaws across the bowl at its rim
# The bowl narrows towards its floor: p20 reads 14.07 with inside jaws deeper in the bowl
# (provenance: measurement 2)
BOWL_FLOOR_D = 15.2          # choice
CUP_RIM_DEPTH = 0.8          # choice: the ring's top, 0.4 in front of the spokes' face
CUP_FLOOR_DEPTH = 2.2        # choice: the bowl's floor, 1.0 behind the spokes' face
BOSS_FILLET = 0.4            # choice: where the boss meets the bowl's floor
RING_ROUND = 0.35            # choice: height of the ring's rounded top
HUB_PAD_R = 6.8              # p41 img., video: five small raised pads between the spokes
HUB_PAD_D = 1.0              # choice (video: small round pads)
HUB_PAD_HEIGHT = 0.3         # choice
WEB_R = 11.2                 # p41 img.: windows start at r 11.4 between the spokes; cup 10.32

# Spokes: five, tapered and swept
SPOKES = 5
SPOKE_FACE_DEPTH = 1.2       # video: behind the face
SPOKE_PLATE = 1.5            # choice: thickness of the spoke's front plate
SPOKE_WEB_WIDTH = 3.47       # p18, p19: the spoke's back, near the hub
HUB_BACK_DEPTH = 4.5         # choice: the spokes' back plane behind the face
SPOKE_ROOT_RADIUS = 9.0      # choice: the loft starts inside the cup's wall
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
BORE_BOTTOM_DEPTH = 2.0      # choice: blind, the boss is closed (photos)
FIN_THICKNESS = 1.51         # p31
FIN_TOP_Z = 3.0              # video, p32: the fins reach about half way up the tube
FIN_OUTER_R = 20.0           # p32: the fins run out along the spokes

# Derived positions
RIM_FACE_Z = TYRE_OUTER_FACE_Z + TYRE_LIP_THICKNESS      # F: the outer flange face, -12.26
RIM_INNER_FACE_Z = RIM_FACE_Z + RIM_WIDTH                 # the motor-side flange face, 12.74
HUB_TUBE_END_Z = RIM_FACE_Z + RIM_LENGTH                  # 16.16
TYRE_INNER_FACE_Z = RIM_INNER_FACE_Z + TYRE_LIP_THICKNESS  # 13.48
SPOKE_FACE_Z = RIM_FACE_Z + SPOKE_FACE_DEPTH
SPOKE_PLATE_BACK_Z = SPOKE_FACE_Z + SPOKE_PLATE
HUB_BACK_Z = RIM_FACE_Z + HUB_BACK_DEPTH

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


def revolve_path(start: Tuple[float, float], pieces: Sequence) -> cq.Workplane:
    """Revolve a closed (r, z) path about the z axis.

    The pieces are ('line', end), ('arc', middle, end) or ('spline', end, start_tangent,
    end_tangent), all in (r, z).
    """
    wp = cq.Workplane('XZ').moveTo(*start)
    for piece in pieces:
        if piece[0] == 'line':
            wp = wp.lineTo(*piece[1])
        elif piece[0] == 'arc':
            wp = wp.threePointArc(piece[1], piece[2])
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


def tyre_envelope(inset: float) -> cq.Workplane:
    """Return the tyre's outer layer from r 28 out, its outer surfaces moved inwards by inset."""
    outer = TYRE_OUTER_D / 2 - inset
    z0, z1 = TYRE_OUTER_FACE_Z + inset, TYRE_INNER_FACE_Z - inset
    shoulder = TYRE_SHOULDER_RADIUS - inset
    return revolve_path((28.0, z0), [
        *round_corner((outer, z0), shoulder, (-1, 0), (0, 1)),
        *round_corner((outer, z1), shoulder, (0, -1), (-1, 0)),
        ('line', (28.0, z1))])


def tyre_body() -> cq.Workplane:
    """Return the tyre without tread grooves: lips over the flanges, rings in the plain bands."""
    outer = TYRE_OUTER_D / 2
    z0, z1 = TYRE_OUTER_FACE_Z, TYRE_INNER_FACE_Z
    # The tyre's own inner surface (66.6 - 2 x 7.46 = 51.68) clears the flanges (51.22): the
    # tyre is held by its edge lips and by the inner rings in the plain bands, not stretched
    seat = TYRE_OUTER_D / 2 - TYRE_WALL
    if seat <= FLANGE_D / 2:
        raise ValueError("the tyre's inner surface must clear the rim's flanges")
    lip = TYRE_LIP_INNER_D / 2
    gap = TYRE_SEAT_GAP
    rings = []
    for lo, hi in reversed(plain_bands()):
        a, b = lo + 0.3, hi - 0.3
        depth = seat - BAND_D / 2 - gap
        rings += [('line', (seat, b)), ('line', (seat - depth, b - 0.4)),
                  ('line', (seat - depth, a + 0.4)), ('line', (seat, a))]
    return revolve_path((lip, z0), [
        *round_corner((outer, z0), TYRE_SHOULDER_RADIUS, (-1, 0), (0, 1)),
        *round_corner((outer, z1), TYRE_SHOULDER_RADIUS, (0, -1), (-1, 0)),
        ('line', (lip, z1)), ('line', (lip, RIM_INNER_FACE_Z + gap)),
        ('line', (seat, RIM_INNER_FACE_Z + gap)), *rings,
        ('line', (seat, RIM_FACE_Z - gap)), ('line', (lip, RIM_FACE_Z - gap))])


def tread_cutter() -> cq.Workplane:
    """Return the grooves: circumferential rings and transverse slabs, one tread depth deep."""
    reach = TYRE_OUTER_D / 2 + 5
    centre = (TYRE_OUTER_FACE_Z + TYRE_INNER_FACE_Z) / 2
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
    skin = tyre_envelope(0.0).cut(tyre_envelope(TREAD_DEPTH))
    return rings.union(slabs).intersect(skin)


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
    """Return the hub: boss standing in the bowl, the ring around it, the collar and the tube.

    The bowl's wall curves from the ring's inside (CUP_INNER_D) down to its floor; the ring's
    top is rounded and its outside drops to the spokes' face.
    """
    f = RIM_FACE_Z
    boss, cup, inner = BOSS_D / 2, CUP_D / 2, CUP_INNER_D / 2
    floor_r = BOWL_FLOOR_D / 2
    rim_z, floor_z = f + CUP_RIM_DEPTH, f + CUP_FLOOR_DEPTH
    top = rim_z + RING_ROUND
    collar, tube = HUB_COLLAR_D / 2, HUB_TUBE_D / 2
    collar_top = HUB_BACK_Z + HUB_COLLAR_HEIGHT
    if not boss + BOSS_FILLET < floor_r < inner < cup:
        raise ValueError('the bowl needs boss < floor < inner wall < ring')
    if not top < SPOKE_FACE_Z < floor_z < f + BORE_BOTTOM_DEPTH + 1.0:
        raise ValueError("the ring must stand in front of the spokes' face, the floor behind it")
    # The bowl's wall: a concave spline from the floor's edge up to the ring's inside, leaving
    # the floor flat and arriving with the tangent of the ring's rounded top, so both joins are
    # smooth. The ring's top is the circle through (inner, top), (middle, rim_z), (cup, top)
    half = (cup - inner) / 2
    centre_z = (half * half + top * top - rim_z * rim_z) / (2 * (top - rim_z))
    length = math.hypot(centre_z - top, half)
    ring_tangent = ((centre_z - top) / length, -half / length)
    return revolve_path((0, f), [
        ('line', (boss - BOSS_CHAMFER, f)), ('line', (boss, f + BOSS_CHAMFER)),
        *round_corner((boss, floor_z), BOSS_FILLET, (0, -1), (1, 0)),
        ('line', (floor_r, floor_z)),
        ('spline', (inner, top), (1.0, 0.0), ring_tangent),
        ('arc', ((inner + cup) / 2, rim_z), (cup, top)),
        ('line', (cup, HUB_BACK_Z)), ('line', (collar, HUB_BACK_Z)),
        ('line', (collar, collar_top - HUB_COLLAR_CHAMFER)),
        ('line', (collar - HUB_COLLAR_CHAMFER, collar_top)),
        *round_corner((tube, collar_top), HUB_TUBE_FILLET, (1, 0), (0, 1)),
        ('line', (tube, HUB_TUBE_END_Z - HUB_TUBE_CHAMFER)),
        ('line', (tube - HUB_TUBE_CHAMFER, HUB_TUBE_END_Z)), ('line', (0, HUB_TUBE_END_Z))])


def make_rim() -> cq.Workplane:
    """Return the yellow rim: barrel, ribs, ring, hub, web, spokes, fins, bore and marks."""
    if SPOKE_TIP_RADIUS >= BARREL_INNER_D / 2 or SPOKE_TIP_RADIUS <= WINDOW_OUTER_R:
        raise ValueError("the spokes' tips must end inside the ring's curl")
    # The spokes' central face: an annulus from the ring to WEB_R (the bowl stays open)
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
    # Raised pads on the hub cup's floor between the spokes, a recessed mark on each spoke
    floor_z = RIM_FACE_Z + CUP_FLOOR_DEPTH
    pads = union_all([cq.Workplane('XY', origin=(HUB_PAD_R, 0, floor_z - HUB_PAD_HEIGHT))
                      .circle(HUB_PAD_D / 2).extrude(HUB_PAD_HEIGHT + 0.01)
                      .rotate((0, 0, 0), (0, 0, 1), 360.0 * (k + 0.5) / SPOKES)
                      for k in range(SPOKES)])
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
FRAME = ("The axle along z at x = y = 0, the tyre's outer face at z = -13 mm, the rim's "
         "motor-side flange face at z = +12.74 mm and the hub tube's end at z = +16.16 mm.")
SUB_PARTS = {'rim': 'rim', 'tyre': 'tyre'}


def build() -> List[Model]:
    """Return the wheel's mesh: the yellow rim and the black tyre."""
    solids = {'rim': make_rim(), 'tyre': make_tyre()}
    sub_parts = [SubPart(part, {material: tessellate_faces(solids[part], LINEAR_DEFLECTION,
                                                           ANGULAR_DEFLECTION)})
                 for part, material in SUB_PARTS.items()]
    return [Model('tt_motor_wheel_65mm', TITLE, FRAME, MATERIALS, sub_parts)]
