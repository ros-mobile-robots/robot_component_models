# Copyright (c) 2026, ROS Mobile Robots
# SPDX-License-Identifier: BSD-3-Clause
"""Generate a 360 degree laser scanner of the RPLIDAR A2 type: base, rotating head, window.

Modelled from the public datasheet dimensions only (RPLIDAR A2M8 datasheet rev. 2.5, Figure 5-1
"Mechanical Dimensions"): base diameter and height, head diameter, overall height, the optical
window's centre height and outline, the four M3 mounting holes and the locating hole. The
proportions of the window bezel and the head's rounded top edge are read from the same drawing,
the colours from a photo of Remo on ros-mobile-robots.com. No cable, no logos, no printed text.
Sources in provenance/rplidar_a2.md.

Frame: the rotation axis along z, the bottom at z = -29.85 mm (so the optical window's centre
is at z = +0.95 mm), the cable exit towards -x, the window towards +x. Millimetres in the STL,
metres in the COLLADA and glTF files.
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

TITLE = '360 degree laser scanner, RPLIDAR A2 size'

# ---------------------------------------------------------------------------------------------
# Parameters, in mm. Datasheet values are marked (DS), values read off the drawing's scale
# (drawing) and choices (choice).
# ---------------------------------------------------------------------------------------------
BOTTOM_Z = -29.85            # frame alignment: the earlier mesh's bottom
BASE_DIAMETER = 75.70        # DS
BASE_HEIGHT = 18.60          # DS
BASE_CHAMFER = 0.8           # choice
NECK_TOP = 19.40             # DS: the head starts 19.40 above the bottom
NECK_DIAMETER = 71.0         # drawing: a short step between base and head
HEAD_DIAMETER = 72.50        # DS
RING_HEIGHT = 1.30           # DS: the band at the head's lower edge (copper-coloured, photo)
RING_DEPTH = 1.5             # choice: radial thickness of that band
TOTAL_HEIGHT = 40.80         # DS
HEAD_TOP_ROUND = 11.0        # drawing: radius of the head's rounded top edge
WINDOW_CENTRE = 30.80        # DS: optical centre above the bottom (laser band 6 mm high)
BEZEL_LENGTH = 58.0          # drawing: outer outline of the window bezel, end to end
BEZEL_HEIGHT = 14.9          # drawing
BEZEL_WIDTH = 1.1            # drawing: the raised rim's width
BEZEL_PROUD = 0.3            # choice: the rim stands out from the head
BEZEL_DEPTH = 1.0            # choice
BAND_RECESS = 1.2            # choice: the surface inside the rim is recessed
OPENING_LENGTH = 18.7        # drawing: two openings, emitter and camera
OPENING_HEIGHT = 10.0        # drawing
OPENING_OFFSET = 17.7        # drawing: their centres from the window's centre
OPENING_DEPTH = 7.8          # choice
GLASS_THICKNESS = 0.8        # choice: dark glass at the bottom of each opening
MOUNT_HOLE_RADIUS = 29.50    # DS: 4 x M3 on R29.50, at 45 degrees to the cable axis
MOUNT_HOLE_DIAMETER = 2.5    # M3 tapping hole
MOUNT_HOLE_DEPTH = 4.0       # choice: DS limits the screws to 4 mm, the depth isn't given
LOCATING_HOLE_DIAMETER = 3.0  # DS: 3 mm, 1.0 deep, on R29.50 opposite the cable
LOCATING_HOLE_DEPTH = 1.0
CASE_SCREW_RADIUS = 34.0     # drawing: six case screws in the bottom
CASE_SCREW_DIAMETER = 3.4
CASE_SCREW_DEPTH = 1.2
CASE_SCREW_ANGLES = (30, 90, 150, 210, 270, 330)
CABLE_COVER_EDGE = 25.0      # DS: the cable cover's inner edge from the centre (bottom view)
CABLE_COVER_WIDTH = 26.0     # drawing
SEAM = 0.6                   # choice: seam groove around the cable cover
CABLE_BOOT = (3.0, 9.0, 4.5)  # length beyond the base, width, height (choice, drawing)
CABLE_BOOT_LIFT = 1.0        # boot's lower edge above the bottom


LINEAR_DEFLECTION = 0.04
ANGULAR_DEFLECTION = 0.3

MATERIALS = {
    'body': phong((0.09, 0.09, 0.10, 1.0), (0.15, 0.15, 0.15, 1.0), 12.0),
    'trim': phong((0.52, 0.22, 0.18, 1.0), (0.35, 0.30, 0.28, 1.0), 30.0),
    'window': phong((0.02, 0.02, 0.03, 1.0), (0.60, 0.60, 0.60, 1.0), 80.0),
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


def revolve_path(start: Tuple[float, float], pieces: Sequence) -> cq.Workplane:
    """Revolve a closed (r, z) path of lines and arcs about the z axis."""
    wp = cq.Workplane('XZ').moveTo(*start)
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


def z_at(height: float) -> float:
    """Return the frame z of a height above the scanner's bottom."""
    return BOTTOM_Z + height


def head_shell(offset: float) -> cq.Workplane:
    """Return the head above the ring, its outer surface moved outwards by offset (mm)."""
    radius = HEAD_DIAMETER / 2 + offset
    top = z_at(TOTAL_HEIGHT) + offset
    bottom = z_at(NECK_TOP + RING_HEIGHT) + max(-offset, 0.0)
    return revolve_path((0, bottom), [
        ('line', (radius, bottom)),
        *round_corner((radius, top), HEAD_TOP_ROUND + offset, (0, -1), (-1, 0)),
        ('line', (0, top))])


def window_prism(length: float, height: float, y_centre: float = 0.0) -> cq.Workplane:
    """Return a stadium-shaped prism along +x, seen from +x, centred on the window."""
    return (cq.Workplane('YZ', origin=(0, y_centre, z_at(WINDOW_CENTRE)))
            .slot2D(length, height).extrude(HEAD_DIAMETER))


def cylinder(radius: float) -> cq.Workplane:
    """Return a tall cylinder about the z axis."""
    return cq.Workplane('XY', origin=(0, 0, BOTTOM_Z - 1)).circle(radius).extrude(TOTAL_HEIGHT + 2)


def make_parts() -> Dict[str, cq.Workplane]:
    """Return the scanner's solids per material."""
    rb, rn, rh = BASE_DIAMETER / 2, NECK_DIAMETER / 2, HEAD_DIAMETER / 2
    base = revolve_path((0, z_at(0)), [
        ('line', (rb - BASE_CHAMFER, z_at(0))), ('line', (rb, z_at(BASE_CHAMFER))),
        ('line', (rb, z_at(BASE_HEIGHT - 0.5))), ('line', (rb - 0.5, z_at(BASE_HEIGHT))),
        ('line', (rn, z_at(BASE_HEIGHT))), ('line', (rn, z_at(NECK_TOP))),
        ('line', (rh - RING_DEPTH, z_at(NECK_TOP))),
        ('line', (rh - RING_DEPTH, z_at(NECK_TOP + RING_HEIGHT))),
        ('line', (0, z_at(NECK_TOP + RING_HEIGHT)))])
    ring = revolve_path((rh - RING_DEPTH, z_at(NECK_TOP)), [
        ('line', (rh, z_at(NECK_TOP))), ('line', (rh, z_at(NECK_TOP + RING_HEIGHT))),
        ('line', (rh - RING_DEPTH, z_at(NECK_TOP + RING_HEIGHT)))])
    # Cable boot on -x, low on the base
    length, width, height = CABLE_BOOT
    boot = (cq.Workplane('XY', origin=(-(rb + length) + (length + 1.5) / 2, 0,
                                       z_at(CABLE_BOOT_LIFT) + height / 2))
            .box(length + 1.5, width, height).edges('|Z').fillet(0.8))
    base = base.union(boot)
    # Bottom: four M3 holes, the locating hole, case screws, the cable cover's seam
    holes = union_all([
        cq.Workplane('XY', origin=(MOUNT_HOLE_RADIUS * math.cos(math.radians(45 + 90 * k)),
                                   MOUNT_HOLE_RADIUS * math.sin(math.radians(45 + 90 * k)),
                                   z_at(-0.01)))
        .circle(MOUNT_HOLE_DIAMETER / 2).extrude(MOUNT_HOLE_DEPTH)
        for k in range(4)])
    holes = holes.union(cq.Workplane('XY', origin=(MOUNT_HOLE_RADIUS, 0, z_at(-0.01)))
                        .circle(LOCATING_HOLE_DIAMETER / 2).extrude(LOCATING_HOLE_DEPTH))
    for angle in CASE_SCREW_ANGLES:
        a = math.radians(angle)
        holes = holes.union(cq.Workplane('XY', origin=(CASE_SCREW_RADIUS * math.cos(a),
                                                       CASE_SCREW_RADIUS * math.sin(a),
                                                       z_at(-0.01)))
                            .circle(CASE_SCREW_DIAMETER / 2).extrude(CASE_SCREW_DEPTH))
    cover = (cq.Workplane('XY', origin=(-(CABLE_COVER_EDGE + rb) / 2 - 2, 0, z_at(-0.01)))
             .rect(rb - CABLE_COVER_EDGE + 4, CABLE_COVER_WIDTH).extrude(0.5))
    inner = (cq.Workplane('XY', origin=(-(CABLE_COVER_EDGE + rb) / 2 - 2 - SEAM, 0, z_at(-0.02)))
             .rect(rb - CABLE_COVER_EDGE + 4, CABLE_COVER_WIDTH - 2 * SEAM).extrude(0.6))
    seam = cover.cut(inner).intersect(cylinder(rb - 1.2))
    base = base.cut(holes).cut(seam)

    head = head_shell(0.0)
    skin_in = head.cut(head_shell(-BEZEL_DEPTH))
    rim_region = window_prism(BEZEL_LENGTH, BEZEL_HEIGHT).cut(
        window_prism(BEZEL_LENGTH - 2 * BEZEL_WIDTH, BEZEL_HEIGHT - 2 * BEZEL_WIDTH))
    bezel = rim_region.intersect(head_shell(BEZEL_PROUD).cut(head_shell(-BEZEL_DEPTH)))
    head = head.cut(rim_region.intersect(skin_in))
    band = window_prism(BEZEL_LENGTH - 2 * BEZEL_WIDTH, BEZEL_HEIGHT - 2 * BEZEL_WIDTH).intersect(
        head_shell(0.0).cut(head_shell(-BAND_RECESS)))
    head = head.cut(band)
    floor = rh - OPENING_DEPTH
    panes = []
    for side in (-1, 1):
        opening = window_prism(OPENING_LENGTH, OPENING_HEIGHT, side * OPENING_OFFSET)
        head = head.cut(opening.cut(cylinder(floor)))
        pane = opening.intersect(cylinder(floor + GLASS_THICKNESS).cut(cylinder(floor)))
        panes.append(pane)
    return {'body': base.union(head), 'trim': ring.union(bezel), 'window': union_all(panes)}


# ---------------------------------------------------------------------------------------------
# Mesh
# ---------------------------------------------------------------------------------------------
FRAME = ('The rotation axis along z at x = y = 0, the bottom at z = -29.85 mm, the optical '
         "window's centre at z = +0.95 mm and facing +x, the cable exit towards -x.")
SUB_PARTS = {'housing': 'body', 'trim': 'trim', 'windows': 'window'}


def build() -> List[Model]:
    """Return the scanner's mesh: housing, copper-coloured trim and dark windows."""
    solids = make_parts()
    sub_parts = [SubPart(part, {material: tessellate_faces(solids[material], LINEAR_DEFLECTION,
                                                           ANGULAR_DEFLECTION)})
                 for part, material in SUB_PARTS.items()]
    return [Model('rplidar-a2', TITLE, FRAME, MATERIALS, sub_parts)]
