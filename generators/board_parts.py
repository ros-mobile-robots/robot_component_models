# SPDX-FileCopyrightText: 2026 ROS Mobile Robots
# SPDX-License-Identifier: BSD-3-Clause
"""Shapes that single-board computers share: connectors, headers, small parts and materials.

The connectors are drawn in their own frame and placed with place(): the opening faces +x, the
front face is at x = 0, the body reaches back to x = -depth, it is centred on y = 0 and stands
on z = 0. The body stays inside the box (-depth ... 0) x (-width / 2 ... width / 2) x
(0 ... height) that the caller gives. Two allowances reach out of it, as the connectors' fronts
do on the real boards: a USB stack's front frame has ears of `ears` on both sides (y), and an
RJ45 jack's LEDs stand out of the front face by `led_protrusion` (x). All other details stay
inside the box.

The materials are Phong values for COLLADA (Gazebo and RViz show the specular highlights) and
metal or not for glTF (PBR metallic and roughness).
"""
import math
from typing import Dict
from typing import List
from typing import Sequence
from typing import Tuple

import cadquery as cq
import numpy as np

from generators.mesh_export import Material
from generators.mesh_export import phong
from generators.mesh_export import Surface

Part = Tuple[str, cq.Shape]
Point = Tuple[float, float]

# name: (diffuse RGB, specular grey, shininess, metal). The colours are sampled from product
# photos (see the provenance pages); metals get strong, small highlights, the solder mask a
# semi-gloss, plastics a dull sheen.
MATERIAL_SPECS: Dict[str, Tuple[Tuple[float, float, float], float, float, bool]] = {
    'solder_mask_green': ((0.20, 0.53, 0.37), 0.35, 40.0, False),
    'solder_mask_black': ((0.06, 0.06, 0.07), 0.40, 45.0, False),
    'module_mask_green': ((0.14, 0.42, 0.21), 0.35, 40.0, False),
    'fr4_edge': ((0.80, 0.73, 0.55), 0.10, 10.0, False),
    'gold': ((0.90, 0.70, 0.32), 0.90, 70.0, True),
    'nickel': ((0.62, 0.61, 0.60), 0.90, 60.0, True),
    'steel_lid': ((0.66, 0.63, 0.61), 0.70, 35.0, True),
    'tin': ((0.74, 0.74, 0.76), 0.90, 50.0, True),
    'brass': ((0.78, 0.62, 0.32), 0.85, 50.0, True),
    'anodised_black': ((0.09, 0.09, 0.10), 0.35, 18.0, True),
    'black_plastic': ((0.05, 0.05, 0.05), 0.25, 20.0, False),
    'chip': ((0.10, 0.10, 0.11), 0.30, 25.0, False),
    'die': ((0.17, 0.18, 0.22), 0.80, 80.0, False),
    'thermal_paste': ((0.55, 0.56, 0.58), 0.15, 10.0, False),
    'thermal_pad': ((0.45, 0.50, 0.58), 0.10, 8.0, False),
    'ceramic': ((0.66, 0.56, 0.40), 0.20, 15.0, False),
    'substrate': ((0.14, 0.24, 0.16), 0.25, 20.0, False),
    'inductor': ((0.22, 0.22, 0.24), 0.15, 12.0, False),
    'usb3_blue': ((0.20, 0.50, 0.85), 0.30, 25.0, False),
    'cream_plastic': ((0.86, 0.85, 0.80), 0.20, 15.0, False),
    'latch_brown': ((0.30, 0.22, 0.14), 0.20, 15.0, False),
    'white_plastic': ((0.92, 0.91, 0.86), 0.20, 15.0, False),
    'capacitor_stripe': ((0.42, 0.47, 0.58), 0.30, 25.0, False),
    'led_red': ((0.90, 0.10, 0.10), 0.40, 30.0, False),
    'led_green': ((0.20, 0.85, 0.25), 0.40, 30.0, False),
    'led_yellow': ((0.95, 0.80, 0.10), 0.40, 30.0, False),
}
# A darker ambient than diffuse keeps the shading in Gazebo, whose scenes have ambient light.
AMBIENT_FACTOR = 0.6


def materials(names: Sequence[str]) -> Dict[str, Material]:
    """Return the named materials of MATERIAL_SPECS, in the given order."""
    result = {}
    for name in names:
        rgb, specular, shininess, metal = MATERIAL_SPECS[name]
        result[name] = phong(rgb, (specular, specular, specular), shininess, metal,
                             AMBIENT_FACTOR)
    return result


# ---------------------------------------------------------------------------------------------
# Basic shapes
# ---------------------------------------------------------------------------------------------
def box(x0: float, y0: float, z0: float, x1: float, y1: float, z1: float) -> cq.Shape:
    """Return the box between two corners."""
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, cq.Vector(x0, y0, z0))


def rounded_box(x0: float, y0: float, z0: float, x1: float, y1: float, z1: float,
                vertical: float = 0.0, top: float = 0.0, chamfer_top: float = 0.0) -> cq.Shape:
    """Return a box with rounded vertical edges and a rounded or chamfered top edge."""
    shape = cq.Workplane('XY').box(x1 - x0, y1 - y0, z1 - z0, centered=False) \
        .translate((x0, y0, z0))
    if vertical:
        shape = shape.edges('|Z').fillet(vertical)
    if top:
        shape = shape.faces('>Z').edges().fillet(top)
    if chamfer_top:
        shape = shape.faces('>Z').edges().chamfer(chamfer_top)
    return shape.val()


def prism(points: Sequence[Point], z0: float, z1: float) -> cq.Shape:
    """Return the polygon extruded from z0 to z1."""
    return cq.Workplane('XY', origin=(0, 0, z0)).polyline(list(points)).close() \
        .extrude(z1 - z0).val()


def cylinder(radius: float, base: Tuple[float, float, float],
             direction: Tuple[float, float, float], length: float) -> cq.Shape:
    """Return a cylinder from its base centre along the direction."""
    return cq.Solid.makeCylinder(radius, length, cq.Vector(*base), cq.Vector(*direction))


def ring(centre: Point, outer: float, inner: float, z0: float, z1: float) -> cq.Shape:
    """Return a flat annulus between z0 and z1, given its outer and inner diameters."""
    disc = cylinder(outer / 2, (centre[0], centre[1], z0), (0, 0, 1), z1 - z0)
    hole = cylinder(inner / 2, (centre[0], centre[1], z0 - 0.1), (0, 0, 1), z1 - z0 + 0.2)
    return disc.cut(hole)


def front_profile(points: Sequence[Point], front_x: float, depth: float,
                  inset: float = 0.0) -> cq.Shape:
    """Return a (y, z) outline extruded from the front face x = front_x back to -x.

    With an inset the outline is shrunk by that much, for the walls of a liner.
    """
    sketch = cq.Workplane('YZ', origin=(front_x - depth, 0, 0)).polyline(list(points)).close()
    if inset:
        sketch = sketch.offset2D(-inset)
    return sketch.extrude(depth).val()


def chamfered_rectangle(width: float, height: float, chamfer: float, bottom: float,
                        corners: str = 'lower') -> List[Point]:
    """Return the (y, z) outline of an opening centred on y = 0, chamfered at two corners."""
    half = width / 2
    if corners == 'lower':
        return [(-half + chamfer, bottom), (half - chamfer, bottom), (half, bottom + chamfer),
                (half, bottom + height), (-half, bottom + height), (-half, bottom + chamfer)]
    if corners == 'upper':
        return [(-half, bottom), (half, bottom), (half, bottom + height - chamfer),
                (half - chamfer, bottom + height), (-half + chamfer, bottom + height),
                (-half, bottom + height - chamfer)]
    # One chamfered corner, as the DisplayPort receptacle has
    return [(-half, bottom), (half - chamfer, bottom), (half, bottom + chamfer),
            (half, bottom + height), (-half, bottom + height)]


def opening_with_liner(points: Sequence[Point], depth: float,
                       wall: float) -> Tuple[cq.Shape, cq.Shape]:
    """Return the cut for an opening in the front face (x = 0) and a dark liner for it."""
    opening = front_profile(points, 0.1, depth + 0.1)
    liner = front_profile(points, 0.0, depth).cut(front_profile(points, 0.1, depth - wall + 0.1,
                                                                wall))
    return opening, liner


def place(parts: Sequence[Part], yaw_degrees: float,
          offset: Tuple[float, float, float]) -> List[Part]:
    """Return the parts turned about z, then moved by the offset."""
    placed = []
    for material, shape in parts:
        turned = shape.rotate(cq.Vector(0, 0, 0), cq.Vector(0, 0, 1), yaw_degrees) \
            if yaw_degrees else shape
        placed.append((material, turned.translate(cq.Vector(*offset))))
    return placed


# ---------------------------------------------------------------------------------------------
# Pin headers
# ---------------------------------------------------------------------------------------------
PIN_SIZE = 0.64
PIN_PITCH = 2.54
HEADER_BASE_HEIGHT = 2.5


def header_pin(x: float, y: float, z0: float, z1: float) -> cq.Shape:
    """Return a square 0.64 mm pin from z0 to z1 with pointed (chamfered) ends."""
    pin = cq.Workplane('XY').box(PIN_SIZE, PIN_SIZE, z1 - z0).translate((x, y, (z0 + z1) / 2))
    return pin.faces('>Z or <Z').edges().chamfer(0.2).val()


def pin_header(pins: Sequence[Point], top: float, bottom: float, pin_height: float,
               tail: float) -> List[Part]:
    """Return a 2.54 mm pin header: one plastic block per pin row and gold pins.

    The rows are the pins that share an x (or a y for headers along x): each gets its own
    block with chamfered top edges, so the base shows the grooves of a real header.
    """
    parts: List[Part] = []
    xs = sorted({round(x, 3) for x, _ in pins})
    ys = sorted({round(y, 3) for _, y in pins})
    along_x = len(xs) >= len(ys)
    for key in (xs if along_x else ys):
        row = [p for p in pins if round(p[0] if along_x else p[1], 3) == key]
        others = [p[1] if along_x else p[0] for p in row]
        lo, hi = min(others) - PIN_PITCH / 2, max(others) + PIN_PITCH / 2
        if along_x:
            block = (key - PIN_PITCH / 2, lo, key + PIN_PITCH / 2, hi)
        else:
            block = (lo, key - PIN_PITCH / 2, hi, key + PIN_PITCH / 2)
        parts.append(('black_plastic', rounded_box(block[0] + 0.03, block[1], top,
                                                   block[2] - 0.03, block[3],
                                                   top + HEADER_BASE_HEIGHT, chamfer_top=0.3)))
    parts += [('gold', header_pin(x, y, bottom - tail, top + pin_height)) for x, y in pins]
    return parts


# ---------------------------------------------------------------------------------------------
# Connectors in their own frame (opening towards +x, front face at x = 0)
# ---------------------------------------------------------------------------------------------
# Insides of the USB Type-A receptacles, from GCT's drawing of the stacked USB 3.0 Type-A
# receptacle USB1100 (see the provenance pages): the opening, the tongue and its gap to the
# opening's top, the depth to the rear housing, the ports 8.5 mm apart, the USB 2.0 contacts'
# positions and width, and the SuperSpeed contacts within 3 mm of the tongue's tip.
USB_A_OPENING = (12.5, 5.12)  # width, height
USB_A_TONGUE = (11.1, 1.84, 0.40)  # width, thickness, gap from the opening's top
USB_A_PORT_DEPTH = 10.25  # from the front to the rear housing
USB_A_PORT_BOTTOMS = (0.95, 9.45)  # above the board
USB2_CONTACTS = ((-3.5, -1.0, 1.0, 3.5), 0.95, 0.35)  # y, width, reach below the tongue
USB3_CONTACTS = (5, 2.3, 1.0, 3.0)  # count, pitch (estimated), width, length from the tip
# RJ45: eight spring contacts at the modular plug's 1.02 mm (0.040 in) pitch, angled down and
# back from the top of the plug opening; wire size and angle estimated from photos.
RJ45_CONTACTS = (8, 1.02, 0.4, 0.3)  # count, pitch, width, thickness
# A small receptacle's tongue: (half width, z of the bottom face, z of the top face, distance of
# the tip behind the front) and its contact rows: (count, pitch, contact width, on the top face).
Tongue = Tuple[float, float, float, float]
ContactRow = Tuple[int, float, float, bool]


def side_profile(points: Sequence[Point], y0: float, y1: float) -> cq.Shape:
    """Return an (x, z) outline extruded across y from y0 to y1."""
    return cq.Workplane('XZ', origin=(0, y1, 0)).polyline(list(points)).close() \
        .extrude(y1 - y0).val()


def bent_strip(path: Sequence[Point], thickness: float, y: float, width: float) -> cq.Shape:
    """Return a stamped contact: a strip along an (x, z) path, its thickness above the path."""
    outline = list(path) + [(x, z + thickness) for x, z in reversed(path)]
    return side_profile(outline, y - width / 2, y + width / 2)


def tongue_block(x0: float, y0: float, z0: float, x1: float, y1: float, z1: float,
                 chamfer: float) -> cq.Shape:
    """Return a connector tongue: a block with chamfered edges at its tip (x1)."""
    return cq.Workplane('XY').box(x1 - x0, y1 - y0, z1 - z0, centered=False) \
        .translate((x0, y0, z0)).faces('>X').edges('|Y').chamfer(chamfer).val()


def contact_rows(rows: Sequence[ContactRow], tongue: Tongue, length: float,
                 centre_y: float = 0.0) -> List[Part]:
    """Return gold contacts on a tongue's faces, each row centred on centre_y."""
    half, z0, z1, tip = tongue
    parts: List[Part] = []
    for count, pitch, width, on_top in rows:
        for k in range(count):
            y = centre_y + (k - (count - 1) / 2) * pitch
            if abs(y) + width / 2 > half:
                continue
            z = (z1, z1 + 0.04) if on_top else (z0 - 0.04, z0)
            parts.append(('gold', box(-tip - length, y - width / 2, z[0], -tip - 0.15,
                                      y + width / 2, z[1])))
    return parts


def spring_finger(x_front: float, y: float, z_top: float, length: float = 3.2,
                  width: float = 1.4) -> Tuple[cq.Shape, cq.Shape]:
    """Return the cut and the bent-in tab of a stamped spring finger in a shell's top face."""
    slot = 0.3
    cut = box(x_front - length - slot, y - width / 2 - slot, z_top - 0.35, x_front,
              y + width / 2 + slot, z_top + 0.1)
    tab = box(x_front - length - slot, y - width / 2, z_top - 0.35, x_front - 0.6,
              y + width / 2, z_top - 0.15)
    return cut, tab


def usb_a_port(bottom: float, tongue: str) -> List[Part]:
    """Return one USB Type-A port's inside: rear housing, tongue, contacts and latch dimples.

    The tongue hangs from the top of the opening. The four USB 2.0 contacts are springs under
    it that bulge 0.35 mm below its face, a USB 3.0 tongue also has five flat SuperSpeed
    contacts near its tip.
    """
    width, height = USB_A_OPENING
    tongue_width, thickness, gap = USB_A_TONGUE
    top = bottom + height
    underside = top - gap - thickness
    rear = -USB_A_PORT_DEPTH + 0.3
    parts: List[Part] = [
        ('black_plastic', box(-USB_A_PORT_DEPTH, -width / 2, bottom, rear, width / 2, top)),
        (tongue, tongue_block(rear, -tongue_width / 2, underside, -0.3, tongue_width / 2,
                              top - gap, 0.3))]
    ys, contact_width, reach = USB2_CONTACTS
    path = [(-8.8, underside + 0.02), (-6.6, underside + 0.02), (-5.4, underside - reach),
            (-4.6, underside - reach), (-3.6, underside - 0.1)]
    parts += [('gold', bent_strip(path, 0.12, y, contact_width)) for y in ys]
    if tongue == 'usb3_blue':
        count, pitch, pad_width, length = USB3_CONTACTS
        for k in range(count):
            y = (k - (count - 1) / 2) * pitch
            parts.append(('gold', box(-0.3 - length, y - pad_width / 2, underside - 0.04, -0.6,
                                      y + pad_width / 2, underside + 0.02)))
    # Two dimples in the bottom wall hold the plug.
    for y in (-3.0, 3.0):
        parts.append(('nickel', box(-5.0, y - 0.6, bottom - 0.01, -4.0, y + 0.6,
                                    bottom + 0.3)))
    return parts


def usb_a_stack(width: float, depth: float, height: float, tongue: str,
                flange: float = 0.3, ears: float = 0.35) -> List[Part]:
    """Return a stacked double USB Type-A receptacle: nickel shell, two ports with insides.

    The shell has a front frame with side ears, a metal wall between the ports, two spring
    fingers per port in the top, dimples and side snap recesses. Inside, the metal walls show
    up to the rear housing.
    """
    half = width / 2
    shell = rounded_box(-depth, -half, 0.0, -flange, half, height, vertical=0.4)
    frame = box(-flange, -half - ears, 0.0, 0.0, half + ears, height)
    opening_width, opening_height = USB_A_OPENING
    parts: List[Part] = []
    for bottom in USB_A_PORT_BOTTOMS:
        port = box(-USB_A_PORT_DEPTH, -opening_width / 2, bottom, 0.1, opening_width / 2,
                   bottom + opening_height)
        shell = shell.cut(port)
        frame = frame.cut(port)
        parts += usb_a_port(bottom, tongue)
    for y in (-3.2, 3.2):
        cut, tab = spring_finger(-1.2, y, height)
        shell = shell.cut(cut)
        parts.append(('nickel', tab))
    for x, y in ((-8.0, -2.5), (-8.0, 2.5)):
        shell = shell.cut(cylinder(0.55, (x, y, height - 0.2), (0, 0, 1), 0.3))
    for side in (-1, 1):
        for bottom in USB_A_PORT_BOTTOMS:
            y0 = side * half - (0.2 if side > 0 else 0.0)
            shell = shell.cut(box(-7.5, y0, bottom + 1.0, -4.5, y0 + 0.2, bottom + 3.5))
    return [('nickel', shell), ('nickel', frame)] + parts


def rj45_jack(width: float, depth: float, height: float, led_protrusion: float = 0.1) -> \
        List[Part]:
    """Return an RJ45 jack with magnetics: rounded shield, plug opening, contacts, LEDs.

    The opening is black plastic inside, with the latch notch at the bottom. Eight gold spring
    contacts come down from its top at the front and run back at about 23 degrees, as in a
    jack without a plug.
    """
    half = width / 2
    shield = cq.Workplane('XY').box(depth, width, height, centered=False) \
        .translate((-depth, -half, 0.0)).edges('|Z').fillet(0.5) \
        .faces('>Z').edges('>X').fillet(1.0).val()
    bottom = 2.2
    profile = [(-5.85, bottom + 1.6), (-3.0, bottom + 1.6), (-3.0, bottom), (3.0, bottom),
               (3.0, bottom + 1.6), (5.85, bottom + 1.6), (5.85, bottom + 9.0),
               (-5.85, bottom + 9.0)]
    opening, liner = opening_with_liner(profile, 12.0, 0.3)
    shield = shield.cut(opening)
    for side in (-1, 1):
        y0 = side * half - (0.25 if side > 0 else 0.0)
        shield = shield.cut(box(-6.0, y0, 1.0, -3.0, y0 + 0.25, height - 3.0))
    parts: List[Part] = [('nickel', shield), ('black_plastic', liner)]
    count, pitch, contact_width, thickness = RJ45_CONTACTS
    ceiling = bottom + 9.0 - 0.3
    path = [(-1.4, ceiling - thickness), (-2.2, ceiling - 0.9), (-8.0, ceiling - 3.4),
            (-9.0, ceiling - 3.4)]
    for k in range(count):
        parts.append(('gold', bent_strip(path, thickness, (k - (count - 1) / 2) * pitch,
                                         contact_width)))
    # The comb that guides the contacts' ends
    parts.append(('black_plastic', box(-11.7, -5.55, ceiling - 4.0, -8.6, 5.55,
                                       ceiling - 3.45)))
    parts.append(('led_green', box(-0.05, -half + 0.8, 0.8, led_protrusion, -half + 2.8, 2.0)))
    parts.append(('led_yellow', box(-0.05, half - 2.8, 0.8, led_protrusion, half - 0.8, 2.0)))
    return parts


def small_receptacle(width: float, depth: float, height: float, opening: Sequence[Point],
                     opening_depth: float, round_top: float, tongue: Tongue,
                     rows: Sequence[ContactRow], contact_length: float) -> List[Part]:
    """Return a small metal receptacle (micro-HDMI, micro-USB) facing +x.

    The shell's metal shows inside the opening up to the black insulator at opening_depth,
    from which the tongue reaches forward with its gold contact rows.
    """
    half = width / 2
    shell = cq.Workplane('XY').box(depth, width, height, centered=False) \
        .translate((-depth, -half, 0.0))
    if round_top:
        shell = shell.edges('|X and >Z').fillet(round_top)
    cut = front_profile(opening, 0.1, opening_depth + 0.1)
    insulator = front_profile(opening, -opening_depth + 0.25, 0.25)
    tongue_half, z0, z1, tip = tongue
    parts: List[Part] = [('nickel', shell.val().cut(cut)), ('black_plastic', insulator),
                         ('black_plastic', tongue_block(-opening_depth + 0.25, -tongue_half, z0,
                                                        -tip, tongue_half, z1, 0.1))]
    return parts + contact_rows(rows, tongue, contact_length)


# ---------------------------------------------------------------------------------------------
# Small surface-mount parts
# ---------------------------------------------------------------------------------------------
def two_terminal(points: Sequence[Point], z0: float, z1: float, body: str) -> List[Part]:
    """Return a chip resistor or capacitor: body and tin end caps along its long side."""
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
    if x1 - x0 >= y1 - y0:
        cap = 0.22 * (x1 - x0)
        return [(body, box(x0 + cap, y0, z0, x1 - cap, y1, z1)),
                ('tin', box(x0, y0, z0, x0 + cap, y1, z1)),
                ('tin', box(x1 - cap, y0, z0, x1, y1, z1))]
    cap = 0.22 * (y1 - y0)
    return [(body, box(x0, y0 + cap, z0, x1, y1 - cap, z1)),
            ('tin', box(x0, y0, z0, x1, y0 + cap, z1)),
            ('tin', box(x0, y1 - cap, z0, x1, y1, z1))]


def quad_package(x0: float, y0: float, x1: float, y1: float, z0: float, height: float,
                 pitch: float = 0.5) -> List[Part]:
    """Return a QFN-like chip: black body with a chamfered top and tin pads along its edges."""
    inset = 0.15
    body = rounded_box(x0 + inset, y0 + inset, z0, x1 - inset, y1 - inset, z0 + height,
                       chamfer_top=0.1)
    parts: List[Part] = [('chip', body)]
    for along_x in (True, False):
        length = (x1 - x0) if along_x else (y1 - y0)
        count = max(int((length - 1.0) / pitch), 0)
        start = -(count - 1) * pitch / 2
        for k in range(count):
            offset = start + k * pitch
            for side in (0, 1):
                if along_x:
                    x = (x0 + x1) / 2 + offset
                    y = y0 if side == 0 else y1 - inset
                    parts.append(('tin', box(x - 0.12, y, z0, x + 0.12, y + inset, z0 + 0.12)))
                else:
                    y = (y0 + y1) / 2 + offset
                    x = x0 if side == 0 else x1 - inset
                    parts.append(('tin', box(x, y - 0.12, z0, x + inset, y + 0.12, z0 + 0.12)))
    return parts


# ---------------------------------------------------------------------------------------------
# Boards: solder mask on the faces, FR-4 on the cut edges, plating in the holes
# ---------------------------------------------------------------------------------------------
def split_board(surface: Surface, mask: str, plated_holes: Sequence[Point] = (),
                plated_radius: float = 0.0) -> List[Tuple[str, Surface]]:
    """Return a board's surface split into solder mask (faces), FR-4 (edges) and gold (holes).

    Triangles facing up or down are solder mask. The others are the cut edges, except those
    within plated_radius of a plated hole's axis, which are the hole's plating.
    """
    corners = surface.vertices[surface.triangles]
    normals = np.cross(corners[:, 1] - corners[:, 0], corners[:, 2] - corners[:, 0])
    lengths = np.linalg.norm(normals, axis=1)
    flat = np.abs(normals[:, 2]) > 0.9 * np.where(lengths > 0, lengths, 1.0)
    centres = corners.mean(axis=1)[:, :2]
    plated = np.zeros(len(corners), dtype=bool)
    for x, y in plated_holes:
        plated |= np.hypot(centres[:, 0] - x, centres[:, 1] - y) < plated_radius
    groups = ((mask, flat), ('gold', ~flat & plated), ('fr4_edge', ~flat & ~plated))
    return [(name, Surface(surface.vertices, surface.normals, surface.triangles[selected]))
            for name, selected in groups if selected.any()]


def drop_degenerate(surface: Surface) -> Surface:
    """Return the surface without zero-area triangles.

    Where fillets meet, OCCT's triangulation can leave triangles whose corners coincide.
    They show nothing but make a closed shell look open to mesh checks.
    """
    corners = surface.vertices[surface.triangles]
    areas = np.linalg.norm(np.cross(corners[:, 1] - corners[:, 0], corners[:, 2] - corners[:, 0]),
                           axis=1)
    return Surface(surface.vertices, surface.normals, surface.triangles[areas > 1e-9])


def fin_polygon(root: Point, tip: Point, thickness: float) -> List[Point]:
    """Return the outline of a straight fin of the given thickness from root to tip."""
    dx, dy = tip[0] - root[0], tip[1] - root[1]
    length = math.hypot(dx, dy)
    px, py = -dy / length * thickness / 2, dx / length * thickness / 2
    return [(root[0] - px, root[1] - py), (tip[0] - px, tip[1] - py),
            (tip[0] + px, tip[1] + py), (root[0] + px, root[1] + py)]
