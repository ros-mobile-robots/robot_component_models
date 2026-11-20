# SPDX-FileCopyrightText: 2026 ROS Mobile Robots
# SPDX-License-Identifier: BSD-3-Clause
"""Generate the DG01D-E gear motor with its hall encoder: gearbox, motor housing, encoder cover.

Modelled from SparkFun's drawing DS-16413 ("DG01D-E motor with encoder", page 1: a view of the
shaft's side and a view of the motor's side, with their dimensions; page 2: the encoder board's
size), from measurements of the motor made for Remo's design, whose chassis holds the motor
in a pocket, and from a motor measured with a caliper. SparkFun's product photos of ROB-16413
give the look: the light blue plastic housing, the metal output shaft with two flats, the
screws, holes and vents, and the encoder's connector. No logos and no printed text. Sources in
provenance/dg01d_e_gearmotor.md.

Frame (positions in mm, the files are in metres): x along the motor, from the encoder's end at
x = -39.725 to the mounting tab's end at x = +39.975. y along the output shaft, which stands
out of the gearbox's +y face. z across the gearbox's 22.4 mm height. The gearbox's middle
planes are y = 0 and z = 0. The output shaft's axis is at x = 23.765, z = 0.
"""
import math
from typing import Dict
from typing import List
from typing import NamedTuple
from typing import Sequence
from typing import Tuple

import cadquery as cq

from generators import board_parts
from generators.board_parts import box
from generators.board_parts import cylinder
from generators.mesh_export import Model
from generators.mesh_export import phong
from generators.mesh_export import require_known_materials
from generators.mesh_export import SubPart
from generators.mesh_export import tessellate_faces

TITLE = 'Gear motor with hall encoder, DG01D-E size'


class Span(NamedTuple):
    """An interval along one axis (mm)."""

    low: float
    high: float


class Hole(NamedTuple):
    """A blind round hole in a face of the gearbox: its centre, diameter and depth (mm)."""

    x: float
    z: float
    diameter: float
    depth: float


class Slot(NamedTuple):
    """A pair of vents in a face of the motor housing, mirrored about z = 0 (mm).

    x and z are the centre of the vent on the +z side, length runs along x, width along z.
    """

    x: float
    z: float
    length: float
    width: float


class Cross(NamedTuple):
    """A cross recess in a screw's head: each slot's length and width, and its depth (mm)."""

    length: float
    width: float
    depth: float


class Recess(NamedTuple):
    """A shallow rectangular recess with rounded corners in a face (mm)."""

    x: Span
    half_height: float
    depth: float
    corner: float


# ---------------------------------------------------------------------------------------------
# Parameters, in mm, the source next to each value (provenance/dg01d_e_gearmotor.md):
#   DS        a dimension of SparkFun's drawing DS-16413, page 1
#   DS p2     the drawing's page 2, the encoder board's specification
#   DS scale  read off the drawing's scale, where it gives no dimension
#   Remo      measurements of the motor made for Remo's design, whose chassis holds the motor
#   caliper   measured on the motor with a caliper
#   motor     the shape of the motor measured with the caliper, where the caliper gives no
#             value
#   photo N   SparkFun's product photo N of ROB-16413, scaled with the drawing's dimensions
#   choice    not measured: a value that looks like the photos
#   derived   follows from other values in this file
# x runs along the motor, y along the output shaft, z across the gearbox (module docstring).
# ---------------------------------------------------------------------------------------------
DRAWING_LENGTH = 79.95       # DS: the overall length, from the encoder's end to the tab's end
TAB_END_X = DRAWING_LENGTH / 2  # derived: x = 0 halves the drawing's overall length, 39.975
GEARBOX_END_X = -DRAWING_LENGTH / 2 + 75.15  # DS: 75.15 from the encoder's end, 35.175
# The motor measured with the caliper is 0.25 shorter than the drawing at the encoder's end:
# its encoder cover ends closer to the gearbox, which stays where the drawing puts it
ENCODER_END_X = GEARBOX_END_X - 74.90  # caliper: 74.90 to the gearbox's end, -39.725

# Gearbox: a block with two round corners at the tab's end
GEARBOX_BACK_X = -1.675      # Remo (DS scale -1.66): the face towards the motor housing
GEARBOX_THICKNESS = 18.75    # DS: along y
GEARBOX_HEIGHT = 22.40       # DS (caliper 22.35): along z
GEARBOX_END_ROUND = 5.0      # Remo (DS scale 4.8): the two corners at the tab's end
GEARBOX_EDGE_ROUND = 0.3     # Remo: the edges around the +y, -y and back faces

# Features on the gearbox's faces. The drawing chains their x from the boss's centre
BOSS_X = 12.765              # Remo: the round boss on the shaft's side
SHAFT_X = BOSS_X + 11.00     # DS: 11.00 from the boss, 23.765
MOUNT_HOLE_X = BOSS_X - 9.00  # DS: 9.00 from the boss, 3.765
TAB_HOLE_X = BOSS_X + 24.81  # DS: 24.81 from the boss, 37.575
MOUNT_HOLE_Z = 8.7           # Remo (DS scale 8.9): +-, where the chassis' screw holes are
MOUNT_HOLE_DIAMETER = 3.08   # DS (caliper 3.16): two holes through the gearbox
BOSS_DIAMETER = 3.98         # DS (caliper 3.93)
BOSS_HEIGHT = 1.77           # DS: above the +y face
BOSS_TOP_ROUND = 0.7         # Remo: the top edge
BOSS_DIMPLE = Hole(BOSS_X, 0.0, 0.8, 0.5)  # photo 02: a small hole in the top; size: choice
# Two screws hold the gearbox together, sunk into round recesses on the +y face
SCREW_X = 1.07               # DS scale
SCREW_Z = 6.29               # DS scale: +-
SCREW_RECESS_DIAMETER = 4.7  # DS scale
SCREW_RECESS_DEPTH = 1.5     # choice
SCREW_HEAD_DIAMETER = 3.4    # photo 02: about 0.73 of the recess
SCREW_HEAD_SINK = 0.5        # choice: the head's top below the face
SCREW_HEAD_ROUND = 0.3       # choice
SCREW_CROSS = Cross(2.0, 0.5, 0.5)  # choice
# Small blind holes: on the +y face from the drawing (the diameter inside its circles' lines),
# on the -y face from photo 03
SHAFT_SIDE_HOLES = (Hole(16.83, 9.78, 1.4, 2.0),    # DS scale; depth: choice
                    Hole(32.74, 8.45, 1.4, 2.0))    # DS scale; depth: choice
BACK_SIDE_HOLES = (Hole(32.74, 8.45, 1.4, 2.0),     # photo 03 (32.5, 8.6): as on the +y face
                   Hole(0.13, 9.4, 2.3, 2.0))       # photo 03; depth: choice
# photo 03: a shallow recess between the mounting holes on the -y face, a dot in it
BACK_RECESS = Recess(Span(-0.47, 6.62), 6.05, 0.3, 1.0)  # photo 03; depth and corners: choice
BACK_RECESS_DOT = Hole(3.3, 0.0, 0.6, 0.5)  # photo 03; size: choice

# Output shaft: metal, with two flats that face +z and -z, a bore in its end
SHAFT_DIAMETER = 5.40        # DS
SHAFT_FLATS = 3.70           # DS: across the flats
SHAFT_LENGTH = 8.65          # DS (caliper 8.65): past the gearbox's +y face
SHAFT_BORE_DIAMETER = 1.8    # Remo (DS scale 1.8)
SHAFT_BORE_DEPTH = 6.0       # choice
SHAFT_END_CHAMFER = 0.2      # choice
SHAFT_ROOT = 0.4             # choice: the shaft starts this far inside the gearbox
COLLAR_DIAMETER = 7.0        # DS scale: the ring around the shaft's foot
COLLAR_HEIGHT = 0.3          # DS scale

# Mounting tab at the gearbox's end, centred on the gearbox's middle planes
TAB_HEIGHT = 5.00            # DS (caliper 5.00): along z
TAB_THICKNESS = 2.80         # DS: along y. Centred: Remo (DS scale 0.57 towards -y)
TAB_END_ROUND = 2.0          # Remo: the two corners at its end
TAB_EDGE_ROUND = 0.3         # Remo: the edges around its +y and -y faces
TAB_HOLE_DIAMETER = 2.99     # DS (caliper 2.99)
TAB_ROOT = 1.0               # choice: the tab starts this far inside the gearbox
# Pieces of different sub-parts overlap where they join, so that no two faces lie on each
# other: the hidden part stays inside the neighbour
OVERLAP = 0.3                # choice
# A piece that the union adds to a body starts this far inside it, so the union joins them
SINK = 0.1                   # choice
# A cutter reaches this far past the face that it cuts, so the cut leaves no skin
CUTTER_REACH = 1.0           # choice
# A straight piece of an outline is at least this long, or the outline leaves it out
SHORTEST_STRAIGHT = 0.01     # choice
# Two lengths that differ by less than this count as equal: the floating point roundoff of
# adding and subtracting the values in this file
ROUNDOFF = 1e-9              # choice

# Neck between the gearbox and the motor housing, with a fin on each side: round, cut by two
# flats on its +y and -y sides
NECK_THICKNESS = 15.95       # Remo: along y, across the flats
NECK_HEIGHT = GEARBOX_HEIGHT  # DS: the top view's lines run on from the gearbox. The diameter
FIN_FRONT_X = BOSS_X - 19.19  # DS (caliper 19.04): 19.19 from the boss's centre, -6.425
FIN_BASE_LENGTH = 2.90       # Remo (DS scale 3.1): along x, at the neck
FIN_TOP_LENGTH = 2.30        # Remo (DS scale 2.2): along x, at the top
FIN_SLOPE_TOP_Y = 10.215     # Remo: the back face slopes up to here, then rises straight
FIN_TOP_Y = 11.175           # Remo (caliper 22.18 across both): from the middle plane, both sides
FIN_HEIGHT = 5.30            # Remo (DS scale 5.2): along z

# Motor housing: round, cut by two flats with the vents on its +y and -y sides, and a chamfer
# down to the neck. The motor's can inside shows through the vents
HOUSING_START_X = -32.975    # Remo (DS scale -33.08): the end at the encoder
HOUSING_LENGTH = 22.30       # DS (caliper 22.29): to the end of the chamfer, where the neck starts
HOUSING_THICKNESS = 18.75    # Remo: along y, across the flats, as thick as the gearbox
HOUSING_HEIGHT = 25.20       # DS: along z. The diameter: the flats end at z = +-8.42 (DS scale 8)
HOUSING_END_ROUND = 0.3      # Remo: the edges of the end at the encoder
HOUSING_CHAMFER = (HOUSING_HEIGHT - NECK_HEIGHT) / 2  # derived: 45 degrees down to the neck, 1.4
SHAFT_SIDE_VENTS = Slot(-17.98, 3.57, 7.25, 1.9)  # DS scale
BACK_SIDE_VENTS = Slot(-15.7, 3.6, 3.0, 1.6)  # photo 03
VENT_DEPTH = 1.2             # choice: the motor's can shows at the bottom
MOTOR_CAN_PLATE = 0.2        # choice: the can's visible face, above the vents' bottom
MOTOR_CAN_MARGIN = 0.1       # choice: the can's plates reach this far under the vent's edge

# Encoder cover at the motor housing's end, shaped like a D across the motor's axis: a round
# part on the +y side, and a narrower box towards -y that holds the connector and reaches on
# under the motor housing. A window for the connector in the box's -y face
ENCODER_TOP_Y = 7.975        # Remo: the +y face, flush with the neck's
ENCODER_DEPTH = 25.79        # DS (caliper 25.78): along y, so the -y face is at -17.815
ENCODER_HEIGHT = 19.20       # DS: along z, the round part
ENCODER_TOP_EDGE_ROUND = 8.0  # motor: the round part's two corners, a flat strip of 3.2 between
ENCODER_ROUND_LENGTH = 10.5  # motor: from the +y face to where the round part's +-z faces end
ENCODER_STEP_LENGTH = 3.0    # motor: along y, an S-curve from there down to the box's faces
ENCODER_BOTTOM_EDGE_ROUND = 0.3  # Remo: the long edges on the -y side
ENCODER_END_EDGE_ROUND = 0.3  # Remo: around the end face
CONNECTOR_BOX_HEIGHT = 16.82  # caliper: along z
CONNECTOR_BOX_END_X = ENCODER_END_X + 13.38  # caliper: 13.38 from the cover's end, -26.345
WINDOW_X = Span(-34.475, -27.475)  # Remo
WINDOW_HALF_HEIGHT = 7.6     # motor: the box's sides stay about 0.8 wide
WINDOW_FLOOR_Y = -9.815      # Remo: the window's bottom, inside the cover
LATCH_X = Span(CONNECTOR_BOX_END_X - 0.5, -20.1)  # photo 03: on the motor housing's -y face
LATCH_HALF_HEIGHT = 4.0      # photo 03
LATCH_THICKNESS = 0.5        # choice

# Encoder board and its six-pin connector, seen through the window
BOARD_THICKNESS = 1.2        # DS p2: the board is 26 x 22 x 1.2
CONTACTS = 6                 # DS p2: G, H1, H2, V, M+, M-; photo 03: six contacts
CONTACT_PITCH = 2.0          # choice: a 2.0 mm pitch header (photo 03: about 2.1)
CONNECTOR_CENTRE_X = -32.1   # photo 03
CONNECTOR_DEPTH = 4.5        # photo 03 (4.6), along x
CONNECTOR_HEIGHT = 6.0       # choice: above the board
CONNECTOR_END_WALL = 2.4     # photo 03: the connector's body is about 14.8 long along z
CONNECTOR_WALL = 0.7         # choice
CONNECTOR_CAVITY_DEPTH = 4.3  # choice
CONTACT_SIZE = 0.64          # choice: square contacts
CONTACT_SINK = 0.5           # choice: their ends below the opening
PIN_END_X = -28.6            # photo 03: a row of pin ends beside the connector
PIN_END_SIZE = 0.6           # choice
PIN_END_HEIGHT = 1.0         # choice

LINEAR_DEFLECTION = 0.02
ANGULAR_DEFLECTION = 0.3
AMBIENT_FACTOR = board_parts.AMBIENT_FACTOR

MATERIALS = {
    # photos: the light blue plastic of the gearbox, the motor housing and the encoder cover
    'blue_plastic': phong((0.50, 0.78, 0.94, 1.0), (0.25, 0.25, 0.25, 1.0), 20.0,
                          ambient_factor=AMBIENT_FACTOR),
    # photos 02 and 05: the bright metal of the shaft and the screws
    'steel': phong((0.78, 0.78, 0.80, 1.0), (0.70, 0.70, 0.70, 1.0), 40.0, metallic=True,
                   ambient_factor=AMBIENT_FACTOR),
    # photo 02: the motor's can, grey metal behind the vents
    'motor_can': phong((0.36, 0.37, 0.40, 1.0), (0.60, 0.60, 0.60, 1.0), 40.0, metallic=True,
                       ambient_factor=AMBIENT_FACTOR),
    **board_parts.materials(['solder_mask_green', 'white_plastic', 'tin']),
}

Pieces = Dict[str, List[cq.Shape]]


# ---------------------------------------------------------------------------------------------
# Checks of the parameters
# ---------------------------------------------------------------------------------------------
def check_housing_chamfer_meets_neck() -> None:
    """Raise a ValueError if the housing's 45 degree chamfer doesn't end on the neck's section.

    On the round sides the chamfer takes the housing's diameter down to the neck's diameter. On
    the flats it has to take the housing's thickness down to the neck's thickness by the same
    amount.
    """
    thickness_step = (HOUSING_THICKNESS - NECK_THICKNESS) / 2
    if abs(thickness_step - HOUSING_CHAMFER) > ROUNDOFF:
        raise ValueError(f'the chamfer is {HOUSING_CHAMFER:.3f} mm along z but the neck needs '
                         f'{thickness_step:.3f} mm along y')


def flat_half_width(diameter: float, thickness: float) -> float:
    """Return half the width (along z) of the flats that cut a round part down to a thickness."""
    return math.sqrt((diameter / 2) ** 2 - (thickness / 2) ** 2)


def check_connector_box_under_flat() -> None:
    """Raise a ValueError if the connector's box is wider than the motor housing's -y flat.

    The box reaches into the housing through the flat, so a wider box would stand out of the
    housing's round sides.
    """
    flat = flat_half_width(HOUSING_HEIGHT, HOUSING_THICKNESS)
    if CONNECTOR_BOX_HEIGHT / 2 > flat:
        raise ValueError(f"the connector's box is {CONNECTOR_BOX_HEIGHT:.2f} mm wide, the motor "
                         f"housing's flat {2 * flat:.2f} mm")


def check_encoder_cover_outline() -> None:
    """Raise a ValueError if the pieces of the encoder cover's D-shaped outline don't fit.

    Along y from the +y face come the round part's corners, its +-z faces, the S-curve down to
    the narrower box and the box's faces, which end in the rounds of its -y end. The round
    part's +-z faces and the box's faces need a straight piece each, SHORTEST_STRAIGHT or
    longer. The flat strip between the corners is as long or left out: then the round part is
    a semicircle. Lengths within ROUNDOFF of a limit count as the limit.
    """
    shortest = SHORTEST_STRAIGHT - ROUNDOFF
    flat_strip = ENCODER_HEIGHT - 2 * ENCODER_TOP_EDGE_ROUND
    if flat_strip < -ROUNDOFF:
        raise ValueError("the encoder cover's corners are larger than half its height")
    if ROUNDOFF < flat_strip < shortest:
        raise ValueError("the encoder cover's flat strip between its corners is too short")
    if ENCODER_ROUND_LENGTH - ENCODER_TOP_EDGE_ROUND < shortest:
        raise ValueError("the encoder cover's round part needs straight +-z faces past its "
                         'corners')
    box_faces = (ENCODER_DEPTH - ENCODER_ROUND_LENGTH - ENCODER_STEP_LENGTH
                 - ENCODER_BOTTOM_EDGE_ROUND)
    if box_faces < shortest:
        raise ValueError("the encoder cover's box needs straight faces between the S-curve and "
                         'its -y end')
    if CONNECTOR_BOX_HEIGHT >= ENCODER_HEIGHT:
        raise ValueError("the connector's box isn't narrower than the encoder cover's round "
                         'part')


def inside_gearbox_face(x: float, z: float, radius: float) -> bool:
    """Return whether a circle at (x, z) lies on a face of the gearbox, inside its edge rounds.

    The faces' outline is the gearbox's block with the two round corners at the tab's end. The
    circle has to keep GEARBOX_EDGE_ROUND from the outline, where the edges are rounded.
    """
    margin = radius + GEARBOX_EDGE_ROUND
    half_height = GEARBOX_HEIGHT / 2
    if not GEARBOX_BACK_X + margin <= x <= GEARBOX_END_X - margin:
        return False
    if abs(z) > half_height - margin:
        return False
    corner_x = GEARBOX_END_X - GEARBOX_END_ROUND
    corner_z = half_height - GEARBOX_END_ROUND
    if x > corner_x and abs(z) > corner_z:
        return math.hypot(x - corner_x, abs(z) - corner_z) <= GEARBOX_END_ROUND - margin
    return True


def check_screw_recesses_inside_face() -> None:
    """Raise a ValueError if a screw's recess leaves the gearbox's +y face."""
    if not inside_gearbox_face(SCREW_X, SCREW_Z, SCREW_RECESS_DIAMETER / 2):
        raise ValueError("a screw's recess reaches past the gearbox's +y face")


def check_holes_inside_face() -> None:
    """Raise a ValueError if a hole or the -y recess leaves the gearbox's face.

    The holes are the mounting holes and the small holes on both faces. The recess has to stay
    clear of the mounting holes too.
    """
    holes = [Hole(MOUNT_HOLE_X, MOUNT_HOLE_Z, MOUNT_HOLE_DIAMETER, GEARBOX_THICKNESS)]
    for hole in holes + list(SHAFT_SIDE_HOLES + BACK_SIDE_HOLES):
        if not inside_gearbox_face(hole.x, hole.z, hole.diameter / 2):
            raise ValueError(f'the hole at (x, z) = ({hole.x}, {hole.z}) mm reaches past the '
                             "gearbox's face")
    heights = (-BACK_RECESS.half_height, BACK_RECESS.half_height)
    corners = [(x, z) for x in BACK_RECESS.x for z in heights]
    if not all(inside_gearbox_face(x, z, 0.0) for x, z in corners):
        raise ValueError("the -y recess reaches past the gearbox's face")
    if BACK_RECESS.half_height >= MOUNT_HOLE_Z - MOUNT_HOLE_DIAMETER / 2:
        raise ValueError('the -y recess reaches into the mounting holes')


def check_connector_inside_window() -> None:
    """Raise a ValueError if the connector or its row of pin ends leaves the window."""
    length = (CONTACTS - 1) * CONTACT_PITCH + 2 * CONNECTOR_END_WALL
    xs = (CONNECTOR_CENTRE_X - CONNECTOR_DEPTH / 2, CONNECTOR_CENTRE_X + CONNECTOR_DEPTH / 2)
    if xs[0] <= WINDOW_X.low or xs[1] >= PIN_END_X - PIN_END_SIZE / 2 or \
            PIN_END_X + PIN_END_SIZE / 2 >= WINDOW_X.high or length / 2 >= WINDOW_HALF_HEIGHT:
        raise ValueError("the connector doesn't fit into the encoder cover's window")
    if WINDOW_FLOOR_Y - BOARD_THICKNESS - CONNECTOR_HEIGHT <= -(ENCODER_DEPTH - ENCODER_TOP_Y):
        raise ValueError("the connector stands out of the encoder cover's -y face")


def check_window_inside_box() -> None:
    """Raise a ValueError if the connector's window cuts through the sides or the ends of its box.

    The encoder board's piece around the window reaches OVERLAP past the window's edges, and
    has to stay hidden inside the box as well. At the cover's end it keeps clear of the round of
    the end face's edges, so its corners stay inside that round too.
    """
    if WINDOW_HALF_HEIGHT + OVERLAP >= CONNECTOR_BOX_HEIGHT / 2 or \
            WINDOW_X.high + OVERLAP >= CONNECTOR_BOX_END_X or \
            WINDOW_X.low - OVERLAP <= ENCODER_END_X + ENCODER_END_EDGE_ROUND:
        raise ValueError("the connector's window cuts through its box")


def check_shaft() -> None:
    """Raise a ValueError if the flats or the bore don't fit the shaft."""
    if not SHAFT_BORE_DIAMETER < SHAFT_FLATS < SHAFT_DIAMETER < COLLAR_DIAMETER:
        raise ValueError('the shaft needs bore < flats < diameter < collar')
    if SHAFT_BORE_DEPTH >= SHAFT_LENGTH:
        raise ValueError("the shaft's bore is deeper than the shaft is long")


def check_fins_on_neck() -> None:
    """Raise a ValueError if a fin's foot leaves the neck's flat."""
    neck_start = HOUSING_START_X + HOUSING_LENGTH
    if FIN_FRONT_X - FIN_BASE_LENGTH <= neck_start or FIN_FRONT_X >= GEARBOX_BACK_X or \
            FIN_HEIGHT / 2 > flat_half_width(NECK_HEIGHT, NECK_THICKNESS):
        raise ValueError("a fin's foot reaches past the neck's flat")


PARAMETER_CHECKS = (check_housing_chamfer_meets_neck, check_connector_box_under_flat,
                    check_encoder_cover_outline, check_screw_recesses_inside_face,
                    check_holes_inside_face, check_connector_inside_window,
                    check_window_inside_box, check_shaft, check_fins_on_neck)


def check_parameters() -> None:
    """Run the checks of the parameters, each raising a ValueError for a wrong value."""
    for check in PARAMETER_CHECKS:
        check()


# ---------------------------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------------------------
def solid(workplane: cq.Workplane) -> cq.Shape:
    """Return the single solid of a workplane."""
    return workplane.val()


def along_y(x: float, z: float, radius: float, y0: float, y1: float) -> cq.Shape:
    """Return a cylinder along y from y0 to y1, its axis at (x, z)."""
    return cylinder(radius, (x, y0, z), (0, 1, 0), y1 - y0)


def face_hole(hole: Hole, face_y: float, outward: int) -> cq.Shape:
    """Return the cutter of a blind hole in the face at face_y, whose outside is towards outward.

    outward is +1 for a face that looks towards +y, -1 for one that looks towards -y.
    """
    inner = face_y - outward * hole.depth
    outer = face_y + outward * CUTTER_REACH
    return along_y(hole.x, hole.z, hole.diameter / 2, min(inner, outer), max(inner, outer))


def mirrored_z(shape: cq.Shape) -> List[cq.Shape]:
    """Return the shape and its mirror image about z = 0."""
    return [shape, shape.mirror('XY')]


def rounded_slot(slot: Slot, z: float, y0: float, y1: float) -> cq.Shape:
    """Return a vent's prism with round ends along y, centred at (slot.x, z)."""
    return solid(cq.Workplane('XZ', origin=(0, y1, 0)).center(slot.x, z)
                 .slot2D(slot.length, slot.width, 0).extrude(y1 - y0))


def round_face_edges(shape: cq.Workplane, selectors: Sequence[str],
                     radius: float) -> cq.Shape:
    """Return the shape with the edges of the selected faces rounded, each edge once.

    The edges are collected face by face in the selectors' order. CadQuery's "or" selector
    collects them through a set, whose order changes from run to run, and so would the order
    of the rounded shape's faces and the bytes of the mesh files.
    """
    edges: List[cq.Edge] = []
    for selector in selectors:
        for edge in shape.faces(selector).edges().vals():
            if not any(edge.isSame(other) for other in edges):
                edges.append(edge)
    return shape.val().fillet(radius, edges)


def union_all(shapes: Sequence[cq.Shape]) -> cq.Shape:
    """Return the union of the shapes."""
    result = shapes[0]
    for shape in shapes[1:]:
        result = result.fuse(shape)
    return result.clean()


def cut_all(shape: cq.Shape, cutters: Sequence[cq.Shape]) -> cq.Shape:
    """Return the shape with the cutters taken away, one after the other."""
    for cutter in cutters:
        shape = shape.cut(cutter)
    return shape.clean()


# ---------------------------------------------------------------------------------------------
# Gearbox, tab, boss and collar, and the neck with its fins
# ---------------------------------------------------------------------------------------------
def gearbox_block() -> cq.Shape:
    """Return the gearbox's block with its round end and its rounded face edges."""
    block = cq.Workplane('XY').add(box(GEARBOX_BACK_X, -GEARBOX_THICKNESS / 2,
                                       -GEARBOX_HEIGHT / 2, GEARBOX_END_X,
                                       GEARBOX_THICKNESS / 2, GEARBOX_HEIGHT / 2))
    block = block.edges('|Y').edges('>X').fillet(GEARBOX_END_ROUND)
    return round_face_edges(block, ('>Y', '<Y', '<X'), GEARBOX_EDGE_ROUND)


def tab() -> cq.Shape:
    """Return the mounting tab at the gearbox's end, with its hole."""
    plate = cq.Workplane('XY').add(box(GEARBOX_END_X - TAB_ROOT, -TAB_THICKNESS / 2,
                                       -TAB_HEIGHT / 2, TAB_END_X, TAB_THICKNESS / 2,
                                       TAB_HEIGHT / 2))
    plate = plate.edges('|Y').edges('>X').fillet(TAB_END_ROUND)
    plate = round_face_edges(plate, ('>Y', '<Y'), TAB_EDGE_ROUND)
    hole = along_y(TAB_HOLE_X, 0.0, TAB_HOLE_DIAMETER / 2, -TAB_THICKNESS, TAB_THICKNESS)
    return plate.cut(hole)


def flatted_rod(start: float, end: float, diameter: float, thickness: float) -> cq.Shape:
    """Return a round rod along x from start to end, cut by two flats thickness apart along y."""
    rod = cylinder(diameter / 2, (start, 0.0, 0.0), (1, 0, 0), end - start)
    flats = box(start - CUTTER_REACH, -thickness / 2, -diameter, end + CUTTER_REACH,
                thickness / 2, diameter)
    return rod.intersect(flats)


def neck() -> cq.Shape:
    """Return the neck from the motor housing's chamfer into the gearbox's back face.

    The neck is round and as high as the gearbox, cut by two flats. It reaches into the gearbox
    as far as the round of the gearbox's back edges, so it fills the groove that the round
    leaves next to it, and its top ends where the gearbox's top face starts.
    """
    return flatted_rod(HOUSING_START_X + HOUSING_LENGTH, GEARBOX_BACK_X + GEARBOX_EDGE_ROUND,
                       NECK_HEIGHT, NECK_THICKNESS)


def fin(side: int) -> cq.Shape:
    """Return the fin on the neck's +y (side 1) or -y (side -1) face.

    The front faces the gearbox and rises straight. The back slopes from the foot up to
    FIN_SLOPE_TOP_Y, then rises straight to the top.
    """
    base_y = NECK_THICKNESS / 2 - SINK
    outline = [(FIN_FRONT_X, base_y), (FIN_FRONT_X, FIN_TOP_Y),
               (FIN_FRONT_X - FIN_TOP_LENGTH, FIN_TOP_Y),
               (FIN_FRONT_X - FIN_TOP_LENGTH, FIN_SLOPE_TOP_Y),
               (FIN_FRONT_X - FIN_BASE_LENGTH, NECK_THICKNESS / 2),
               (FIN_FRONT_X - FIN_BASE_LENGTH, base_y)]
    shape = board_parts.prism(outline, -FIN_HEIGHT / 2, FIN_HEIGHT / 2)
    return shape if side > 0 else shape.mirror('XZ')


def boss() -> cq.Shape:
    """Return the round boss on the +y face, with a rounded top and a small hole in it."""
    face = GEARBOX_THICKNESS / 2
    peg = cq.Workplane('XY').add(along_y(BOSS_X, 0.0, BOSS_DIAMETER / 2, face - SINK,
                                         face + BOSS_HEIGHT))
    peg = solid(peg.faces('>Y').edges().fillet(BOSS_TOP_ROUND))
    return peg.cut(face_hole(BOSS_DIMPLE, face + BOSS_HEIGHT, 1))


def collar() -> cq.Shape:
    """Return the flat ring around the shaft's foot on the +y face."""
    face = GEARBOX_THICKNESS / 2
    return along_y(SHAFT_X, 0.0, COLLAR_DIAMETER / 2, face - SINK, face + COLLAR_HEIGHT)


def gearbox_cutters() -> List[cq.Shape]:
    """Return what the gearbox's faces lose: holes, screw recesses and the -y recess."""
    face = GEARBOX_THICKNESS / 2
    cutters = []
    for z in (MOUNT_HOLE_Z, -MOUNT_HOLE_Z):
        cutters.append(along_y(MOUNT_HOLE_X, z, MOUNT_HOLE_DIAMETER / 2, -face - CUTTER_REACH,
                               face + CUTTER_REACH))
    for z in (SCREW_Z, -SCREW_Z):
        cutters.append(along_y(SCREW_X, z, SCREW_RECESS_DIAMETER / 2, face - SCREW_RECESS_DEPTH,
                               face + CUTTER_REACH))
    for hole in SHAFT_SIDE_HOLES:
        cutters += mirrored_z(face_hole(hole, face, 1))
    for hole in BACK_SIDE_HOLES:
        cutters += mirrored_z(face_hole(hole, -face, -1))
    recess = cq.Workplane('XZ', origin=(0, -face + BACK_RECESS.depth, 0)) \
        .center((BACK_RECESS.x.low + BACK_RECESS.x.high) / 2, 0) \
        .rect(BACK_RECESS.x.high - BACK_RECESS.x.low, 2 * BACK_RECESS.half_height) \
        .extrude(BACK_RECESS.depth + CUTTER_REACH)
    cutters.append(solid(recess.edges('|Y').fillet(BACK_RECESS.corner)))
    cutters.append(face_hole(BACK_RECESS_DOT, -face + BACK_RECESS.depth, -1))
    return cutters


def make_gearbox() -> cq.Shape:
    """Return the gearbox with its tab, boss and collar, and the holes cut."""
    body = union_all([gearbox_block(), tab(), boss(), collar()])
    return cut_all(body, gearbox_cutters())


def screw_head(z: float) -> cq.Shape:
    """Return a pan head screw's head in its recess, with a cross recess."""
    top = GEARBOX_THICKNESS / 2 - SCREW_HEAD_SINK
    bottom = GEARBOX_THICKNESS / 2 - SCREW_RECESS_DEPTH - OVERLAP
    head = cq.Workplane('XY').add(along_y(SCREW_X, z, SCREW_HEAD_DIAMETER / 2, bottom, top))
    head = solid(head.faces('>Y').edges().fillet(SCREW_HEAD_ROUND))
    length, width, depth = SCREW_CROSS.length, SCREW_CROSS.width, SCREW_CROSS.depth
    cross = [box(SCREW_X - length / 2, top - depth, z - width / 2, SCREW_X + length / 2,
                 top + CUTTER_REACH, z + width / 2),
             box(SCREW_X - width / 2, top - depth, z - length / 2, SCREW_X + width / 2,
                 top + CUTTER_REACH, z + length / 2)]
    return cut_all(head, cross)


def make_screws() -> List[cq.Shape]:
    """Return the two screws' heads on the +y face."""
    return [screw_head(SCREW_Z), screw_head(-SCREW_Z)]


def make_shaft() -> cq.Shape:
    """Return the output shaft with its two flats, a chamfered end and a bore in the end."""
    face = GEARBOX_THICKNESS / 2
    end = face + SHAFT_LENGTH
    round_bar = along_y(SHAFT_X, 0.0, SHAFT_DIAMETER / 2, face - SHAFT_ROOT, end)
    flats = box(SHAFT_X - SHAFT_DIAMETER, face - SHAFT_ROOT - CUTTER_REACH, -SHAFT_FLATS / 2,
                SHAFT_X + SHAFT_DIAMETER, end + CUTTER_REACH, SHAFT_FLATS / 2)
    bar = cq.Workplane('XY').add(round_bar.intersect(flats))
    bar = solid(bar.faces('>Y').edges().chamfer(SHAFT_END_CHAMFER))
    return bar.cut(along_y(SHAFT_X, 0.0, SHAFT_BORE_DIAMETER / 2, end - SHAFT_BORE_DEPTH,
                           end + CUTTER_REACH))


# ---------------------------------------------------------------------------------------------
# Motor housing and the motor's can behind its vents
# ---------------------------------------------------------------------------------------------
def vent_cutters() -> List[cq.Shape]:
    """Return the vents' cutters in the housing's +y and -y faces."""
    face = HOUSING_THICKNESS / 2
    cutters = []
    for z in (SHAFT_SIDE_VENTS.z, -SHAFT_SIDE_VENTS.z):
        cutters.append(rounded_slot(SHAFT_SIDE_VENTS, z, face - VENT_DEPTH, face + CUTTER_REACH))
    for z in (BACK_SIDE_VENTS.z, -BACK_SIDE_VENTS.z):
        cutters.append(rounded_slot(BACK_SIDE_VENTS, z, -face - CUTTER_REACH,
                                    -face + VENT_DEPTH))
    return cutters


def housing_side(workplane: cq.Workplane, reach: float) -> cq.Workplane:
    """Draw the motor housing's side along x, reaching from the motor's axis out to reach.

    The workplane's first axis is x, its second the distance from the motor's axis. The line
    starts on the axis at the encoder's end, rises with the round there, runs along the housing
    and comes down the chamfer at the neck's end, back to the axis.
    """
    start, end = HOUSING_START_X, HOUSING_START_X + HOUSING_LENGTH
    radius = HOUSING_END_ROUND
    corner = radius * (1 - math.cos(math.pi / 4))
    return (workplane.moveTo(start, 0.0).lineTo(start, reach - radius)
            .threePointArc((start + corner, reach - corner), (start + radius, reach))
            .lineTo(end - HOUSING_CHAMFER, reach).lineTo(end, reach - HOUSING_CHAMFER)
            .lineTo(end, 0.0))


def housing_block() -> cq.Shape:
    """Return the motor housing without its vents: round, cut by two flats, and its chamfer.

    The round sides come from the side turned about the x axis, the flats from the side
    mirrored about y = 0 and extruded along z. Both carry the round at the encoder's end and
    the chamfer down to the neck, so their intersection has them on the round sides and on the
    flats.
    """
    radius = HOUSING_HEIGHT / 2
    rod = housing_side(cq.Workplane('XY'), radius).close().revolve(360.0, (0, 0, 0), (1, 0, 0))
    flats = housing_side(cq.Workplane('XY', origin=(0, 0, -radius - CUTTER_REACH)),
                         HOUSING_THICKNESS / 2).mirrorX().extrude(2 * (radius + CUTTER_REACH))
    return solid(rod).intersect(solid(flats))


def make_motor_housing() -> cq.Shape:
    """Return the motor housing with the neck and its fins, and the vents cut."""
    housing = union_all([housing_block(), neck(), fin(1), fin(-1)])
    return cut_all(housing, vent_cutters())


def make_motor_can() -> List[cq.Shape]:
    """Return the motor can's face at the bottom of each vent.

    Each plate is a little larger than its vent and reaches into the housing, so that only its
    face shows, MOTOR_CAN_PLATE above the vent's bottom.
    """
    bottom = HOUSING_THICKNESS / 2 - VENT_DEPTH
    plates = []
    for vents, side in ((SHAFT_SIDE_VENTS, 1), (BACK_SIDE_VENTS, -1)):
        larger = vents._replace(length=vents.length + 2 * MOTOR_CAN_MARGIN,
                                width=vents.width + 2 * MOTOR_CAN_MARGIN)
        y0, y1 = bottom - OVERLAP, bottom + MOTOR_CAN_PLATE
        for z in (vents.z, -vents.z):
            plate = rounded_slot(larger, z, y0, y1)
            plates.append(plate if side > 0 else plate.mirror('XZ'))
    return plates


# ---------------------------------------------------------------------------------------------
# Encoder cover, board and connector
# ---------------------------------------------------------------------------------------------
def quarter_arc_middle(centre: Tuple[float, float], radius: float,
                       direction: Tuple[int, int]) -> Tuple[float, float]:
    """Return the middle point of a quarter arc about centre, towards the given diagonal.

    direction holds the signs of the diagonal's two components, such as (1, -1).
    """
    reach = radius * math.cos(math.pi / 4)
    return (centre[0] + direction[0] * reach, centre[1] + direction[1] * reach)


def cover_outline(workplane: cq.Workplane) -> cq.Workplane:
    """Draw the encoder cover's D-shaped outline across the motor's axis, and close it.

    The workplane's first axis is the motor's y, its second the motor's z. The outline goes
    round from the box's -y end with its two rounds, along the box's +z face, up the S-curve of
    two equal arcs to the round part's +z face, round the round part's two corners on the +y
    side, and back along the -z side.
    """
    top, bottom = ENCODER_TOP_Y, ENCODER_TOP_Y - ENCODER_DEPTH
    half, box_half = ENCODER_HEIGHT / 2, CONNECTOR_BOX_HEIGHT / 2
    corner, edge = ENCODER_TOP_EDGE_ROUND, ENCODER_BOTTOM_EDGE_ROUND
    round_end = top - ENCODER_ROUND_LENGTH
    step_end = round_end - ENCODER_STEP_LENGTH
    step_middle = ((round_end + step_end) / 2, (half + box_half) / 2)
    outline = (workplane.moveTo(bottom, -(box_half - edge)).lineTo(bottom, box_half - edge)
               .threePointArc(quarter_arc_middle((bottom + edge, box_half - edge), edge, (-1, 1)),
                              (bottom + edge, box_half))
               .lineTo(step_end, box_half)
               .tangentArcPoint(step_middle, relative=False)
               .tangentArcPoint((round_end, half), relative=False)
               .lineTo(top - corner, half)
               .threePointArc(quarter_arc_middle((top - corner, half - corner), corner, (1, 1)),
                              (top, half - corner)))
    if half - corner > ROUNDOFF:
        outline = outline.lineTo(top, corner - half)
    outline = (outline
               .threePointArc(quarter_arc_middle((top - corner, corner - half), corner, (1, -1)),
                              (top - corner, -half))
               .lineTo(round_end, -half)
               .tangentArcPoint((step_middle[0], -step_middle[1]), relative=False)
               .tangentArcPoint((step_end, -box_half), relative=False)
               .lineTo(bottom + edge, -box_half)
               .threePointArc(quarter_arc_middle((bottom + edge, edge - box_half), edge, (-1, -1)),
                              (bottom, edge - box_half)))
    return outline.close()


def cover_end_block() -> cq.Shape:
    """Return the cover's D-shaped part at the motor's end, from the end into the motor housing.

    Its end face's edges are rounded.
    """
    block = cover_outline(cq.Workplane('YZ', origin=(ENCODER_END_X, 0, 0))) \
        .extrude(HOUSING_START_X + OVERLAP - ENCODER_END_X)
    return solid(block.faces('<X').edges().fillet(ENCODER_END_EDGE_ROUND))


def connector_box() -> cq.Shape:
    """Return the cover's part under the motor housing's -y face, which holds the connector."""
    block = cq.Workplane('XY').add(box(HOUSING_START_X - OVERLAP, ENCODER_TOP_Y - ENCODER_DEPTH,
                                       -CONNECTOR_BOX_HEIGHT / 2, CONNECTOR_BOX_END_X,
                                       -HOUSING_THICKNESS / 2 + OVERLAP,
                                       CONNECTOR_BOX_HEIGHT / 2))
    return solid(block.edges('|X').edges('<Y').fillet(ENCODER_BOTTOM_EDGE_ROUND))


def latch() -> cq.Shape:
    """Return the latch on the motor housing's -y face, next to the connector's box."""
    return box(LATCH_X.low, -HOUSING_THICKNESS / 2 - LATCH_THICKNESS, -LATCH_HALF_HEIGHT,
               LATCH_X.high, -HOUSING_THICKNESS / 2 + OVERLAP, LATCH_HALF_HEIGHT)


def connector_window() -> cq.Shape:
    """Return the cutter of the connector's window in the cover's -y face."""
    return box(WINDOW_X.low, ENCODER_TOP_Y - ENCODER_DEPTH - CUTTER_REACH, -WINDOW_HALF_HEIGHT,
               WINDOW_X.high, WINDOW_FLOOR_Y, WINDOW_HALF_HEIGHT)


def make_encoder_cover() -> cq.Shape:
    """Return the encoder cover: the end block, the connector's box and the latch, the window."""
    cover = union_all([cover_end_block(), connector_box(), latch()])
    return cut_all(cover, [connector_window()])


def make_board() -> cq.Shape:
    """Return the encoder board's part around the window: its face shows in the window."""
    return box(WINDOW_X.low - OVERLAP, WINDOW_FLOOR_Y - BOARD_THICKNESS,
               -WINDOW_HALF_HEIGHT - OVERLAP, WINDOW_X.high + OVERLAP, WINDOW_FLOOR_Y + OVERLAP,
               WINDOW_HALF_HEIGHT + OVERLAP)


def contact_zs() -> List[float]:
    """Return the z of the connector's contacts, centred on z = 0."""
    return [(k - (CONTACTS - 1) / 2) * CONTACT_PITCH for k in range(CONTACTS)]


def board_face_y() -> float:
    """Return the y of the board's face that shows in the window, where the connector stands."""
    return WINDOW_FLOOR_Y - BOARD_THICKNESS


def connector_opening_y() -> float:
    """Return the y of the connector's opening, its face towards -y."""
    return board_face_y() - CONNECTOR_HEIGHT


def connector_body() -> cq.Shape:
    """Return the connector's plastic body on the board, with its cavity open towards -y."""
    opening = connector_opening_y()
    half_length = (CONTACTS - 1) * CONTACT_PITCH / 2 + CONNECTOR_END_WALL
    xs = Span(CONNECTOR_CENTRE_X - CONNECTOR_DEPTH / 2, CONNECTOR_CENTRE_X + CONNECTOR_DEPTH / 2)
    body = box(xs.low, opening, -half_length, xs.high, board_face_y() + OVERLAP, half_length)
    cavity = box(xs.low + CONNECTOR_WALL, opening - CUTTER_REACH, -half_length + CONNECTOR_WALL,
                 xs.high - CONNECTOR_WALL, opening + CONNECTOR_CAVITY_DEPTH,
                 half_length - CONNECTOR_WALL)
    return cut_all(body, [cavity])


def contacts() -> List[cq.Shape]:
    """Return the connector's six square contacts in its cavity."""
    opening = connector_opening_y()
    half = CONTACT_SIZE / 2
    return [box(CONNECTOR_CENTRE_X - half, opening + CONTACT_SINK, z - half,
                CONNECTOR_CENTRE_X + half, opening + CONNECTOR_CAVITY_DEPTH + SINK, z + half)
            for z in contact_zs()]


def pin_ends() -> List[cq.Shape]:
    """Return the row of six pin ends on the board beside the connector."""
    face = board_face_y()
    half = PIN_END_SIZE / 2
    return [box(PIN_END_X - half, face - PIN_END_HEIGHT, z - half, PIN_END_X + half,
                face + OVERLAP, z + half) for z in contact_zs()]


def make_connector() -> Pieces:
    """Return the six-pin header on the board: its body, its contacts and the pin ends."""
    return {'white_plastic': [connector_body()], 'tin': contacts() + pin_ends()}


# ---------------------------------------------------------------------------------------------
# Mesh
# ---------------------------------------------------------------------------------------------
FRAME = ("x along the motor, from the encoder's end at x = -39.725 mm to the mounting tab's "
         "end at +39.975 mm. y along the output shaft, which stands out of the gearbox's +y "
         "face. z across the gearbox's 22.4 mm height. The gearbox's middle planes are y = 0 "
         "and z = 0, and the output shaft's axis is at x = 23.765 mm, z = 0.")


def sub_part(name: str, pieces: Pieces) -> SubPart:
    """Return a named sub-part with one surface per material, in the materials' order.

    Raise a ValueError if a material of the pieces is not one of the motor's materials.
    """
    require_known_materials(name, pieces, MATERIALS)
    return SubPart(name, {material: tessellate_faces(
        cq.Compound.makeCompound(pieces[material]), LINEAR_DEFLECTION, ANGULAR_DEFLECTION)
        for material in MATERIALS if material in pieces})


def build() -> List[Model]:
    """Return the motor's mesh with its sub-parts, for an exploded view."""
    check_parameters()
    sub_parts = [
        sub_part('gearbox', {'blue_plastic': [make_gearbox()]}),
        sub_part('screws', {'steel': make_screws()}),
        sub_part('shaft', {'steel': [make_shaft()]}),
        sub_part('motor_housing', {'blue_plastic': [make_motor_housing()]}),
        sub_part('motor_can', {'motor_can': make_motor_can()}),
        sub_part('encoder_cover', {'blue_plastic': [make_encoder_cover()]}),
        sub_part('encoder_board', {'solder_mask_green': [make_board()]}),
        sub_part('connector', make_connector()),
    ]
    return [Model('dg01d_e_gearmotor', TITLE, FRAME, MATERIALS, sub_parts)]
