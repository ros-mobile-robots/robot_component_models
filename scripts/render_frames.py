#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 ROS Mobile Robots
# SPDX-License-Identifier: BSD-3-Clause
"""Render the frame pictures of the provenance pages: where each mesh's origin and axes are.

docs/images/frames/<part>.png shows a part's COLLADA meshes in two views without perspective.
Each view has the origin of the mesh's frame as a red dot, the two positive axes that lie in the
view as red arrows, a green ring around each mounting hole and a dashed line at each mounting
plane. A robot description needs these to place the mesh: the provenance page's Frame section
gives the same points as numbers.

The mounting holes come from the generators' own parameters, so a picture can't show another
place than the mesh has. VTK renders off screen, so no display is needed. The pictures aren't
checked in CI: other graphics drivers give slightly different pixels.

Usage, from the repository's root: python3 -m scripts.render_frames [--out DIR]
"""
import argparse
import math
import os
import sys
from typing import Callable
from typing import Dict
from typing import List
from typing import NamedTuple
from typing import Sequence
from typing import Tuple

import numpy as np
from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont
import vtk
from vtk.util import numpy_support

from generators import breadboard_400_point
from generators import jetson_nano_dev_kit_b01
from generators import PARTS as GENERATED_PARTS
from generators import raspberry_pi_4b
from generators import raspberry_pi_camera_v2
from generators import rplidar_a2
from generators import swivel_caster_30mm
from generators import teensy_4_0
from generators import tt_motor_wheel_65mm
from scripts.render_parts import actor
from scripts.render_parts import background
from scripts.render_parts import PARTS as RENDERED_PARTS
from scripts.render_parts import PartView
from scripts.render_parts import placed_surfaces
from scripts.render_parts import small_png
from scripts.render_parts import Surface
from scripts.render_parts import turn_x
from scripts.render_parts import turn_z

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
M_PER_MM = 0.001
# A point or a direction in a mesh's frame: x, y, z. Points are in millimetres
Vector = Tuple[float, float, float]
X_AXIS, Y_AXIS, Z_AXIS = (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)
AXIS_NAMES = {X_AXIS: '+x', Y_AXIS: '+y', Z_AXIS: '+z'}
# The picture: two views side by side, each in a panel with its title above
PANEL_SIZE = (590, 430)   # pixels
PANEL_MARGIN = 46         # pixels between a panel's edge and the part
TITLE_HEIGHT = 34         # pixels above each panel
GAP = 20                  # pixels between and around the panels
SUPERSAMPLING = 2         # the views are rendered larger by this factor and scaled down
ARROW_LENGTH = 62         # pixels
SCALE_BAR_LIMIT = 150     # pixels: the scale bar is the longest round length below this
SCALE_BAR_LENGTHS = (0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0)  # mm
RING_RADIUS = 9           # pixels
AXIS_COLOUR = (195, 55, 39)
HOLE_COLOUR = (0, 136, 111)
PLANE_COLOUR = (36, 84, 160)
TEXT_COLOUR = (32, 38, 42)
HALO_COLOUR = (255, 255, 255)


class View(NamedTuple):
    """A view without perspective: its title and the frame's directions to the right and up.

    The viewer looks against the cross product of right and up: with +x to the right and +y
    up, from +z down onto the part.
    """

    title: str
    right: Vector
    up: Vector


class Plane(NamedTuple):
    """A plane to mark with a dashed line: its label, its normal axis and its place (mm)."""

    label: str
    normal: Vector
    position: float


class Label(NamedTuple):
    """A text at a point (mm) of the part, moved by an offset in pixels (right, down)."""

    text: str
    point: Vector
    offset: Tuple[int, int]


class FrameFigure(NamedTuple):
    """What a part's frame picture shows, in the frame of the part's first mesh."""

    views: Tuple[View, View]
    holes: Sequence[Vector]
    planes: Sequence[Plane]
    labels: Sequence[Label] = ()


TOP_VIEW = View('From above, +z towards the viewer', X_AXIS, Y_AXIS)
SIDE_VIEW = View('From the side, +y away from the viewer', X_AXIS, Z_AXIS)
PORT_VIEW = View('From the port side, +y towards the viewer', (-1.0, 0.0, 0.0), Z_AXIS)


# ---------------------------------------------------------------------------------------------
# Each part's mounting holes and planes, from its generator's parameters
# ---------------------------------------------------------------------------------------------
def caster_figure() -> FrameFigure:
    """Return the caster's picture: the base's frame, the three meshes placed as on DiffBot."""
    caster = swivel_caster_30mm
    top = caster.STANDOFF_TOP_Z
    holes = [(caster.SWIVEL_AXIS_X + x_sign * caster.PLATE_HOLES[0],
              y_sign * caster.PLATE_HOLES[1], top) for x_sign in (-1, 1) for y_sign in (-1, 1)]
    axle_x = caster.SWIVEL_AXIS_X + caster.AXLE_X
    # Where each label's line ends, on a place of its own mesh that the side view shows: the
    # base at half the height of the front standoff on the viewer's side, the fork on an arm
    # between the cup and the axle, the wheel on its white side below the axle, clear of the
    # arm and of the axle's rivet
    on_base = (caster.SWIVEL_AXIS_X + caster.PLATE_HOLES[0], -caster.PLATE_HOLES[1],
               top - caster.STANDOFF_LENGTH / 2)
    on_fork = (-9.0, 0.0, -7.0)
    below_axle = 0.55 * caster.WHEEL_RADIUS
    on_wheel = (axle_x - below_axle, 0.0, caster.AXLE_Z - below_axle)
    return FrameFigure(
        (TOP_VIEW, SIDE_VIEW), holes,
        [Plane(f"standoffs' tops, z = +{top:g} mm", Z_AXIS, top)],
        [Label('swivel_caster_30mm_base', on_base, (10, -6)),
         Label('swivel_caster_30mm_wheel_support', on_fork, (22, 4)),
         Label('swivel_caster_30mm_wheel', on_wheel, (26, 18))])


def scanner_figure() -> FrameFigure:
    """Return the laser scanner's picture: the four M3 holes in its bottom."""
    scanner = rplidar_a2
    bottom = scanner.BOTTOM_Z
    holes = [(scanner.MOUNT_HOLE_RADIUS * math.cos(math.radians(45 + 90 * k)),
              scanner.MOUNT_HOLE_RADIUS * math.sin(math.radians(45 + 90 * k)), bottom)
             for k in range(4)]
    return FrameFigure(
        (View('From below, +z away from the viewer', X_AXIS, (0.0, -1.0, 0.0)), SIDE_VIEW),
        holes,
        [Plane(f'bottom, z = {bottom:g} mm', Z_AXIS, bottom)])


def raspberry_pi_figure() -> FrameFigure:
    """Return the Raspberry Pi's picture: the four holes, the board's faces."""
    board = raspberry_pi_4b
    underside = -board.BOARD_THICKNESS / 2
    holes = [(board.BOARD_TO_MESH[0] - y, x + board.BOARD_TO_MESH[1], underside)
             for x, y in board.HOLES]
    return FrameFigure(
        (TOP_VIEW, PORT_VIEW), holes,
        [Plane(f"board's underside, z = {underside:g} mm", Z_AXIS, underside)])


def jetson_figure() -> FrameFigure:
    """Return the Jetson kit's picture: the four holes, the carrier's bottom face."""
    kit = jetson_nano_dev_kit_b01
    holes = [(kit.BOARD_TO_MESH[0] - u, kit.BOARD_TO_MESH[1] - v, 0.0) for u, v in kit.HOLES]
    # From the side, the picture shows the room under the module and the heatsink's fins
    return FrameFigure(
        (TOP_VIEW, View('From the side, +x towards the viewer', Y_AXIS, Z_AXIS)), holes,
        [Plane("carrier's bottom face, z = 0", Z_AXIS, 0.0)])


def teensy_figure() -> FrameFigure:
    """Return the Teensy's picture: no holes, the planes of the board and the header."""
    spacer_bottom = -teensy_4_0.SPACER_H
    return FrameFigure(
        (TOP_VIEW, SIDE_VIEW), [],
        [Plane("board's bottom face, z = 0", Z_AXIS, 0.0),
         Plane(f"header spacer's bottom, z = {spacer_bottom:g} mm", Z_AXIS, spacer_bottom)])


def breadboard_figure() -> FrameFigure:
    """Return the breadboard's picture: no holes, the bottom of its foam backing."""
    top = breadboard_400_point.TOP_Z
    return FrameFigure(
        (TOP_VIEW, SIDE_VIEW), [],
        [Plane("foam backing's bottom, z = 0", Z_AXIS, 0.0),
         Plane(f'top, z = {top:g} mm', Z_AXIS, top)])


def camera_figure() -> FrameFigure:
    """Return the camera's picture: the four holes, the board's back face, the lens' axis."""
    camera = raspberry_pi_camera_v2
    back = camera.X_BACK
    holes = [(back, *camera.to_yz(a, b)) for a, b in camera.HOLES_AB]
    lens_y, lens_z = camera.to_yz(*camera.LENS_AB)
    return FrameFigure(
        (View('From the front, +x towards the viewer', Y_AXIS, Z_AXIS),
         View('From the side, +y towards the viewer', (-1.0, 0.0, 0.0), Z_AXIS)),
        holes,
        [Plane(f"board's back face, x = {back:g} mm", X_AXIS, back)],
        [Label('lens, looking along +x', (5.0, lens_y, lens_z), (-150, -34))])


def wheel_figure() -> FrameFigure:
    """Return the wheel's picture: no holes, the axle along z, the outer face, the tube's end."""
    wheel = tt_motor_wheel_65mm
    outer, tube_end = wheel.TYRE_OUTER_FACE_Z, wheel.HUB_TUBE_END_Z
    # the label's line ends on the hub tube's end, at the top of the tube's chamfer
    on_tube_end = (0.0, wheel.HUB_TUBE_D / 2 - wheel.HUB_TUBE_CHAMFER, tube_end)
    return FrameFigure(
        (View('From the outer face, +z away from the viewer', (-1.0, 0.0, 0.0), Y_AXIS),
         View('From the side, +x away from the viewer', Z_AXIS, Y_AXIS)),
        [],
        [Plane(f'outer face, z = {outer:g} mm', Z_AXIS, outer)],
        [Label(f"tube's end, z = +{tube_end:g} mm", on_tube_end, (14, -60))])


FIGURES: Dict[str, Callable[[], FrameFigure]] = {
    'swivel_caster_30mm': caster_figure,
    'rplidar_a2': scanner_figure,
    'raspberry_pi_4b': raspberry_pi_figure,
    'jetson_nano_dev_kit_b01': jetson_figure,
    'teensy_4_0': teensy_figure,
    'breadboard_400_point': breadboard_figure,
    'raspberry_pi_camera_v2': camera_figure,
    'tt_motor_wheel_65mm': wheel_figure,
}


# ---------------------------------------------------------------------------------------------
# Rendering one view and drawing on it
# ---------------------------------------------------------------------------------------------
class Projection(NamedTuple):
    """How a view maps the frame's points to the pixels of its panel."""

    right: np.ndarray
    up: np.ndarray
    centre: np.ndarray      # the point (m) at the panel's centre
    pixels_per_metre: float

    def pixel(self, point: Vector) -> Tuple[float, float]:
        """Return where a point, given in millimetres, lies in the panel: x and y in pixels.

        y counts from the panel's top.
        """
        offset = np.array(point) * M_PER_MM - self.centre
        return (PANEL_SIZE[0] / 2 + float(offset @ self.right) * self.pixels_per_metre,
                PANEL_SIZE[1] / 2 - float(offset @ self.up) * self.pixels_per_metre)


def extent(surfaces: Sequence[Surface], direction: np.ndarray) -> Tuple[float, float]:
    """Return the lowest and highest place (m) of the surfaces along a direction."""
    places = np.concatenate([surface.corners.reshape(-1, 3) for surface in surfaces]) @ direction
    return float(places.min()), float(places.max())


def projection(surfaces: Sequence[Surface], view: View, pixels_per_metre: float) -> Projection:
    """Return the view's projection with the part in the panel's centre."""
    right, up = np.array(view.right), np.array(view.up)
    towards_viewer = np.cross(right, up)
    centre = sum(0.5 * sum(extent(surfaces, direction)) * direction
                 for direction in (right, up, towards_viewer))
    return Projection(right, up, np.asarray(centre), pixels_per_metre)


def fitting_scale(surfaces: Sequence[Surface], view: View) -> float:
    """Return the largest scale (pixels per metre) at which the part fits the view's panel."""
    scales = []
    for direction, pixels in zip((view.right, view.up), PANEL_SIZE):
        low, high = extent(surfaces, np.array(direction))
        scales.append((pixels - 2 * PANEL_MARGIN) / (high - low))
    return min(scales)


def render_view(surfaces: Sequence[Surface], shown: Projection) -> Image.Image:
    """Return the part rendered without perspective, on a transparent background."""
    towards_viewer = np.cross(shown.right, shown.up)
    renderer = vtk.vtkRenderer()
    renderer.SetBackground(0.0, 0.0, 0.0)
    renderer.SetBackgroundAlpha(0.0)
    for surface in surfaces:
        renderer.AddActor(actor(surface, (0.0, 0.0, 0.0)))
    renderer.RemoveAllLights()
    for light_direction, intensity in ((towards_viewer + 0.5 * shown.right + 0.9 * shown.up, 0.6),
                                       (towards_viewer - 0.8 * shown.right + 0.3 * shown.up, 0.3),
                                       (towards_viewer - 0.6 * shown.up, 0.15)):
        light = vtk.vtkLight()
        light.SetLightTypeToSceneLight()
        light.SetPosition(*(shown.centre + light_direction))
        light.SetFocalPoint(*shown.centre)
        light.SetIntensity(intensity)
        renderer.AddLight(light)
    window = vtk.vtkRenderWindow()
    window.SetOffScreenRendering(1)
    window.SetAlphaBitPlanes(1)
    window.AddRenderer(renderer)
    width, height = PANEL_SIZE[0] * SUPERSAMPLING, PANEL_SIZE[1] * SUPERSAMPLING
    window.SetSize(width, height)
    camera = renderer.GetActiveCamera()
    camera.ParallelProjectionOn()
    camera.SetFocalPoint(*shown.centre)
    camera.SetPosition(*(shown.centre + towards_viewer))
    camera.SetViewUp(*shown.up)
    # The parallel scale is half the panel's height in the scene's unit, the metre
    camera.SetParallelScale(PANEL_SIZE[1] / 2 / shown.pixels_per_metre)
    renderer.ResetCameraClippingRange()
    window.Render()
    grab = vtk.vtkWindowToImageFilter()
    grab.SetInput(window)
    grab.SetInputBufferTypeToRGBA()
    grab.Update()
    pixels = numpy_support.vtk_to_numpy(grab.GetOutput().GetPointData().GetScalars())
    rendered = Image.fromarray(np.flipud(pixels.reshape(height, width, 4)).copy(), 'RGBA')
    return rendered.resize(PANEL_SIZE, Image.LANCZOS)


def font(size: int) -> ImageFont.FreeTypeFont:
    """Return Pillow's built-in font, the same on every system."""
    return ImageFont.load_default(size)


def text_with_halo(draw: ImageDraw.ImageDraw, place: Tuple[float, float], text: str,
                   colour: Tuple[int, int, int], size: int = 17) -> None:
    """Draw a text with a white outline, so that it is readable on the part."""
    draw.text(place, text, fill=colour, font=font(size), stroke_width=3, stroke_fill=HALO_COLOUR)


def draw_axes(draw: ImageDraw.ImageDraw, shown: Projection) -> None:
    """Draw the origin as a dot and the positive axes that lie in the view as arrows."""
    origin = shown.pixel((0.0, 0.0, 0.0))
    for axis, name in AXIS_NAMES.items():
        across, upwards = float(np.array(axis) @ shown.right), float(np.array(axis) @ shown.up)
        if abs(across) + abs(upwards) < 0.5:
            continue
        tip = (origin[0] + across * ARROW_LENGTH, origin[1] - upwards * ARROW_LENGTH)
        head = [tip] + [(tip[0] - 11 * across + side * 5 * upwards,
                         tip[1] + 11 * upwards + side * 5 * across) for side in (-1, 1)]
        draw.line([origin, tip], fill=HALO_COLOUR, width=7)
        draw.line([origin, tip], fill=AXIS_COLOUR, width=3)
        draw.polygon(head, fill=AXIS_COLOUR, outline=HALO_COLOUR)
        text_with_halo(draw, (tip[0] + 17 * across - 11, tip[1] - 17 * upwards - 12), name,
                       AXIS_COLOUR, 19)
    draw.ellipse((origin[0] - 5, origin[1] - 5, origin[0] + 5, origin[1] + 5),
                 fill=AXIS_COLOUR, outline=HALO_COLOUR)


def draw_holes(draw: ImageDraw.ImageDraw, shown: Projection, holes: Sequence[Vector]) -> None:
    """Draw a ring around each mounting hole, in a view that looks along the holes."""
    towards_viewer = np.cross(shown.right, shown.up)
    spread = np.ptp(np.array(holes) @ towards_viewer) if holes else 1.0
    if spread > 1e-6:
        return
    for hole in holes:
        x, y = shown.pixel(hole)
        for width, colour in ((6, HALO_COLOUR), (3, HOLE_COLOUR)):
            draw.ellipse((x - RING_RADIUS, y - RING_RADIUS, x + RING_RADIUS, y + RING_RADIUS),
                         outline=colour, width=width)


def draw_planes(draw: ImageDraw.ImageDraw, shown: Projection, planes: Sequence[Plane]) -> None:
    """Draw a dashed line with its label at each plane that the view sees edge-on."""
    for plane in planes:
        normal = np.array(plane.normal)
        on_plane = tuple(float(v) for v in normal * plane.position)
        x, y = shown.pixel((on_plane[0], on_plane[1], on_plane[2]))
        if abs(float(normal @ shown.up)) > 0.5:
            for start in range(8, PANEL_SIZE[0] - 8, 14):
                draw.line([(start, y), (start + 8, y)], fill=PLANE_COLOUR, width=2)
            text_with_halo(draw, (10, y + 5), plane.label, PLANE_COLOUR)
        elif abs(float(normal @ shown.right)) > 0.5:
            for start in range(8, PANEL_SIZE[1] - 8, 14):
                draw.line([(x, start), (x, start + 8)], fill=PLANE_COLOUR, width=2)
            text_with_halo(draw, (x + 8, 10), plane.label, PLANE_COLOUR)


def draw_scale_bar(draw: ImageDraw.ImageDraw, shown: Projection) -> None:
    """Draw a bar of a round length in the panel's lower right corner, with its length.

    The two views of a part have their own scales, so each panel says how long 1 mm is.
    """
    longest = SCALE_BAR_LIMIT / (shown.pixels_per_metre * M_PER_MM)
    length = max(candidate for candidate in SCALE_BAR_LENGTHS if candidate <= longest)
    pixels = length * M_PER_MM * shown.pixels_per_metre
    right, y = PANEL_SIZE[0] - 14, PANEL_SIZE[1] - 16
    draw.line([(right - pixels, y), (right, y)], fill=TEXT_COLOUR, width=3)
    for x in (right - pixels, right):
        draw.line([(x, y - 5), (x, y + 5)], fill=TEXT_COLOUR, width=2)
    text = f'{length:g} mm'
    text_with_halo(draw, (right - pixels / 2 - draw.textlength(text, font=font(15)) / 2, y - 24),
                   text, TEXT_COLOUR, 15)


def draw_labels(draw: ImageDraw.ImageDraw, shown: Projection, labels: Sequence[Label]) -> None:
    """Draw each label next to its point, with a thin line from the point to the text."""
    for label in labels:
        x, y = shown.pixel(label.point)
        width = draw.textlength(label.text, font=font(16))
        text_x = min(max(x + label.offset[0], 8), PANEL_SIZE[0] - width - 8)
        text_y = y + label.offset[1]
        draw.line([(x, y), (text_x, text_y + 10)], fill=TEXT_COLOUR, width=1)
        draw.ellipse((x - 3, y - 3, x + 3, y + 3), fill=TEXT_COLOUR, outline=HALO_COLOUR)
        text_with_halo(draw, (text_x, text_y), label.text, TEXT_COLOUR, 16)


def surfaces_in_frame(view: PartView) -> List[Surface]:
    """Return a part's surfaces in the frame of its first mesh.

    The placements put a part's other meshes into its first mesh's frame (the caster's fork and
    wheel). A placement can also turn the first mesh for the README's picture (the wheel stands
    on its tread there). That turn is taken back here, so that the axes drawn are the mesh's own.
    """
    first = view.placements[0]
    surfaces = []
    for surface in placed_surfaces(view):
        corners = turn_x(turn_z(surface.corners - np.array(first.offset), -first.yaw), -first.roll)
        normals = turn_x(turn_z(surface.normals, -first.yaw), -first.roll)
        surfaces.append(surface._replace(corners=corners, normals=normals))
    return surfaces


def render_figure(name: str) -> Image.Image:
    """Return a part's frame picture: its two views with the frame drawn on them."""
    figure = FIGURES[name]()
    surfaces = surfaces_in_frame(RENDERED_PARTS[name])
    size = (2 * PANEL_SIZE[0] + 3 * GAP, PANEL_SIZE[1] + TITLE_HEIGHT + 2 * GAP)
    picture = Image.new('RGB', size, (255, 255, 255))
    titles = ImageDraw.Draw(picture)
    for column, view in enumerate(figure.views):
        shown = projection(surfaces, view, fitting_scale(surfaces, view))
        panel = background(PANEL_SIZE)
        rendered = render_view(surfaces, shown)
        panel.paste(rendered, (0, 0), rendered)
        draw = ImageDraw.Draw(panel)
        draw_planes(draw, shown, figure.planes)
        draw_holes(draw, shown, figure.holes)
        draw_labels(draw, shown, figure.labels if column == 1 else ())
        draw_axes(draw, shown)
        draw_scale_bar(draw, shown)
        left = GAP + column * (PANEL_SIZE[0] + GAP)
        picture.paste(panel, (left, GAP + TITLE_HEIGHT))
        titles.text((left, GAP), view.title, fill=TEXT_COLOUR, font=font(20))
    return picture


def main() -> int:
    """Render every part's frame picture."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--out', default=os.path.join(ROOT, 'docs', 'images', 'frames'))
    args = parser.parse_args()
    os.makedirs(args.out, exist_ok=True)
    for name in GENERATED_PARTS:
        path = os.path.join(args.out, f'{name}.png')
        small_png(render_figure(name)).save(path, optimize=True)
        print(path)
    return 0


if __name__ == '__main__':
    sys.exit(main())
