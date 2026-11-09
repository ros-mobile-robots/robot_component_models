#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 ROS Mobile Robots
# SPDX-License-Identifier: BSD-3-Clause
"""Render the README's pictures from the COLLADA meshes: one per part and all parts together.

The pictures show the meshes as written, with their materials, on a light background:
docs/images/<part>.png (800 x 600) and docs/images/all_parts.png (1600 x 800), with 256
colours to keep the files small. VTK renders
off screen, so no display is needed. The pictures aren't checked in CI: other graphics drivers
give slightly different pixels.

Usage: python3 scripts/render_parts.py [--out DIR]
"""
import argparse
import math
import os
import sys
from typing import Dict
from typing import List
from typing import NamedTuple
from typing import Sequence
from typing import Tuple

import collada
import numpy as np
from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont
import vtk
from vtk.util import numpy_support

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MESHES = os.path.join(ROOT, 'meshes')
BACKGROUND = ((0.98, 0.98, 0.97), (0.86, 0.87, 0.88))  # top and bottom of the gradient


class Placement(NamedTuple):
    """A mesh file and where it goes: offset (m) and turn about z (degrees)."""

    path: str
    offset: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    yaw: float = 0.0


class PartView(NamedTuple):
    """A part's meshes, its caption and the camera's azimuth and elevation (degrees)."""

    caption: str
    placements: List[Placement]
    azimuth: float
    elevation: float


# The caster's meshes as DiffBot's macro places them: the fork 1 mm behind the base's frame,
# the wheel on the fork's axle (16 mm further back, 20 mm below the fork's top).
PARTS: Dict[str, PartView] = {
    'swivel_caster_30mm': PartView('Swivel caster', [
        Placement('swivel_caster_30mm/swivel_caster_30mm_base.dae'),
        Placement('swivel_caster_30mm/swivel_caster_30mm_wheel_support.dae', (-0.001, 0.0, 0.0)),
        Placement('swivel_caster_30mm/swivel_caster_30mm_wheel.dae', (-0.017, 0.0, -0.020))],
        -60.0, 15.0),
    'rplidar_a2': PartView('Laser scanner (RPLIDAR A2 size)',
                           [Placement('rplidar_a2/rplidar_a2.dae')], 25.0, 22.0),
    'rplidar_a1': PartView('Laser scanner (RPLIDAR A1 size)',
                           [Placement('rplidar_a1/rplidar_a1.dae')], 215.0, 24.0),
    'raspberry_pi_4b': PartView('Raspberry Pi 4 Model B',
                                [Placement('raspberry_pi_4b/raspberry_pi_4b.dae')], 50.0, 38.0),
    'jetson_nano_dev_kit_b01': PartView(
        'Jetson Nano Developer Kit B01',
        [Placement('jetson_nano_dev_kit_b01/jetson_nano_dev_kit_b01.dae')], 60.0, 32.0),
    'teensy_4_0': PartView('Teensy 4.0 with headers', [Placement('teensy_4_0/teensy_4_0.dae')],
                           -140.0, 32.0),
    'breadboard_400_point': PartView(
        '400-point breadboard', [Placement('breadboard_400_point/breadboard_400_point.dae')],
        -60.0, 45.0),
    'raspberry_pi_camera_v2': PartView(
        'Camera Module v2', [Placement('raspberry_pi_camera_v2/raspberry_pi_camera_v2.dae')],
        30.0, 18.0),
}
# All parts together, real size, as seen from the overview's camera: back row, then front row,
# each part turned so that its typical side faces the camera.
OVERVIEW_AZIMUTH, OVERVIEW_ELEVATION = -90.0, 32.0
OVERVIEW_ROWS = [
    [('jetson_nano_dev_kit_b01', 180.0), ('raspberry_pi_4b', 180.0),
     ('breadboard_400_point', 90.0), ('rplidar_a1', 55.0)],
    [('raspberry_pi_camera_v2', -90.0), ('teensy_4_0', 0.0), ('swivel_caster_30mm', -30.0),
     ('rplidar_a2', -90.0)],
]


class Surface(NamedTuple):
    """Triangles of one material: corners and normals (k, 3, 3), Phong values."""

    corners: np.ndarray
    normals: np.ndarray
    diffuse: Tuple[float, ...]
    specular: float
    shininess: float


def read_dae(path: str) -> List[Surface]:
    """Return the surfaces of a COLLADA file with their materials, read by pycollada."""
    document = collada.Collada(path)
    surfaces = []
    for geometry in document.scene.objects('geometry'):
        for primitive in geometry.primitives():
            effect = primitive.material.effect
            specular = effect.specular if isinstance(effect.specular, tuple) else (0.0,) * 3
            surfaces.append(Surface(np.asarray(primitive.vertex[primitive.vertex_index]),
                                    np.asarray(primitive.normal[primitive.normal_index]),
                                    tuple(effect.diffuse), float(np.mean(specular[:3])),
                                    float(effect.shininess or 1.0)))
    return surfaces


def turn_z(points: np.ndarray, degrees: float) -> np.ndarray:
    """Return points (..., 3) turned about z."""
    c, s = math.cos(math.radians(degrees)), math.sin(math.radians(degrees))
    return points @ np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]).T


def placed_surfaces(view: PartView, yaw: float = 0.0) -> List[Surface]:
    """Return a part's surfaces, placed, then turned about z by yaw."""
    surfaces = []
    for placement in view.placements:
        for surface in read_dae(os.path.join(MESHES, placement.path)):
            corners = turn_z(surface.corners, placement.yaw) + np.array(placement.offset)
            normals = turn_z(surface.normals, placement.yaw)
            surfaces.append(surface._replace(corners=turn_z(corners, yaw),
                                             normals=turn_z(normals, yaw)))
    return surfaces


def actor(surface: Surface, offset: Sequence[float]) -> vtk.vtkActor:
    """Return a VTK actor for one surface, moved by the offset (m)."""
    points = surface.corners.reshape(-1, 3) + np.asarray(offset)
    vtk_points = vtk.vtkPoints()
    vtk_points.SetData(numpy_support.numpy_to_vtk(points.astype(np.float64), deep=True))
    cells = vtk.vtkCellArray()
    cells.SetData(numpy_support.numpy_to_vtkIdTypeArray(
        np.arange(0, len(points) + 1, 3, dtype=np.int64), deep=True),
        numpy_support.numpy_to_vtkIdTypeArray(np.arange(len(points), dtype=np.int64), deep=True))
    poly = vtk.vtkPolyData()
    poly.SetPoints(vtk_points)
    poly.SetPolys(cells)
    normals = numpy_support.numpy_to_vtk(surface.normals.reshape(-1, 3).astype(np.float32),
                                         deep=True)
    normals.SetName('Normals')
    poly.GetPointData().SetNormals(normals)
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputData(poly)
    result = vtk.vtkActor()
    result.SetMapper(mapper)
    prop = result.GetProperty()
    prop.SetColor(*surface.diffuse[:3])
    prop.SetAmbient(0.3)
    prop.SetDiffuse(0.85)
    prop.SetSpecular(surface.specular)
    prop.SetSpecularPower(max(surface.shininess, 1.0))
    prop.SetInterpolationToPhong()
    return result


def bounds(surfaces: Sequence[Surface]) -> Tuple[np.ndarray, np.ndarray]:
    """Return the lowest and highest corner of the surfaces."""
    points = np.concatenate([surface.corners.reshape(-1, 3) for surface in surfaces])
    return points.min(axis=0), points.max(axis=0)


def direction(azimuth: float, elevation: float) -> np.ndarray:
    """Return the unit vector from the target to the camera."""
    a, e = math.radians(azimuth), math.radians(elevation)
    return np.array([math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e)])


def render(actors: Sequence[vtk.vtkActor], low: np.ndarray, high: np.ndarray, azimuth: float,
           elevation: float, size: Tuple[int, int],
           labels: Sequence[Tuple[str, np.ndarray]] = ()) -> Image.Image:
    """Render the actors with three lights, cropped to the parts and fitted into the size.

    The scene is rendered at twice the picture's size and scaled down, which smooths the edges.
    """
    renderer = vtk.vtkRenderer()
    renderer.SetBackground(0.0, 0.0, 0.0)
    renderer.SetBackgroundAlpha(0.0)
    for item in actors:
        renderer.AddActor(item)
    centre = (low + high) / 2
    radius = float(np.linalg.norm(high - low)) / 2
    renderer.RemoveAllLights()
    for light_direction, intensity in ((direction(azimuth + 40, 55), 0.9),
                                       (direction(azimuth - 70, 20), 0.45),
                                       (direction(azimuth + 180, 60), 0.35)):
        light = vtk.vtkLight()
        light.SetLightTypeToSceneLight()
        light.SetPosition(*(centre + light_direction * radius * 20))
        light.SetFocalPoint(*centre)
        light.SetIntensity(intensity)
        renderer.AddLight(light)
    window = vtk.vtkRenderWindow()
    window.SetOffScreenRendering(1)
    window.SetAlphaBitPlanes(1)
    window.AddRenderer(renderer)
    window.SetSize(size[0] * 2, size[1] * 2)
    camera = renderer.GetActiveCamera()
    camera.SetFocalPoint(*centre)
    camera.SetPosition(*(centre + direction(azimuth, elevation) * radius * 10))
    camera.SetViewUp(0, 0, 1)
    camera.SetViewAngle(12)
    renderer.ResetCamera(low[0], high[0], low[1], high[1], low[2], high[2])
    renderer.ResetCameraClippingRange()
    window.Render()
    grab = vtk.vtkWindowToImageFilter()
    grab.SetInput(window)
    grab.SetInputBufferTypeToRGBA()
    grab.Update()
    image = grab.GetOutput()
    width, height, _ = image.GetDimensions()
    pixels = numpy_support.vtk_to_numpy(image.GetPointData().GetScalars()).reshape(height, width,
                                                                                   4)
    rendered = Image.fromarray(np.flipud(pixels).copy(), 'RGBA')
    points = [display_point(renderer, point, height) for _, point in labels]
    return compose(rendered, size, [(text, point) for (text, _), point in zip(labels, points)])


def display_point(renderer: vtk.vtkRenderer, point: np.ndarray,
                  height: int) -> Tuple[float, float]:
    """Return the pixel (x, y from the top) of a point (m) in the rendered picture."""
    coordinate = vtk.vtkCoordinate()
    coordinate.SetCoordinateSystemToWorld()
    coordinate.SetValue(*point)
    x, y = coordinate.GetComputedDoubleDisplayValue(renderer)
    return x, height - y


def background(size: Tuple[int, int]) -> Image.Image:
    """Return the light vertical gradient behind the parts."""
    top, bottom = np.array(BACKGROUND[0]) * 255, np.array(BACKGROUND[1]) * 255
    rows = np.linspace(0.0, 1.0, size[1])[:, None] * (bottom - top) + top
    return Image.fromarray(np.repeat(rows[:, None, :], size[0], axis=1).astype(np.uint8), 'RGB')


def compose(rendered: Image.Image, size: Tuple[int, int],
            labels: Sequence[Tuple[str, Tuple[float, float]]]) -> Image.Image:
    """Crop the rendered parts, fit them with a margin onto the background, add the labels."""
    font = label_font()
    label_height = 40 if labels else 0
    left, top, right, bottom = rendered.getchannel('A').getbbox() or (0, 0, *rendered.size)
    margin = 0.06 * min(size)
    scale = min((size[0] - 2 * margin) / (right - left),
                (size[1] - 2 * margin - label_height) / (bottom - top))
    cropped = rendered.crop((left, top, right, bottom))
    fitted = cropped.resize((round(cropped.width * scale), round(cropped.height * scale)),
                            Image.LANCZOS)
    x0 = round((size[0] - fitted.width) / 2)
    y0 = round((size[1] - label_height - fitted.height) / 2)
    picture = background(size)
    picture.paste(fitted, (x0, y0), fitted)
    draw = ImageDraw.Draw(picture)
    for text, (x, y) in labels:
        px, py = x0 + (x - left) * scale, y0 + (y - top) * scale
        width = draw.textlength(text, font=font)
        left_edge = min(max(px - width / 2, 8), size[0] - width - 8)
        draw.text((left_edge, py + 6), text, fill=(40, 40, 44), font=font)
    return picture


def label_font() -> ImageFont.ImageFont:
    """Return the labels' font: DejaVu Sans if installed, else Pillow's default."""
    try:
        return ImageFont.truetype('DejaVuSans.ttf', 20)
    except OSError:
        return ImageFont.load_default()


def render_part(name: str, view: PartView) -> Image.Image:
    """Render one part."""
    surfaces = placed_surfaces(view)
    low, high = bounds(surfaces)
    return render([actor(surface, (0, 0, 0)) for surface in surfaces], low, high, view.azimuth,
                  view.elevation, (800, 600))


def render_overview() -> Image.Image:
    """Render all parts together at their real size, in two rows facing the camera."""
    right = direction(OVERVIEW_AZIMUTH + 90, 0)
    towards = direction(OVERVIEW_AZIMUTH, 0)
    actors: List[vtk.vtkActor] = []
    labels: List[Tuple[str, np.ndarray]] = []
    gap = 0.035
    for row, depth in zip(OVERVIEW_ROWS, (-0.09, 0.09)):
        placed = [(name, placed_surfaces(PARTS[name], yaw + OVERVIEW_AZIMUTH + 90))
                  for name, yaw in row]
        widths = [float(np.dot(np.ptp(np.concatenate(
            [s.corners.reshape(-1, 3) for s in surfaces]), axis=0), np.abs(right)))
            for _, surfaces in placed]
        position = -(sum(widths) + gap * (len(widths) - 1)) / 2
        for (name, surfaces), width in zip(placed, widths):
            low, high = bounds(surfaces)
            centre = (low + high) / 2
            target = right * (position + width / 2) + towards * depth
            offset = np.array([target[0] - centre[0], target[1] - centre[1], -low[2]])
            actors += [actor(surface, offset) for surface in surfaces]
            front = towards * (np.dot(high - centre, np.abs(towards)) + 0.004)
            labels.append((PARTS[name].caption, np.array([target[0], target[1], 0.0]) + front))
            position += width + gap
    points = np.array([actor_.GetBounds() for actor_ in actors])
    low = np.array([points[:, 0].min(), points[:, 2].min(), points[:, 4].min()])
    high = np.array([points[:, 1].max(), points[:, 3].max(), points[:, 5].max()])
    return render(actors, low, high, OVERVIEW_AZIMUTH, OVERVIEW_ELEVATION, (1600, 800), labels)


def small_png(picture: Image.Image) -> Image.Image:
    """Return the picture with a palette of 256 colours, dithered."""
    return picture.quantize(colors=256, method=Image.Quantize.MEDIANCUT,
                            dither=Image.Dither.FLOYDSTEINBERG)


def main() -> int:
    """Render every part and the overview."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--out', default=os.path.join(ROOT, 'docs', 'images'))
    args = parser.parse_args()
    os.makedirs(args.out, exist_ok=True)
    for name, view in PARTS.items():
        path = os.path.join(args.out, f'{name}.png')
        small_png(render_part(name, view)).save(path, optimize=True)
        print(path)
    path = os.path.join(args.out, 'all_parts.png')
    small_png(render_overview()).save(path, optimize=True)
    print(path)
    return 0


if __name__ == '__main__':
    sys.exit(main())
