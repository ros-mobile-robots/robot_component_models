# SPDX-FileCopyrightText: 2026 ROS Mobile Robots
# SPDX-License-Identifier: BSD-3-Clause
"""Turn the generators' CadQuery solids into STL, COLLADA and binary glTF files.

Every generator returns its meshes as Model objects: named sub-parts, each with one Surface per
material, in millimetres in the mesh's own frame. This module writes each Model three times, all
in metres (ROS's unit, REP 103), converted from millimetres when it writes the vertices:

- <name>.stl: binary STL, without colours,
- <name>.dae: COLLADA 1.4.1, Z up, one node per sub-part, coloured materials,
- <name>.glb: binary glTF 2.0, one node per sub-part with the same materials.

The glTF vertex data stay in the mesh's frame (Z up). A root node turns them into glTF's Y up.
The files contain no time stamps or paths, so a second run writes the same bytes.
"""
import json
import math
import struct
from typing import Callable
from typing import Dict
from typing import List
from typing import NamedTuple
from typing import Optional
from typing import Sequence
from typing import Tuple

import cadquery as cq
import numpy as np
from OCP.BRep import BRep_Tool
from OCP.BRepLib import BRepLib_ToolTriangulatedShape
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.TopAbs import TopAbs_REVERSED
from OCP.TopLoc import TopLoc_Location

Colour = Tuple[float, float, float, float]
# Splits a piece's triangle corners (m, 3, 3) into materials: an array of m material names
Classifier = Callable[[np.ndarray], np.ndarray]

# COLLADA requires these dates. They are fixed so that the files are reproducible.
COLLADA_DATE = '2026-11-08T00:00:00Z'
# The license of the meshes, as in LICENSE and package.xml. Written into every COLLADA and glTF
# file, and into a <file>.license file next to each STL and glTF file (binary formats).
COPYRIGHT_HOLDER = 'ROS Mobile Robots'
COPYRIGHT_YEAR = 2026
LICENSE_ID = 'BSD-3-Clause'
COPYRIGHT = f'Copyright (c) {COPYRIGHT_YEAR}, {COPYRIGHT_HOLDER}. {LICENSE_ID}, see LICENSE.'
SPDX_LINES = (f'SPDX-FileCopyrightText: {COPYRIGHT_YEAR} {COPYRIGHT_HOLDER}',
              f'SPDX-License-Identifier: {LICENSE_ID}')
MM_TO_M = 0.001
# glTF's root node rotation: -90 degrees about x turns the meshes' Z up into glTF's Y up
Z_UP_TO_Y_UP = [-math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)]


class Material(NamedTuple):
    """A coloured material: Phong values for COLLADA, metal or not for glTF."""

    diffuse: Colour
    ambient: Colour
    specular: Colour
    shininess: float
    metallic: bool


class Surface(NamedTuple):
    """Triangles of one material: vertices (n, 3) in mm, unit normals (n, 3), triangles (m, 3)."""

    vertices: np.ndarray
    normals: np.ndarray
    triangles: np.ndarray


class SubPart(NamedTuple):
    """A named piece of a model, such as a connector, with its surfaces by material name."""

    name: str
    surfaces: Dict[str, Surface]


class Model(NamedTuple):
    """One mesh file set: its name, a one-line title, its frame and its sub-parts."""

    name: str
    title: str
    frame: str
    materials: Dict[str, Material]
    sub_parts: List[SubPart]


def phong(diffuse: Sequence[float], specular: Sequence[float], shininess: float,
          metallic: bool = False, ambient_factor: float = 1.0) -> Material:
    """Return a material from its diffuse and specular colours (RGB or RGBA)."""
    rgba = tuple(diffuse) if len(diffuse) == 4 else (*diffuse, 1.0)
    spec = tuple(specular) if len(specular) == 4 else (*specular, 1.0)
    ambient = (rgba[0] * ambient_factor, rgba[1] * ambient_factor, rgba[2] * ambient_factor,
               rgba[3])
    return Material(rgba, ambient, spec, shininess, metallic)  # type: ignore[arg-type]


# ---------------------------------------------------------------------------------------------
# Tessellation: each family of generators keeps the method its meshes were checked with
# ---------------------------------------------------------------------------------------------
def tessellate_faces(shape: cq.Workplane, linear: float, angular: float) -> Surface:
    """Return a solid's triangles face by face, with the surface normals at the vertices.

    Neighbouring faces share their edge nodes up to about 1e-12 mm. Snapping to 10 nm makes
    them identical, so they stay identical in the float32 STL and the shells stay closed. The
    triangles that collapse to a line at the poles of spheres and cones are dropped.
    """
    solid = shape.val() if isinstance(shape, cq.Workplane) else shape
    BRepMesh_IncrementalMesh(solid.wrapped, linear, False, angular, True)
    vertex_blocks: List[np.ndarray] = []
    triangle_blocks: List[np.ndarray] = []
    normal_blocks: List[np.ndarray] = []
    offset = 0
    for face in solid.Faces():
        location = TopLoc_Location()
        poly = BRep_Tool.Triangulation_s(face.wrapped, location)
        if poly is None:
            continue
        BRepLib_ToolTriangulatedShape.ComputeNormals_s(face.wrapped, poly)
        transform = location.Transformation()
        nodes = range(1, poly.NbNodes() + 1)
        vertices = np.array([[p.X(), p.Y(), p.Z()]
                             for p in (poly.Node(i).Transformed(transform) for i in nodes)])
        normals = np.array([[d.X(), d.Y(), d.Z()]
                            for d in (poly.Normal(i).Transformed(transform) for i in nodes)])
        triangles = np.array([poly.Triangle(i).Get()
                              for i in range(1, poly.NbTriangles() + 1)]) - 1
        if face.wrapped.Orientation() == TopAbs_REVERSED:
            normals = -normals
            triangles = triangles[:, ::-1]
        vertex_blocks.append(vertices)
        normal_blocks.append(normals)
        triangle_blocks.append(triangles + offset)
        offset += len(vertices)
    vertices = np.round(np.vstack(vertex_blocks), 5)
    triangles = np.vstack(triangle_blocks)
    a, b, c = vertices[triangles[:, 0]], vertices[triangles[:, 1]], vertices[triangles[:, 2]]
    keep = ~((a == b).all(1) | (b == c).all(1) | (c == a).all(1))
    return Surface(vertices, np.vstack(normal_blocks), triangles[keep])


def tessellate_shape(shape: cq.Shape, linear: float, angular: float) -> Surface:
    """Return a shape's triangles with area-weighted normals, smooth inside each face."""
    points, faces = shape.tessellate(linear, angular)
    vertices = np.array([p.toTuple() for p in points], dtype=float)
    triangles = np.array(faces, dtype=np.int64).reshape(-1, 3)
    return Surface(vertices, vertex_normals(vertices, triangles), triangles)


def tessellate_absolute(workplane: cq.Workplane, linear: float, angular: float) -> np.ndarray:
    """Return the triangle corners (m, 3, 3) of a workplane's solids, absolute deflection.

    CadQuery's own call uses a relative deflection, too fine for small circles. The large
    tolerance of the second call keeps the first triangulation.
    """
    shape = cq.Compound.makeCompound([o for o in workplane.vals() if isinstance(o, cq.Shape)])
    BRepMesh_IncrementalMesh(shape.wrapped, linear, False, angular, True)
    points, faces = shape.tessellate(1e3, angular)
    vertices = np.array([p.toTuple() for p in points])
    return vertices[np.array(faces, dtype=np.int64)]


def smooth_surface(corners: np.ndarray, angle_degrees: float = 30.0) -> Surface:
    """Return triangle corners (m, 3, 3) as a surface, smooth across edges below the angle.

    Corners at exactly the same position share a vertex. A corner's normal is the area-weighted
    sum of the normals of the triangles around its vertex whose normals lie within the angle of
    its own triangle's normal. Corners of a vertex with the same normal share an output vertex.
    The triangles keep their order and corners. Only +, -, *, / and sqrt decide the normals, and
    every CPU rounds these alike, so this step doesn't depend on the CPU's vector instructions.
    The triangles themselves come from OCP's mesher.
    """
    count = len(corners)
    positions, inverse = np.unique(corners.reshape(-1, 3), axis=0, return_inverse=True)
    vertex_of_corner = inverse.reshape(-1)
    face_of_corner = np.repeat(np.arange(count), 3)
    area_normals = cross_rows(corners[:, 1] - corners[:, 0], corners[:, 2] - corners[:, 0])
    unit_normals = unit_rows(area_normals)
    limit = math.cos(math.radians(angle_degrees))
    order = np.argsort(vertex_of_corner, kind='stable')
    starts = np.flatnonzero(np.r_[True, np.diff(vertex_of_corner[order]) != 0])
    groups = np.split(order, starts[1:])
    own = np.concatenate([np.repeat(group, len(group)) for group in groups])
    other = np.concatenate([np.tile(group, len(group)) for group in groups])
    first, second = unit_normals[face_of_corner[own]], unit_normals[face_of_corner[other]]
    dots = (first[:, 0] * second[:, 0] + first[:, 1] * second[:, 1]
            + first[:, 2] * second[:, 2])
    within = dots >= limit
    sums = np.zeros((count * 3, 3))
    np.add.at(sums, own[within], area_normals[face_of_corner[other[within]]])
    normals = unit_rows(sums)
    index: Dict[Tuple[int, float, float, float], int] = {}
    corner_index = np.empty(count * 3, dtype=np.int64)
    for corner, (vertex, normal) in enumerate(zip(vertex_of_corner.tolist(), normals.tolist())):
        corner_index[corner] = index.setdefault((vertex, *normal), len(index))
    keys = list(index)
    return Surface(positions[[key[0] for key in keys]], np.array([key[1:] for key in keys]),
                   corner_index.reshape(-1, 3))


def cross_rows(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Return the cross products of the rows, component by component."""
    return np.stack([a[:, 1] * b[:, 2] - a[:, 2] * b[:, 1],
                     a[:, 2] * b[:, 0] - a[:, 0] * b[:, 2],
                     a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]], axis=1)


def unit_rows(rows: np.ndarray) -> np.ndarray:
    """Return the rows scaled to unit length, zero rows as they are."""
    lengths = np.sqrt(rows[:, 0] * rows[:, 0] + rows[:, 1] * rows[:, 1] + rows[:, 2] * rows[:, 2])
    return rows / np.where(lengths > 0, lengths, 1.0)[:, None]


def vertex_normals(vertices: np.ndarray, triangles: np.ndarray) -> np.ndarray:
    """Return the area-weighted unit normals of the vertices, zero for unused vertices."""
    corners = vertices[triangles]
    face_normals = np.cross(corners[:, 1] - corners[:, 0], corners[:, 2] - corners[:, 0])
    normals = np.zeros_like(vertices)
    for k in range(3):
        np.add.at(normals, triangles[:, k], face_normals)
    lengths = np.linalg.norm(normals, axis=1, keepdims=True)
    return normals / np.where(lengths > 0, lengths, 1.0)


def transform_surface(surface: Surface, rotation: np.ndarray,
                      offset: Tuple[float, float, float]) -> Surface:
    """Return the surface turned by a 3 x 3 rotation matrix and moved by the offset (mm)."""
    return Surface(surface.vertices @ rotation.T + np.array(offset), surface.normals @ rotation.T,
                   surface.triangles)


def merge_surfaces(surfaces: Sequence[Surface]) -> Surface:
    """Return several surfaces as one."""
    offsets = np.cumsum([0] + [len(s.vertices) for s in surfaces[:-1]])
    return Surface(np.vstack([s.vertices for s in surfaces]),
                   np.vstack([s.normals for s in surfaces]),
                   np.vstack([s.triangles + o for s, o in zip(surfaces, offsets)]))


def surfaces_by_material(pieces: Sequence[Tuple[str, Surface]],
                         order: Sequence[str]) -> Dict[str, Surface]:
    """Return one surface per material from (material, surface) pieces, in the given order."""
    groups: Dict[str, List[Surface]] = {}
    for material, surface in pieces:
        groups.setdefault(material, []).append(surface)
    return {name: merge_surfaces(groups[name]) for name in order if name in groups}


def classified_sub_part(name: str, pieces: Sequence[Tuple[str, cq.Workplane]],
                        order: Sequence[str], linear: float, angular: float,
                        classifiers: Optional[Dict[str, Classifier]] = None) -> SubPart:
    """Return a sub-part from (material, workplane) pieces, with smooth normals per material.

    The pieces are tessellated with an absolute deflection. A classifier splits a piece's
    triangles into several materials, for example a board's solder mask and its edges.
    """
    corners: Dict[str, List[np.ndarray]] = {}
    for material, workplane in pieces:
        triangles = tessellate_absolute(workplane, linear, angular)
        classify = (classifiers or {}).get(material)
        materials = classify(triangles) if classify else np.full(len(triangles), material,
                                                                 dtype=object)
        for known in order:
            selected = triangles[materials == known]
            if len(selected):
                corners.setdefault(known, []).append(selected)
    return SubPart(name, {material: smooth_surface(np.concatenate(corners[material]))
                          for material in order if material in corners})


def compact_surface(surface: Surface) -> Surface:
    """Return the surface without unused vertices, every normal of unit length.

    A vertex without a normal (only on collapsed triangles) gets its triangles' normal, or +z.
    """
    used = np.unique(surface.triangles)
    index = np.full(len(surface.vertices), -1, dtype=np.int64)
    index[used] = np.arange(len(used))
    vertices = surface.vertices[used]
    triangles = index[surface.triangles]
    normals = surface.normals[used].astype(float)
    lengths = np.linalg.norm(normals, axis=1)
    missing = lengths < 1e-9
    if missing.any():
        fallback = vertex_normals(vertices, triangles)[missing]
        fallback[np.linalg.norm(fallback, axis=1) < 1e-9] = (0.0, 0.0, 1.0)
        normals[missing] = fallback
        lengths = np.linalg.norm(normals, axis=1)
    return Surface(vertices, normals / lengths[:, None], triangles)


# ---------------------------------------------------------------------------------------------
# Helpers for the writers
# ---------------------------------------------------------------------------------------------
def surfaces_in_order(model: Model) -> List[Tuple[str, str, Surface]]:
    """Return (sub-part, material, surface) for every surface of the model, in file order."""
    return [(part.name, material, compact_surface(surface))
            for part in model.sub_parts for material, surface in part.surfaces.items()
            if len(surface.triangles)]


def used_materials(model: Model) -> List[str]:
    """Return the names of the materials the model uses, in the order of model.materials."""
    used = {material for part in model.sub_parts for material, surface in part.surfaces.items()
            if len(surface.triangles)}
    return [name for name in model.materials if name in used]


def triangle_corners(model: Model) -> np.ndarray:
    """Return all triangle corners (m, 3, 3) of the model in mm, in file order."""
    return np.concatenate([surface.vertices[surface.triangles]
                           for _, _, surface in surfaces_in_order(model)])


def numbers(values: np.ndarray, digits: int) -> str:
    """Return the values as text with the given significant digits, ten to a line."""
    flat = [f'{value:.{digits}g}' for value in np.asarray(values).ravel()]
    return '\n'.join(' '.join(flat[i:i + 10]) for i in range(0, len(flat), 10))


def colour_text(colour: Sequence[float]) -> str:
    """Return a colour as COLLADA text."""
    return ' '.join(f'{value:.4g}' for value in colour)


def xml_id(text: str) -> str:
    """Return the text as a COLLADA id: letters, digits, '_' and '-' only."""
    return ''.join(c if c.isalnum() or c in '_-' else '_' for c in text)


# ---------------------------------------------------------------------------------------------
# STL
# ---------------------------------------------------------------------------------------------
def write_stl(model: Model, path: str) -> int:
    """Write the model as one binary STL in metres and return its number of triangles."""
    corners = triangle_corners(model)
    normals = np.cross(corners[:, 1] - corners[:, 0], corners[:, 2] - corners[:, 0])
    lengths = np.linalg.norm(normals, axis=1, keepdims=True)
    normals = normals / np.where(lengths > 0, lengths, 1.0)
    records = np.zeros(len(corners), dtype=[('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])
    records['n'] = normals
    records['v'] = corners * MM_TO_M
    header = f'{model.name}, metres, {LICENSE_ID}, robot_component_models'
    with open(path, 'wb') as stl:
        stl.write(header.encode('ascii')[:80].ljust(80, b' '))
        stl.write(struct.pack('<I', len(records)))
        stl.write(records.tobytes())
    return len(records)


# ---------------------------------------------------------------------------------------------
# COLLADA
# ---------------------------------------------------------------------------------------------
def collada_effect(name: str, material: Material) -> str:
    """Return the COLLADA effect of a material."""
    return (f'  <effect id="{name}-effect">\n'
            f'   <profile_COMMON><technique sid="common"><phong>\n'
            f'    <ambient><color>{colour_text(material.ambient)}</color></ambient>\n'
            f'    <diffuse><color>{colour_text(material.diffuse)}</color></diffuse>\n'
            f'    <specular><color>{colour_text(material.specular)}</color></specular>\n'
            f'    <shininess><float>{material.shininess:g}</float></shininess>\n'
            f'   </phong></technique></profile_COMMON>\n'
            f'  </effect>\n')


def collada_source(geometry: str, kind: str, values: np.ndarray, digits: int) -> str:
    """Return a COLLADA source of 3D vectors."""
    return (f'    <source id="{geometry}-{kind}">\n'
            f'     <float_array id="{geometry}-{kind}-array" count="{values.size}">\n'
            f'{numbers(values, digits)}\n'
            f'     </float_array>\n'
            f'     <technique_common>\n'
            f'      <accessor source="#{geometry}-{kind}-array" count="{len(values)}"'
            f' stride="3">\n'
            f'       <param name="X" type="float"/><param name="Y" type="float"/>'
            f'<param name="Z" type="float"/>\n'
            f'      </accessor>\n'
            f'     </technique_common>\n'
            f'    </source>\n')


def collada_geometry(geometry: str, surfaces: Sequence[Tuple[str, Surface]]) -> str:
    """Return one COLLADA geometry with a triangle list per material, positions in metres."""
    merged = merge_surfaces([surface for _, surface in surfaces])
    text = (f'  <geometry id="{geometry}" name="{geometry}">\n   <mesh>\n'
            + collada_source(geometry, 'positions', merged.vertices * MM_TO_M, 7)
            + collada_source(geometry, 'normals', merged.normals, 5)
            + f'    <vertices id="{geometry}-vertices">\n'
            f'     <input semantic="POSITION" source="#{geometry}-positions"/>\n'
            f'     <input semantic="NORMAL" source="#{geometry}-normals"/>\n'
            f'    </vertices>\n')
    offset = 0
    for material, surface in surfaces:
        indices = surface.triangles + offset
        offset += len(surface.vertices)
        text += (f'    <triangles material="{material}" count="{len(indices)}">\n'
                 f'     <input semantic="VERTEX" source="#{geometry}-vertices" offset="0"/>\n'
                 f'     <p>\n{numbers(indices, 10)}\n     </p>\n'
                 f'    </triangles>\n')
    return text + '   </mesh>\n  </geometry>\n'


def write_dae(model: Model, path: str, generator: str) -> None:
    """Write the model as COLLADA 1.4.1 in metres, Z up, one node per sub-part."""
    by_part: Dict[str, List[Tuple[str, Surface]]] = {}
    for part, material, surface in surfaces_in_order(model):
        by_part.setdefault(part, []).append((material, surface))
    materials = used_materials(model)
    geometries, nodes = '', ''
    for part, surfaces in by_part.items():
        geometry = xml_id(f'{model.name}-{part}')
        geometries += collada_geometry(geometry, surfaces)
        binds = ''.join(f'       <instance_material symbol="{material}" '
                        f'target="#{material}-material"/>\n' for material, _ in surfaces)
        nodes += (f'   <node id="{geometry}-node" name="{xml_id(part)}">\n'
                  f'    <instance_geometry url="#{geometry}">\n'
                  f'     <bind_material><technique_common>\n{binds}'
                  f'     </technique_common></bind_material>\n'
                  f'    </instance_geometry>\n'
                  f'   </node>\n')
    text = (f'<?xml version="1.0" encoding="utf-8"?>\n'
            f'<!-- {model.title}. Display model generated by {generator}.\n'
            f'     {SPDX_LINES[0]}\n'
            f'     {SPDX_LINES[1]} -->\n'
            f'<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">\n'
            f' <asset>\n'
            f'  <contributor>\n'
            f'   <authoring_tool>robot_component_models, {generator} (CadQuery)</authoring_tool>\n'
            f'   <copyright>{COPYRIGHT}</copyright>\n'
            f'  </contributor>\n'
            f'  <created>{COLLADA_DATE}</created>\n'
            f'  <modified>{COLLADA_DATE}</modified>\n'
            f'  <unit name="meter" meter="1"/>\n'
            f'  <up_axis>Z_UP</up_axis>\n'
            f' </asset>\n'
            f' <library_effects>\n'
            + ''.join(collada_effect(name, model.materials[name]) for name in materials)
            + ' </library_effects>\n <library_materials>\n'
            + ''.join(f'  <material id="{name}-material" name="{name}">'
                      f'<instance_effect url="#{name}-effect"/></material>\n'
                      for name in materials)
            + ' </library_materials>\n <library_geometries>\n'
            + geometries
            + ' </library_geometries>\n <library_visual_scenes>\n'
            f'  <visual_scene id="scene" name="{model.name}">\n'
            + nodes
            + '  </visual_scene>\n </library_visual_scenes>\n'
            ' <scene><instance_visual_scene url="#scene"/></scene>\n'
            '</COLLADA>\n')
    with open(path, 'w', encoding='utf-8', newline='\n') as dae:
        dae.write(text)


# ---------------------------------------------------------------------------------------------
# Binary glTF
# ---------------------------------------------------------------------------------------------
class GlbBuffer:
    """The binary chunk of a glTF file with its buffer views and accessors."""

    def __init__(self) -> None:
        """Start an empty buffer."""
        self.data = bytearray()
        self.views: List[Dict[str, int]] = []
        self.accessors: List[Dict[str, object]] = []

    def add(self, array: np.ndarray, target: int, component: int, kind: str,
            bounds: bool = False) -> int:
        """Append an array as a buffer view with one accessor, return the accessor's index."""
        while len(self.data) % 4:
            self.data.append(0)
        raw = array.tobytes()
        self.views.append({'buffer': 0, 'byteOffset': len(self.data), 'byteLength': len(raw),
                           'target': target})
        self.data += raw
        accessor: Dict[str, object] = {'bufferView': len(self.views) - 1,
                                       'componentType': component, 'count': len(array),
                                       'type': kind}
        if bounds:
            accessor['min'] = [float(v) for v in array.min(axis=0)]
            accessor['max'] = [float(v) for v in array.max(axis=0)]
        self.accessors.append(accessor)
        return len(self.accessors) - 1


def gltf_material(name: str, material: Material) -> Dict[str, object]:
    """Return a glTF PBR material: the diffuse colour, metal or not, roughness from shininess."""
    roughness = round(math.sqrt(2.0 / (material.shininess + 2.0)), 3)
    return {'name': name,
            'pbrMetallicRoughness': {'baseColorFactor': [float(c) for c in material.diffuse],
                                     'metallicFactor': 1.0 if material.metallic else 0.0,
                                     'roughnessFactor': roughness}}


def write_glb(model: Model, path: str) -> None:
    """Write the model as binary glTF 2.0 in metres, one named node per sub-part."""
    materials = used_materials(model)
    buffer = GlbBuffer()
    meshes: List[Dict[str, object]] = []
    for part in model.sub_parts:
        primitives = []
        for material, surface in part.surfaces.items():
            if not len(surface.triangles):
                continue
            surface = compact_surface(surface)
            positions = (surface.vertices * MM_TO_M).astype('<f4')
            normals = surface.normals.astype('<f4')
            index_type, component = ('<u2', 5123) if len(positions) < 65536 else ('<u4', 5125)
            indices = surface.triangles.astype(index_type).ravel()
            primitives.append({
                'attributes': {'POSITION': buffer.add(positions, 34962, 5126, 'VEC3', True),
                               'NORMAL': buffer.add(normals, 34962, 5126, 'VEC3')},
                'indices': buffer.add(indices, 34963, component, 'SCALAR'),
                'material': materials.index(material), 'mode': 4})
        if primitives:
            meshes.append({'name': part.name, 'primitives': primitives})
    while len(buffer.data) % 4:
        buffer.data.append(0)
    children = list(range(1, len(meshes) + 1))
    document = {
        'asset': {'version': '2.0', 'generator': 'robot_component_models generators',
                  'copyright': COPYRIGHT},
        'scene': 0,
        'scenes': [{'name': model.name, 'nodes': [0]}],
        'nodes': [{'name': model.name, 'rotation': Z_UP_TO_Y_UP, 'children': children}]
        + [{'name': mesh['name'], 'mesh': k} for k, mesh in enumerate(meshes)],
        'meshes': meshes,
        'materials': [gltf_material(name, model.materials[name]) for name in materials],
        'accessors': buffer.accessors,
        'bufferViews': buffer.views,
        'buffers': [{'byteLength': len(buffer.data)}],
    }
    text = json.dumps(document, separators=(',', ':')).encode('utf-8')
    text += b' ' * (-len(text) % 4)
    length = 12 + 8 + len(text) + 8 + len(buffer.data)
    with open(path, 'wb') as glb:
        glb.write(struct.pack('<4sII', b'glTF', 2, length))
        glb.write(struct.pack('<I4s', len(text), b'JSON'))
        glb.write(text)
        glb.write(struct.pack('<I4s', len(buffer.data), b'BIN\x00'))
        glb.write(bytes(buffer.data))


# ---------------------------------------------------------------------------------------------
# License files next to the binary formats
# ---------------------------------------------------------------------------------------------
def write_license_file(path: str) -> str:
    """Write <path>.license with the SPDX copyright and license lines, return its path."""
    license_path = path + '.license'
    with open(license_path, 'w', encoding='utf-8', newline='\n') as stream:
        stream.write('\n'.join(SPDX_LINES) + '\n')
    return license_path
