#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 ROS Mobile Robots
# SPDX-License-Identifier: BSD-3-Clause
"""Check the mesh files in meshes/ against meshes/manifest.yaml.

The checks read the files with other libraries than the generators (trimesh, pycollada, lxml),
so they don't share the generators' mistakes:

- meshes/ holds exactly the manifest's files, each with the manifest's checksum and size,
- every STL is closed and consistently oriented, without collapsed triangles,
- triangle counts and bounding boxes match the manifest, in each file's units,
- every file is in metres, and the COLLADA and glTF files have the STL file's triangles,
  corner for corner and with the same winding,
- every COLLADA file is valid against the COLLADA 1.4.1 schema and loads in pycollada,
- every COLLADA file gives each corner of its triangle lists its own normal, by an index of
  the normal's own that equals the position's index,
- every glTF file loads in trimesh, with the manifest's sub-parts as named nodes,
- every file carries the manifest's license: SPDX lines in the COLLADA header, in a
  <file>.license next to each STL and glTF file.

Usage: python3 scripts/check_meshes.py [--schema-dir DIR]
The COLLADA schema is downloaded from Khronos into DIR (default .cache/collada) and verified by
its checksum. Exit code 0 if every check passes.
"""
import argparse
import hashlib
import json
import os
import struct
import sys
from typing import Any
from typing import Callable
from typing import Dict
from typing import List
from typing import NamedTuple
from typing import Optional
from typing import Tuple
import urllib.request

import collada
from lxml import etree
import numpy as np
from scipy.spatial import cKDTree
import trimesh
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MESHES = os.path.join(ROOT, 'meshes')
COLLADA_NAMESPACE = 'http://www.collada.org/2005/11/COLLADASchema'
SCHEMA_URL = 'https://www.khronos.org/files/collada_schema_1_4_1.xsd'
SCHEMA_SHA256 = '49266165027329388e710ade6a424116570cda6776cbf3623f7b8c78c727b0ff'
# The COLLADA schema imports the XML namespace schema for xml:base. This stand-in declares the
# attributes, so the check doesn't download it from w3.org.
XML_NAMESPACE_SCHEMA = b"""<?xml version="1.0"?>
<xs:schema targetNamespace="http://www.w3.org/XML/1998/namespace"
           xmlns:xs="http://www.w3.org/2001/XMLSchema">
  <xs:attribute name="base" type="xs:anyURI"/>
  <xs:attribute name="lang" type="xs:language"/>
  <xs:attribute name="space" type="xs:NCName"/>
</xs:schema>
"""
# glTF's root node turns the Z-up vertex data into Y up: (x, y, z) Z up is (x, z, -y) Y up
Y_UP_TO_Z_UP = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]])
COPYRIGHT_LINE = 'SPDX-FileCopyrightText: 2026 ROS Mobile Robots'
# Bounding boxes in the manifest are rounded to 0.001 mm or 0.000001 m. The float32 vertices
# of the glTF files and the 7 digits of the COLLADA files add less than that.
BOX_TOLERANCE = {'metre': 0.0000011}
# The three files of a mesh are written from the same triangles. For a coordinate below 1 m,
# the seven significant digits of a COLLADA file round by at most 0.00000005 m and the float32
# numbers of an STL or glTF file by at most 0.00000003 m. A corner then lies within
# 0.00000014 m of its vertex in the STL file. A corner of a COLLADA or glTF file is the
# nearest STL vertex within this distance, 0.2 micrometres
VERTEX_TOLERANCE = 0.0000002


class MeshFile(NamedTuple):
    """A file of the manifest with its mesh's entry and its own entry."""

    path: str
    mesh: Dict[str, Any]
    entry: Dict[str, Any]
    license_id: str


class Geometry(NamedTuple):
    """A file's triangle corners (m, 3, 3) in its own units and frame (Z up)."""

    corners: np.ndarray


Check = Callable[[MeshFile], List[str]]


def load_manifest() -> Dict[str, Any]:
    """Return the manifest as read by PyYAML."""
    with open(os.path.join(MESHES, 'manifest.yaml'), encoding='utf-8') as manifest:
        return yaml.safe_load(manifest)


def manifest_files(manifest: Dict[str, Any]) -> List[MeshFile]:
    """Return every file of the manifest."""
    files = []
    for part in manifest['parts'].values():
        for mesh in part['meshes'].values():
            for path, entry in mesh['files'].items():
                files.append(MeshFile(path, mesh, entry, manifest['license']))
    return files


def file_path(mesh_file: MeshFile) -> str:
    """Return the file's path on disk."""
    return os.path.join(MESHES, mesh_file.path)


# ---------------------------------------------------------------------------------------------
# Reading the files
# ---------------------------------------------------------------------------------------------
def stl_corners(path: str) -> np.ndarray:
    """Return the triangle corners of a binary STL, read by trimesh without any processing."""
    mesh = trimesh.load(path, file_type='stl', process=False)
    return np.asarray(mesh.triangles, dtype=float)


def dae_corners(path: str) -> np.ndarray:
    """Return the triangle corners of a COLLADA file, read by pycollada, nodes applied."""
    document = collada.Collada(path)
    blocks = [np.asarray(primitive.vertex[primitive.vertex_index])
              for geometry in document.scene.objects('geometry')
              for primitive in geometry.primitives()]
    return np.concatenate(blocks).astype(float)


def glb_corners(path: str) -> np.ndarray:
    """Return the triangle corners of a glTF file, read by trimesh, turned back to Z up."""
    scene = trimesh.load(path, file_type='glb', process=False)
    corners = np.concatenate([np.asarray(mesh.triangles) for mesh in scene.dump()])
    return corners @ Y_UP_TO_Z_UP.T


READERS = {'.stl': stl_corners, '.dae': dae_corners, '.glb': glb_corners}
GEOMETRY: Dict[str, Geometry] = {}


def geometry(mesh_file: MeshFile) -> Geometry:
    """Return the file's geometry, read once."""
    if mesh_file.path not in GEOMETRY:
        reader = READERS[os.path.splitext(mesh_file.path)[1]]
        GEOMETRY[mesh_file.path] = Geometry(reader(file_path(mesh_file)))
    return GEOMETRY[mesh_file.path]


# ---------------------------------------------------------------------------------------------
# Checks of one file
# ---------------------------------------------------------------------------------------------
def check_checksum(mesh_file: MeshFile) -> List[str]:
    """Check the file's SHA-256 checksum and size against the manifest."""
    with open(file_path(mesh_file), 'rb') as stream:
        data = stream.read()
    problems = []
    if hashlib.sha256(data).hexdigest() != mesh_file.entry['sha256']:
        problems.append('checksum differs from the manifest')
    if len(data) != mesh_file.entry['bytes']:
        problems.append(f'{len(data)} bytes, manifest {mesh_file.entry["bytes"]}')
    return problems


def check_triangle_count(mesh_file: MeshFile) -> List[str]:
    """Check the number of triangles against the manifest."""
    count = len(geometry(mesh_file).corners)
    expected = mesh_file.entry['triangles']
    return [] if count == expected else [f'{count} triangles, manifest {expected}']


def check_bounding_box(mesh_file: MeshFile) -> List[str]:
    """Check the bounding box, in the file's own units, against the manifest."""
    points = geometry(mesh_file).corners.reshape(-1, 3)
    tolerance = BOX_TOLERANCE[str(mesh_file.entry['units'])]
    problems = []
    for name, values in (('min', points.min(axis=0)), ('max', points.max(axis=0))):
        expected = np.array(mesh_file.entry[f'bounding_box_{name}'], dtype=float)
        if np.abs(values - expected).max() > tolerance:
            problems.append(f'bounding box {name} {np.round(values, 6).tolist()}, '
                            f'manifest {expected.tolist()}')
    return problems


def check_units(mesh_file: MeshFile) -> List[str]:
    """Check that an STL file is in metres: a robot component is between 5 mm and 0.5 m long.

    The COLLADA and glTF files are compared with the STL file, see check_same_triangles().
    """
    if not mesh_file.path.endswith('.stl'):
        return []
    size = np.ptp(geometry(mesh_file).corners.reshape(-1, 3), axis=0).max()
    return [] if 0.005 < size < 0.5 else [f'largest size {size:.6f}: not metres?']


def turned_to_smallest_first(triangles: np.ndarray) -> np.ndarray:
    """Return triangles (m, 3) of vertex numbers, each turned so that its smallest comes first.

    Turning keeps a triangle's winding: (a, b, c), (b, c, a) and (c, a, b) are the same face.
    (a, c, b) is that face seen from the other side.
    """
    first = triangles.argmin(axis=1)
    rows = np.arange(len(triangles))
    return np.stack([triangles[rows, (first + k) % 3] for k in range(3)], axis=1)


def triangle_counts(triangles: np.ndarray) -> Dict[Tuple[int, int, int], int]:
    """Return how often each triangle (a, b, c) of vertex numbers occurs."""
    unique, counts = np.unique(triangles, axis=0, return_counts=True)
    return {(int(a), int(b), int(c)): int(count) for (a, b, c), count in zip(unique, counts)}


def triangle_differences(reference: np.ndarray, corners: np.ndarray,
                         tolerance: float) -> List[str]:
    """Return how the triangles (m, 3, 3) of a file differ from the reference file's.

    The two files may list the triangles in another order and may start each triangle at
    another corner. They must have the same triangles with the same winding, because a viewer
    that draws only the front of a face hides a triangle with the opposite winding.

    1. Number the reference's vertices, and give each corner of the file the number of the
       nearest reference vertex within the tolerance.
    2. Turn each triangle so that its smallest number comes first.
    3. Compare how often each triangle occurs in the two files. A triangle of the file whose
       corners are a reference triangle in the opposite order has the opposite winding.
    """
    vertices, numbers = np.unique(reference.reshape(-1, 3), axis=0, return_inverse=True)
    distances, nearest = cKDTree(vertices).query(corners.reshape(-1, 3),
                                                 distance_upper_bound=tolerance)
    away = int(np.isinf(distances).sum())
    if away:
        return [f'{away} corners lie more than {tolerance:g} from every vertex of the STL file']
    expected = triangle_counts(turned_to_smallest_first(numbers.reshape(-1, 3)))
    found = triangle_counts(turned_to_smallest_first(nearest.reshape(-1, 3)))
    missing = {triangle: count - found.get(triangle, 0) for triangle, count in expected.items()
               if count > found.get(triangle, 0)}
    extra = {triangle: count - expected.get(triangle, 0) for triangle, count in found.items()
             if count > expected.get(triangle, 0)}
    opposite = sum(min(count, missing.get((a, c, b), 0)) for (a, b, c), count in extra.items())
    problems = []
    if opposite:
        problems.append(f"{opposite} triangles have the opposite winding of the STL file's")
    if sum(missing.values()) > opposite:
        problems.append(f'{sum(missing.values()) - opposite} triangles of the STL file are '
                        'missing')
    if sum(extra.values()) > opposite:
        problems.append(f'{sum(extra.values()) - opposite} triangles are not in the STL file')
    return problems


def check_same_triangles(mesh_file: MeshFile) -> List[str]:
    """Check that a COLLADA or glTF file has the STL file's triangles, with the same winding.

    The readers apply each file's scene: the COLLADA nodes, and the glTF root node's turn from
    Z up to Y up, which glb_corners() turns back. So the corners are compared in the mesh's
    frame, in metres.
    """
    extension = os.path.splitext(mesh_file.path)[1]
    if extension == '.stl':
        return []
    stl_path = mesh_file.path[:-len(extension)] + '.stl'
    stl = mesh_file._replace(path=stl_path, entry=mesh_file.mesh['files'][stl_path])
    return triangle_differences(geometry(stl).corners, geometry(mesh_file).corners,
                                VERTEX_TOLERANCE)


def check_closed(mesh_file: MeshFile) -> List[str]:
    """Check that an STL is closed and consistently oriented, without collapsed triangles.

    The files are assemblies of closed shells, which may touch. So every directed edge must
    have as many reverse edges as it has copies, with the corners matched exactly.
    """
    if not mesh_file.path.endswith('.stl'):
        return []
    corners = geometry(mesh_file).corners.astype(np.float32)
    _, index = np.unique(corners.reshape(-1, 3), axis=0, return_inverse=True)
    triangles = index.reshape(-1, 3)
    problems = []
    collapsed = int(((triangles[:, 0] == triangles[:, 1]) | (triangles[:, 1] == triangles[:, 2])
                     | (triangles[:, 2] == triangles[:, 0])).sum())
    if collapsed:
        problems.append(f'{collapsed} collapsed triangles')
    edges = np.concatenate([triangles[:, [0, 1]], triangles[:, [1, 2]], triangles[:, [2, 0]]])
    vertex_count = int(index.max()) + 1
    forward, forward_count = np.unique(edges[:, 0] * vertex_count + edges[:, 1],
                                       return_counts=True)
    backward, backward_count = np.unique(edges[:, 1] * vertex_count + edges[:, 0],
                                         return_counts=True)
    unmatched = len(forward) != len(backward) or bool(
        (forward != backward).any() or (forward_count != backward_count).any())
    if unmatched:
        problems.append('edges without a matching reverse edge: not closed or not oriented')
    return problems


def check_collada(mesh_file: MeshFile, schema: etree.XMLSchema) -> List[str]:
    """Check a COLLADA file against the schema, its unit, up axis and materials."""
    document = etree.parse(file_path(mesh_file))
    problems = []
    if not schema.validate(document):
        problems.append(f'not valid COLLADA 1.4.1: {schema.error_log.last_error}')
    namespace = {'c': COLLADA_NAMESPACE}
    if document.xpath('string(//c:asset/c:unit/@meter)', namespaces=namespace) != '1':
        problems.append('unit is not the metre')
    if document.xpath('string(//c:asset/c:up_axis)', namespaces=namespace) != 'Z_UP':
        problems.append('up axis is not Z')
    materials = document.xpath('//c:library_materials/c:material/@name', namespaces=namespace)
    if list(materials) != list(mesh_file.mesh['materials']):
        problems.append(f'materials {list(materials)}, manifest {mesh_file.mesh["materials"]}')
    nodes = document.xpath('//c:visual_scene/c:node/@name', namespaces=namespace)
    if list(nodes) != list(mesh_file.mesh['sub_parts']):
        problems.append(f'nodes {list(nodes)}, manifest {mesh_file.mesh["sub_parts"]}')
    return problems


def corner_index_problems(triangles: Any, vectors: int) -> List[str]:
    """Return the problems of one <triangles> element's indices, for a mesh of that many vectors.

    The writer gives a geometry one normal for each position. So every corner is a pair of
    the same number: the index of its position, then the index of its normal. A pair of two
    different numbers would give the corner the normal of another vertex.
    """
    name = f'<triangles material="{triangles.get("material")}">'
    corners = 3 * int(triangles.get('count', 0))
    indices = np.array((triangles.findtext(f'{{{COLLADA_NAMESPACE}}}p') or '').split(),
                       dtype=np.int64)
    if indices.size != 2 * corners:
        return [f'{name} has {indices.size} indices, not a position and a normal index for '
                f'each of its {corners} corners']
    positions, normals = indices[0::2], indices[1::2]
    problems = []
    beyond = int(((indices < 0) | (indices >= vectors)).sum())
    if beyond:
        problems.append(f'{name} has {beyond} indices outside the {vectors} positions and '
                        'normals of its mesh')
    different = int((positions != normals).sum())
    if different:
        problems.append(f'{name} has {different} corners whose normal index differs from '
                        'their position index')
    return problems


def input_list(element: Any) -> List[str]:
    """Return the inputs of a <vertices> or <triangles> element as the file binds them.

    Each input reads as its semantic, the source it takes its vectors from and, in a triangle
    list, its offset in a corner's indices: 'NORMAL from #box-normals at offset 1'.
    """
    inputs = []
    for bound in element.findall(f'{{{COLLADA_NAMESPACE}}}input'):
        offset = bound.get('offset')
        inputs.append(f'{bound.get("semantic")} from {bound.get("source")}'
                      + ('' if offset is None else f' at offset {offset}'))
    return inputs


def source_vector_count(mesh: Any, source_id: str) -> Tuple[Optional[int], List[str]]:
    """Return how many vectors the mesh's <source> with this id holds, and its problems.

    The source's accessor has to read its vectors from the source's own array, three numbers
    each, from the array's first number on. An accessor that named another source's array
    would read that source's numbers, and one with an offset would read past the array's end.
    The count is None if the mesh has no such source or the source has a problem.
    """
    namespace = {'c': COLLADA_NAMESPACE}
    sources = mesh.xpath(f'c:source[@id="{source_id}"]', namespaces=namespace)
    if len(sources) != 1:
        return None, [f'the mesh has {len(sources)} sources with the id {source_id}, not one']
    arrays = sources[0].findall('c:float_array', namespace)
    accessors = sources[0].findall('c:technique_common/c:accessor', namespace)
    if len(arrays) != 1 or len(accessors) != 1:
        return None, [f'the source {source_id} has {len(arrays)} arrays and {len(accessors)} '
                      'accessors, not one of each']
    array, accessor = arrays[0], accessors[0]
    vectors = int(accessor.get('count', 0))
    if accessor.get('source') != f'#{array.get("id")}' or array.get('id') is None:
        return None, [f"the source {source_id}'s accessor reads {accessor.get('source')}, not "
                      f"the source's own array #{array.get('id')}"]
    if accessor.get('stride') != '3' or int(array.get('count', 0)) != 3 * vectors:
        return None, [f"the source {source_id}'s accessor reads {vectors} vectors with a "
                      f"stride of {accessor.get('stride')} from an array of "
                      f"{array.get('count')} numbers, not three numbers for each vector"]
    if int(accessor.get('offset', 0)) != 0:
        return None, [f"the source {source_id}'s accessor starts at the array's number "
                      f"{accessor.get('offset')}, not at its first number"]
    return vectors, []


def collada_normal_problems(document: Any) -> List[str]:
    """Return the problems of a parsed COLLADA file's normals.

    Gazebo Classic's COLLADA loader merges the vertices that share a position. It keeps the
    corners of a sharp edge apart only when the normals are an input of <triangles> with an
    offset of their own. With the normals as an input of <vertices>, it shades all the faces
    around a sharp edge with the normal of one of them.

    An input names the source that it takes its vectors from. A file can declare a NORMAL
    input at the right offset and bind it to the positions' source: a viewer then reads the
    positions as normals. So the check follows every reference. A geometry with the id G must
    have, in its mesh:

    1. the sources G-positions and G-normals, each read from its own array, with as many
       normals as positions,
    2. <vertices id="G-vertices"> with one input: POSITION from #G-positions,
    3. in every <triangles>, VERTEX from #G-vertices at offset 0 and NORMAL from #G-normals
       at offset 1, and no other input,
    4. for every corner, its position's index and the same number as its normal's index,
       both within the vectors of the two sources.
    """
    namespace = {'c': COLLADA_NAMESPACE}
    problems = []
    for geometry in document.xpath('//c:geometry', namespaces=namespace):
        name = geometry.get('id')
        for mesh in geometry.findall('c:mesh', namespace):
            positions, position_problems = source_vector_count(mesh, f'{name}-positions')
            normals, normal_problems = source_vector_count(mesh, f'{name}-normals')
            problems += position_problems + normal_problems
            if positions is not None and normals is not None and positions != normals:
                problems.append(f'the mesh has {positions} positions and {normals} normals, '
                                'not one normal for each position')
            expected = [f'POSITION from #{name}-positions']
            for vertices in mesh.findall('c:vertices', namespace):
                if vertices.get('id') != f'{name}-vertices' or input_list(vertices) != expected:
                    problems.append(f'<vertices id="{vertices.get("id")}"> has the inputs '
                                    f'{input_list(vertices)}, not {expected} with the id '
                                    f'{name}-vertices')
            expected = [f'VERTEX from #{name}-vertices at offset 0',
                        f'NORMAL from #{name}-normals at offset 1']
            for triangles in mesh.findall('c:triangles', namespace):
                if input_list(triangles) != expected:
                    problems.append(f'<triangles material="{triangles.get("material")}"> has '
                                    f'the inputs {input_list(triangles)}, not {expected}')
                elif positions is not None and positions == normals:
                    problems += corner_index_problems(triangles, positions)
    return problems


def check_collada_normals(mesh_file: MeshFile) -> List[str]:
    """Check that a COLLADA file gives every corner its own normal, by an index of its own."""
    return collada_normal_problems(etree.parse(file_path(mesh_file)))


def gltf_document(path: str) -> Dict[str, Any]:
    """Return the JSON chunk of a binary glTF file."""
    with open(path, 'rb') as stream:
        data = stream.read()
    magic, version, length = struct.unpack_from('<4sII', data, 0)
    json_length, kind = struct.unpack_from('<I4s', data, 12)
    if magic != b'glTF' or version != 2 or length != len(data) or kind != b'JSON':
        raise ValueError('not a binary glTF 2.0 file')
    return json.loads(data[20:20 + json_length])


def check_gltf(mesh_file: MeshFile) -> List[str]:
    """Check a glTF file's root node, its named sub-part nodes and its materials."""
    document = gltf_document(file_path(mesh_file))
    nodes = document['nodes']
    root = nodes[document['scenes'][0]['nodes'][0]]
    problems = []
    if not np.allclose(root.get('rotation', [0, 0, 0, 1]), [-np.sqrt(0.5), 0, 0, np.sqrt(0.5)]):
        problems.append("the root node doesn't turn Z up into Y up")
    children = [nodes[child]['name'] for child in root.get('children', [])]
    if children != list(mesh_file.mesh['sub_parts']):
        problems.append(f'nodes {children}, manifest {mesh_file.mesh["sub_parts"]}')
    materials = [material['name'] for material in document['materials']]
    if materials != list(mesh_file.mesh['materials']):
        problems.append(f'materials {materials}, manifest {mesh_file.mesh["materials"]}')
    return problems


def check_license(mesh_file: MeshFile) -> List[str]:
    """Check the file's license: SPDX lines in a .license file for STL and glTF, in the DAE."""
    expected = [COPYRIGHT_LINE, f'SPDX-License-Identifier: {mesh_file.license_id}']
    if mesh_file.path.endswith('.dae'):
        with open(file_path(mesh_file), encoding='utf-8') as stream:
            head = stream.read(1000)
        return [] if all(line in head for line in expected) else ['no SPDX lines in the header']
    if 'license_file' not in mesh_file.entry:
        return ['no license_file in the manifest']
    path = os.path.join(MESHES, mesh_file.entry['license_file'])
    if not os.path.exists(path):
        return [f'{mesh_file.entry["license_file"]} missing']
    with open(path, encoding='utf-8') as stream:
        found = stream.read().splitlines()
    return [] if found == expected else [f'{mesh_file.entry["license_file"]}: {found}']


FILE_CHECKS: List[Check] = [check_checksum, check_triangle_count, check_bounding_box,
                            check_units, check_same_triangles, check_closed, check_license]


# ---------------------------------------------------------------------------------------------
# Runner
# ---------------------------------------------------------------------------------------------
def collada_schema(folder: str) -> etree.XMLSchema:
    """Return the COLLADA 1.4.1 schema, downloaded once into the folder and verified."""
    path = os.path.join(folder, 'collada_schema_1_4_1.xsd')
    if not os.path.exists(path):
        os.makedirs(folder, exist_ok=True)
        with urllib.request.urlopen(SCHEMA_URL, timeout=60) as response:  # noqa: S310
            data = response.read()
        with open(path, 'wb') as download:
            download.write(data)
    with open(path, 'rb') as schema_file:
        if hashlib.sha256(schema_file.read()).hexdigest() != SCHEMA_SHA256:
            raise RuntimeError(f'{path}: unexpected checksum, delete it to download it again')

    class XmlNamespace(etree.Resolver):
        """Resolve the schema's import of xml.xsd to the stand-in."""

        def resolve(self, url: str, pubid: str, context: object) -> object:
            """Return the stand-in for xml.xsd, nothing for other URLs."""
            if url.endswith('xml.xsd'):
                return self.resolve_string(XML_NAMESPACE_SCHEMA, context)
            return None

    parser = etree.XMLParser()
    parser.resolvers.add(XmlNamespace())
    return etree.XMLSchema(etree.parse(path, parser))


def check_folder_contents(files: List[MeshFile]) -> List[str]:
    """Check that meshes/ holds exactly the manifest's files and the manifest."""
    expected = ({mesh_file.path for mesh_file in files} | {'manifest.yaml'}
                | {str(mesh_file.entry['license_file']) for mesh_file in files
                   if 'license_file' in mesh_file.entry})
    found = {os.path.relpath(os.path.join(folder, name), MESHES)
             for folder, _, names in os.walk(MESHES) for name in names}
    return ([f'meshes/{path}: not in the manifest' for path in sorted(found - expected)]
            + [f'meshes/{path}: missing' for path in sorted(expected - found)])


def main() -> int:
    """Run every check and print the results."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--schema-dir', default=os.path.join(ROOT, '.cache', 'collada'))
    args = parser.parse_args()
    files = manifest_files(load_manifest())
    schema = collada_schema(args.schema_dir)
    problems = check_folder_contents(files)
    for mesh_file in files:
        found = [problem for check in FILE_CHECKS for problem in check(mesh_file)]
        if mesh_file.path.endswith('.dae'):
            found += check_collada(mesh_file, schema)
            found += check_collada_normals(mesh_file)
        if mesh_file.path.endswith('.glb'):
            found += check_gltf(mesh_file)
        print(f'{mesh_file.path}: ' + ('; '.join(found) if found else 'ok'))
        problems += [f'{mesh_file.path}: {problem}' for problem in found]
    print(f'{len(files)} files checked, {len(problems)} problems')
    return 1 if problems else 0


if __name__ == '__main__':
    sys.exit(main())
