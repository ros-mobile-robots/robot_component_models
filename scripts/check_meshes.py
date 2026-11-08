#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 ROS Mobile Robots
# SPDX-License-Identifier: BSD-3-Clause
"""Check the mesh files in meshes/ against meshes/manifest.yaml.

The checks read the files with other libraries than the generators (trimesh, pycollada, lxml),
so they don't share the generators' mistakes:

- meshes/ holds exactly the manifest's files, each with the manifest's checksum and size,
- every STL is closed and consistently oriented, without collapsed triangles,
- triangle counts and bounding boxes match the manifest, in each file's units,
- the COLLADA and glTF files have the STL's geometry in metres,
- every COLLADA file is valid against the COLLADA 1.4.1 schema and loads in pycollada,
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
import urllib.request

import collada
from lxml import etree
import numpy as np
import trimesh
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MESHES = os.path.join(ROOT, 'meshes')
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
STL_TO_METRES = 0.001
COPYRIGHT_LINE = 'SPDX-FileCopyrightText: 2026 ROS Mobile Robots'
# Bounding boxes in the manifest are rounded to 0.001 mm or 0.000001 m. The float32 vertices
# of the glTF files and the 7 digits of the COLLADA files add less than that.
BOX_TOLERANCE = {'millimetre': 0.0006, 'metre': 0.0000011}


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
    """Check that COLLADA and glTF files hold the STL's geometry in metres."""
    extension = os.path.splitext(mesh_file.path)[1]
    if extension == '.stl':
        size = np.ptp(geometry(mesh_file).corners.reshape(-1, 3), axis=0).max()
        return [] if 5.0 < size < 500.0 else [f'largest size {size:.3f}: not millimetres?']
    stl_path = mesh_file.path[:-len(extension)] + '.stl'
    stl = mesh_file._replace(path=stl_path, entry=mesh_file.mesh['files'][stl_path])
    ours = geometry(mesh_file).corners.reshape(-1, 3)
    reference = geometry(stl).corners.reshape(-1, 3) * STL_TO_METRES
    problems = []
    for name, ours_value, reference_value in (('min', ours.min(0), reference.min(0)),
                                              ('max', ours.max(0), reference.max(0))):
        if np.abs(ours_value - reference_value).max() > 1e-6:
            problems.append(f"bounding box {name} differs from the STL's in metres")
    return problems


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
    namespace = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
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
                            check_units, check_closed, check_license]


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
        if mesh_file.path.endswith('.glb'):
            found += check_gltf(mesh_file)
        print(f'{mesh_file.path}: ' + ('; '.join(found) if found else 'ok'))
        problems += [f'{mesh_file.path}: {problem}' for problem in found]
    print(f'{len(files)} files checked, {len(problems)} problems')
    return 1 if problems else 0


if __name__ == '__main__':
    sys.exit(main())
