# SPDX-FileCopyrightText: 2026 ROS Mobile Robots
# SPDX-License-Identifier: BSD-3-Clause
"""Describe the written mesh files in meshes/manifest.yaml.

The numbers are read back from the files as written: checksum, size, triangles and bounding
box in each file's own units. The manifest is written by hand, not with a YAML library, so its
text stays the same from run to run.
"""
import hashlib
import json
import os
import struct
from typing import List
from typing import NamedTuple
from typing import Sequence
from typing import Tuple
import xml.etree.ElementTree as ElementTree

import numpy as np

COLLADA_NS = '{http://www.collada.org/2005/11/COLLADASchema}'
UNITS = {'.stl': 'metre', '.dae': 'metre', '.glb': 'metre'}
UP_AXES = {'.stl': 'Z', '.dae': 'Z', '.glb': 'Y (the root node turns the Z-up vertex data)'}
DECIMALS = {'.stl': 6, '.dae': 6, '.glb': 6}


class FileFacts(NamedTuple):
    """What the manifest records about one written mesh file."""

    path: str
    sha256: str
    size: int
    triangles: int
    low: Tuple[float, float, float]
    high: Tuple[float, float, float]
    license_file: str


class MeshEntry(NamedTuple):
    """One mesh name with its frame, sub-parts, materials and its three files."""

    name: str
    frame: str
    sub_parts: List[str]
    materials: List[str]
    files: List[FileFacts]


class PartEntry(NamedTuple):
    """One part with its title and meshes."""

    name: str
    title: str
    meshes: List[MeshEntry]


def sha256(path: str) -> str:
    """Return the file's SHA-256 checksum."""
    with open(path, 'rb') as stream:
        return hashlib.sha256(stream.read()).hexdigest()


def stl_points(path: str) -> Tuple[np.ndarray, int]:
    """Return the corner points of a binary STL and its number of triangles."""
    with open(path, 'rb') as stream:
        data = stream.read()
    count = struct.unpack_from('<I', data, 80)[0]
    records = np.frombuffer(data, dtype=[('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')],
                            count=count, offset=84)
    return records['v'].reshape(-1, 3).astype(float), count


def dae_points(path: str) -> Tuple[np.ndarray, int]:
    """Return the positions of a COLLADA file written by mesh_export and its triangles."""
    root = ElementTree.parse(path).getroot()
    points = [np.array((array.text or '').split(), dtype=float).reshape(-1, 3)
              for array in root.iter(f'{COLLADA_NS}float_array')
              if array.get('id', '').endswith('-positions-array')]
    count = sum(int(triangles.get('count', 0))
                for triangles in root.iter(f'{COLLADA_NS}triangles'))
    return np.vstack(points), count


def glb_points(path: str) -> Tuple[np.ndarray, int]:
    """Return the positions of a binary glTF file written by mesh_export and its triangles."""
    with open(path, 'rb') as stream:
        data = stream.read()
    json_length = struct.unpack_from('<I', data, 12)[0]
    document = json.loads(data[20:20 + json_length])
    binary = data[20 + json_length + 8:]
    points, count = [], 0
    for mesh in document['meshes']:
        for primitive in mesh['primitives']:
            accessor = document['accessors'][primitive['attributes']['POSITION']]
            view = document['bufferViews'][accessor['bufferView']]
            points.append(np.frombuffer(binary, dtype='<f4', count=accessor['count'] * 3,
                                        offset=view['byteOffset']).reshape(-1, 3))
            count += document['accessors'][primitive['indices']]['count'] // 3
    return np.vstack(points).astype(float), count


READERS = {'.stl': stl_points, '.dae': dae_points, '.glb': glb_points}


def file_facts(root: str, relative: str) -> FileFacts:
    """Return the manifest's facts about a written file, its path relative to meshes/."""
    path = os.path.join(root, relative)
    points, triangles = READERS[os.path.splitext(path)[1]](path)
    low = tuple(float(v) for v in points.min(axis=0))
    high = tuple(float(v) for v in points.max(axis=0))
    license_file = relative + '.license' if os.path.exists(path + '.license') else ''
    return FileFacts(relative, sha256(path), os.path.getsize(path), triangles,
                     low, high, license_file)  # type: ignore[arg-type]


def vector(values: Tuple[float, ...], decimals: int) -> str:
    """Return a vector as a YAML flow sequence, without negative zeros."""
    texts = []
    for value in values:
        text = f'{value:.{decimals}f}'
        texts.append(text[1:] if float(text) == 0.0 and text.startswith('-') else text)
    return '[' + ', '.join(texts) + ']'


def quoted(text: str) -> str:
    """Return the text as a double-quoted YAML scalar."""
    return json.dumps(text, ensure_ascii=False)


def manifest_text(parts: List[PartEntry], spdx_lines: Sequence[str], license_id: str) -> str:
    """Return the manifest as YAML text, with the SPDX lines as its first comments."""
    lines = [f'# {line}' for line in spdx_lines]
    lines += ['# meshes/manifest.yaml: written by `python3 -m generators`, not by hand.',
              "# All files are in metres. Bounding boxes are in each mesh's own frame (see",
              '# frame), before any URDF transform. For glTF, the box of the vertex data, before',
              "# the root node turns them into glTF's Y up.",
              '# STL and glTF files carry their license in the <file>.license next to them.',
              'package: robot_component_models',
              f'license: {license_id}',
              'parts:']
    for part in parts:
        lines += [f'  {part.name}:',
                  f'    title: {quoted(part.title)}',
                  f'    generator: generators/{part.name}.py',
                  f'    provenance: provenance/{part.name}.md',
                  '    meshes:']
        for mesh in part.meshes:
            lines += [f'      {mesh.name}:',
                      f'        frame: {quoted(mesh.frame)}',
                      f'        sub_parts: [{", ".join(mesh.sub_parts)}]',
                      f'        materials: [{", ".join(mesh.materials)}]',
                      '        files:']
            for facts in mesh.files:
                extension = os.path.splitext(facts.path)[1]
                decimals = DECIMALS[extension]
                lines += [f'          {facts.path}:',
                          f'            sha256: {facts.sha256}',
                          f'            bytes: {facts.size}',
                          f'            units: {UNITS[extension]}',
                          f'            up_axis: {quoted(UP_AXES[extension])}',
                          f'            triangles: {facts.triangles}',
                          f'            bounding_box_min: {vector(facts.low, decimals)}',
                          f'            bounding_box_max: {vector(facts.high, decimals)}']
                if facts.license_file:
                    lines.append(f'            license_file: {facts.license_file}')
    return '\n'.join(lines) + '\n'
