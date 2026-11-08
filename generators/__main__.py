# SPDX-FileCopyrightText: 2026 ROS Mobile Robots
# SPDX-License-Identifier: BSD-3-Clause
"""Regenerate the meshes of every part and meshes/manifest.yaml, or check that they are current.

Run from the repository's root, with the packages of generators/requirements.txt:

  python3 -m generators                       write all meshes and the manifest
  python3 -m generators --check               regenerate into a temporary folder, compare
  python3 -m generators swivel_caster_30mm    write one part's meshes, not the manifest
"""
import argparse
from concurrent.futures import ProcessPoolExecutor
import difflib
import importlib
import os
import sys
import tempfile
from typing import List
from typing import Sequence

from generators import PARTS
from generators.manifest import file_facts
from generators.manifest import manifest_text
from generators.manifest import MeshEntry
from generators.manifest import PartEntry
from generators.manifest import sha256
from generators.mesh_export import LICENSE_ID
from generators.mesh_export import SPDX_LINES
from generators.mesh_export import used_materials
from generators.mesh_export import write_dae
from generators.mesh_export import write_glb
from generators.mesh_export import write_license_file
from generators.mesh_export import write_stl

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MESHES = os.path.join(ROOT, 'meshes')
MANIFEST = os.path.join(MESHES, 'manifest.yaml')
EXTENSIONS = ('.stl', '.dae', '.glb')
BINARY_EXTENSIONS = ('.stl', '.glb')  # these get a <file>.license next to them


def generate_part(name: str, out: str) -> PartEntry:
    """Write one part's meshes into out/<part>/ and return their manifest entry."""
    module = importlib.import_module(f'generators.{name}')
    os.makedirs(os.path.join(out, name), exist_ok=True)
    meshes = []
    for model in module.build():
        relative = f'{name}/{model.name}'
        write_stl(model, os.path.join(out, relative + '.stl'))
        write_dae(model, os.path.join(out, relative + '.dae'), f'generators/{name}.py')
        write_glb(model, os.path.join(out, relative + '.glb'))
        for extension in BINARY_EXTENSIONS:
            write_license_file(os.path.join(out, relative + extension))
        files = [file_facts(out, relative + extension) for extension in EXTENSIONS]
        meshes.append(MeshEntry(model.name, model.frame, [part.name for part in model.sub_parts],
                                used_materials(model), files))
        print(f'{relative}: ' + ', '.join(f'{os.path.basename(facts.path)} {facts.triangles} '
                                          f'triangles {facts.size} bytes' for facts in files),
              flush=True)
    return PartEntry(name, module.TITLE, meshes)


def generate(names: Sequence[str], out: str) -> List[PartEntry]:
    """Write the parts' meshes into out/, one process per part, return them in order."""
    with ProcessPoolExecutor(max_workers=min(len(names), os.cpu_count() or 1)) as pool:
        return list(pool.map(generate_part, names, [out] * len(names)))


def committed_files() -> List[str]:
    """Return the mesh files in meshes/, relative to it."""
    found = []
    for folder, _, files in os.walk(MESHES):
        found += [os.path.relpath(os.path.join(folder, f), MESHES) for f in files
                  if f.endswith(EXTENSIONS + ('.license',))]
    return sorted(found)


def check() -> int:
    """Regenerate everything into a temporary folder and compare with meshes/: 0 if equal."""
    with tempfile.TemporaryDirectory() as out:
        parts = generate(PARTS, out)
        text = manifest_text(parts, SPDX_LINES, LICENSE_ID)
        generated = sorted([facts.path for part in parts for mesh in part.meshes
                            for facts in mesh.files]
                           + [facts.license_file for part in parts for mesh in part.meshes
                              for facts in mesh.files if facts.license_file])
        problems = [f'{path}: regenerated file differs from meshes/{path}'
                    for path in generated if os.path.exists(os.path.join(MESHES, path))
                    and sha256(os.path.join(out, path)) != sha256(os.path.join(MESHES, path))]
    problems += [f'{path}: not generated' for path in committed_files() if path not in generated]
    problems += [f'{path}: missing in meshes/' for path in generated
                 if not os.path.exists(os.path.join(MESHES, path))]
    with open(MANIFEST, encoding='utf-8') as manifest:
        committed = manifest.read()
    if committed != text:
        problems.append('meshes/manifest.yaml differs from the regenerated manifest:')
        problems += [line.rstrip('\n') for line in difflib.unified_diff(
            committed.splitlines(True), text.splitlines(True), 'committed', 'regenerated', n=0)]
    for problem in problems:
        print(problem)
    print('regeneration check: ' + ('FAILED' if problems else 'all files byte-identical'))
    return 1 if problems else 0


def main() -> int:
    """Parse the command line and write or check the meshes."""
    parser = argparse.ArgumentParser(prog='python3 -m generators',
                                     description=__doc__.splitlines()[0])
    parser.add_argument('parts', nargs='*', metavar='PART',
                        help='parts to write (default: all, with the manifest): '
                        + ', '.join(PARTS))
    parser.add_argument('--check', action='store_true',
                        help='regenerate into a temporary folder and compare with meshes/')
    args = parser.parse_args()
    unknown = [name for name in args.parts if name not in PARTS]
    if unknown:
        parser.error(f'unknown parts: {", ".join(unknown)}')
    if args.check:
        return check()
    parts = generate(args.parts or PARTS, MESHES)
    if not args.parts:
        with open(MANIFEST, 'w', encoding='utf-8', newline='\n') as manifest:
            manifest.write(manifest_text(parts, SPDX_LINES, LICENSE_ID))
        print(f'wrote {os.path.relpath(MANIFEST, ROOT)}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
