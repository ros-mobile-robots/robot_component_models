# SPDX-FileCopyrightText: 2026 ROS Mobile Robots
# SPDX-License-Identifier: BSD-3-Clause
"""Regenerate the meshes of every part and meshes/manifest.yaml, or check that they are current.

Run from the repository's root, with the packages of generators/requirements.txt:

  python3 -m generators                       write all meshes and the manifest
  python3 -m generators swivel_caster_30mm    write one part's meshes, not the manifest
  python3 -m generators --check               regenerate into a temporary folder, compare
  python3 -m generators --check --keep DIR    regenerate into the new folder DIR, compare,
                                              and keep the regenerated files there
  python3 -m generators --millimetre-collada DIR
                                              write only the COLLADA files, in millimetres,
                                              into the new folder DIR (meshes/ stays in metres)

Checking only reads meshes/. It never writes there: the regenerated files go into a temporary
folder, or into a folder that the run creates outside meshes/. The temporary folder's place is
chosen without a test write, so a TMPDIR inside meshes/ is refused before anything is written.
The chosen place is a resolved path, and the processes that build the parts get it too.
generators/generation.py holds what those processes run.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor
import difflib
import importlib
import os
import sys
import tempfile
from typing import List
from typing import Optional
from typing import Sequence

from generators import PARTS
from generators.generation import EXTENSIONS
from generators.generation import generate
from generators.generation import use_temporary_folder
from generators.manifest import manifest_text
from generators.manifest import sha256
from generators.mesh_export import LICENSE_ID
from generators.mesh_export import MILLIMETRE
from generators.mesh_export import require_model_materials
from generators.mesh_export import SPDX_LINES
from generators.mesh_export import write_dae

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MESHES = os.path.join(ROOT, 'meshes')
MANIFEST = os.path.join(MESHES, 'manifest.yaml')
# The environment variables that name a folder for temporary files, in tempfile's order, and
# the folders that tempfile tries when no variable names one
TEMPORARY_FOLDER_VARIABLES = ('TMPDIR', 'TEMP', 'TMP')
DEFAULT_TEMPORARY_FOLDERS = ('/tmp', '/var/tmp', '/usr/tmp')


def generate_part_millimetre_collada(name: str, out: str) -> List[str]:
    """Write one part's COLLADA files in millimetres into out/<part>/ and return their paths."""
    module = importlib.import_module(f'generators.{name}')
    os.makedirs(os.path.join(out, name), exist_ok=True)
    written = []
    for model in module.build():
        require_model_materials(model)
        path = os.path.join(out, name, model.name + '.dae')
        write_dae(model, path, f'generators/{name}.py', MILLIMETRE)
        print(f'{path}: millimetres, {os.path.getsize(path)} bytes', flush=True)
        written.append(path)
    return written


def generate_millimetre_collada(names: Sequence[str], out: str) -> List[str]:
    """Write the parts' COLLADA files in millimetres into out/, one process per part."""
    with ProcessPoolExecutor(max_workers=min(len(names), os.cpu_count() or 1)) as pool:
        written = pool.map(generate_part_millimetre_collada, names, [out] * len(names))
        return [path for paths in written for path in paths]


def without_repeats(names: Sequence[str]) -> List[str]:
    """Return the names in their order, each once.

    A part that is named twice would otherwise be written by two processes at the same time,
    into the same files.
    """
    return list(dict.fromkeys(names))


def committed_files() -> List[str]:
    """Return the mesh files in meshes/, relative to it."""
    found = []
    for folder, _, files in os.walk(MESHES):
        found += [os.path.relpath(os.path.join(folder, f), MESHES) for f in files
                  if f.endswith(EXTENSIONS + ('.license',))]
    return sorted(found)


def is_inside(path: str, folder: str) -> bool:
    """Return whether the path is the folder or lies inside it, symbolic links resolved."""
    path, folder = os.path.realpath(path), os.path.realpath(folder)
    return path == folder or path.startswith(folder + os.sep)


def new_folder_path(folder: str) -> str:
    """Return the one path that stands for DIR: its parent resolved, its last name as given.

    The path has no '.', no '..' and no symbolic link in its parent, so it names the same
    place when it is checked, when it is created and when the files are written. The last
    name is not resolved: if a symbolic link has that name, the path names the link itself.
    Raise ValueError if DIR doesn't end with a name, as '', '/' and 'folder/..' do.
    """
    parent, name = os.path.split(folder.rstrip(os.sep))
    while name == os.curdir:
        parent, name = os.path.split(parent.rstrip(os.sep))
    if name in ('', os.pardir):
        raise ValueError('DIR has to end with the name of the new folder')
    return os.path.join(os.path.realpath(parent), name)


def create_output_folder(folder: str) -> str:
    """Create the new folder DIR for regenerated files and return its path.

    --check --keep and --millimetre-collada write into such a folder. The folder must lie
    outside meshes/ and must not exist yet. A folder that this run creates holds no file and
    no link to one. So the regenerated files can't replace an existing file:
    not a file in meshes/, not a copy of one somewhere else, and not through a symbolic or a
    hard link that an existing folder could hold. os.mkdir() creates the folder and fails if
    anything has its name already, also a symbolic link without a target.
    Raise ValueError with the reason if the folder can't be created.
    """
    destination = new_folder_path(folder)
    if is_inside(destination, MESHES):
        raise ValueError('DIR must lie outside meshes/')
    if os.path.lexists(destination):
        raise ValueError(f'{destination} exists already. The files go only into a folder that '
                         'this run creates, so that they never replace an existing file. '
                         'Remove the folder or name another one')
    try:
        os.makedirs(os.path.dirname(destination), exist_ok=True)
        os.mkdir(destination)
    except OSError as error:
        raise ValueError(f"{destination} can't be created: {error.strerror}") from None
    return destination


def temporary_parent() -> str:
    """Return a folder outside meshes/ for the check's temporary files.

    tempfile finds its folder by writing a test file into each candidate: the folders that the
    environment names, then the system's. A candidate inside meshes/ would get that test file.
    So this function chooses without writing: the first candidate that exists, is writable and
    lies outside meshes/. A variable that names a folder inside meshes/ is an error, because
    the caller asked for temporary files there.

    Each candidate is resolved once, and the checks and the returned path use that resolved
    path. A spelling such as link/../meshes names the folder that the system reaches through
    the link, and not the folder that is left when '..' is cancelled against 'link'.
    Raise ValueError with the reason if no folder can be used.
    """
    named = []
    for variable in TEMPORARY_FOLDER_VARIABLES:
        folder = os.environ.get(variable)
        if folder and is_inside(folder, MESHES):
            raise ValueError(f'{variable} names {folder}, which lies inside meshes/. Set '
                             f'{variable} to a folder outside meshes/')
        if folder:
            named.append(folder)
    for folder in named + list(DEFAULT_TEMPORARY_FOLDERS):
        resolved = os.path.realpath(folder)
        if (os.path.isdir(resolved) and os.access(resolved, os.W_OK | os.X_OK)
                and not is_inside(resolved, MESHES)):
            return resolved
    raise ValueError('no writable folder for temporary files outside meshes/. Set TMPDIR to '
                     'one')


def regeneration_problems(out: str, temporary: str) -> List[str]:
    """Regenerate everything into out/ and return how the result differs from meshes/.

    The processes that build the parts put their temporary files into the temporary folder.
    """
    parts = generate(PARTS, out, temporary)
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
    return problems


def check(temporary: str, keep: Optional[str] = None) -> int:
    """Regenerate everything outside meshes/ and compare with meshes/: 0 if equal.

    temporary is the folder for temporary files that temporary_parent() chose. With keep, the
    regenerated files go into that folder, which create_output_folder() has made, and stay
    there for a closer look. Without it, they go into a new folder in the temporary folder,
    which is made only then. meshes/ is only read.
    """
    if keep is not None:
        problems = regeneration_problems(keep, temporary)
    else:
        with tempfile.TemporaryDirectory(dir=temporary) as regenerated:
            problems = regeneration_problems(regenerated, temporary)
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
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--check', action='store_true',
                      help='regenerate every part outside meshes/ and compare with meshes/, '
                      'which is only read')
    mode.add_argument('--millimetre-collada', metavar='DIR',
                      help='write only the COLLADA files, in millimetres and with that unit '
                      'declared, into the new folder DIR, as DIR/<part>/<name>.dae. DIR must '
                      'not exist yet and must lie outside meshes/, whose files stay in metres')
    parser.add_argument('--keep', metavar='DIR',
                        help='with --check: regenerate into the new folder DIR and keep the '
                        'files. DIR must not exist yet and must lie outside meshes/')
    args = parser.parse_args()
    unknown = [name for name in args.parts if name not in PARTS]
    if unknown:
        parser.error(f'unknown parts: {", ".join(unknown)}')
    if args.keep is not None and not args.check:
        parser.error('--keep DIR needs --check: it keeps the files that the check regenerates')
    if args.check and args.parts:
        parser.error('--check regenerates and compares every part: leave out the part names')
    if args.check:
        try:
            temporary = temporary_parent()
        except ValueError as problem:
            parser.error(f'--check: {problem}')
        # From here on, every use of tempfile takes this folder without a test write, also in
        # the libraries that the generators load
        use_temporary_folder(temporary)
        keep = None
        if args.keep is not None:
            try:
                keep = create_output_folder(args.keep)
            except ValueError as problem:
                parser.error(f'--keep: {problem}')
        return check(temporary, keep)
    if args.millimetre_collada is not None:
        try:
            destination = create_output_folder(args.millimetre_collada)
        except ValueError as problem:
            parser.error(f'--millimetre-collada: {problem}')
        written = generate_millimetre_collada(without_repeats(args.parts) or PARTS, destination)
        print(f'COLLADA files written in millimetres: {len(written)}')
        return 0
    parts = generate(without_repeats(args.parts) or PARTS, MESHES)
    if not args.parts:
        with open(MANIFEST, 'w', encoding='utf-8', newline='\n') as manifest:
            manifest.write(manifest_text(parts, SPDX_LINES, LICENSE_ID))
        print(f'wrote {os.path.relpath(MANIFEST, ROOT)}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
