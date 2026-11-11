#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 ROS Mobile Robots
# SPDX-License-Identifier: BSD-3-Clause
"""Check that `python3 -m generators --millimetre-collada DIR` never changes an existing file.

The option writes COLLADA files in millimetres. A released file in meshes/ is in metres and
must never get another unit under its name. So the option writes only into a folder that it
creates itself, outside meshes/. This script runs the real command with the folders that the
option has to refuse, and with one that it has to accept:

- meshes/ itself, a folder inside it, and paths that lead into it through a symbolic link,
- a folder that exists already: an empty one, one whose part folder is a symbolic link into
  meshes/, one whose COLLADA file is a symbolic link or a hard link to a released file, and
  the folder of an earlier run,
- a symbolic link whose target doesn't exist,
- each existing folder and link once more, spelled <parent>/missing/../<name>/. with a folder
  `missing` that doesn't exist: the same place under another spelling,
- a new folder, also in that spelling, which must then hold the part's millimetre file and
  nothing else.

A refusal must end with exit code 2, a usage error, and must leave every file as it was. Each
case runs on a copy of generators/ and meshes/ of its own, in a temporary folder. So a failing
case can't change the repository's own files, and it can't hide the next case's failure.

Usage: python3 scripts/check_millimetre_option.py
Run it with the packages of generators/requirements.txt. Exit code 0 if every case passes.
"""
import functools
import hashlib
import os
import shutil
import subprocess
import sys
import tempfile
from typing import Callable
from typing import Dict
from typing import List
from typing import NamedTuple
from typing import Tuple

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# The part of the cases: the one with the fewest triangles, which is the quickest to generate
PART = 'raspberry_pi_camera_v2'
PART_FILE = os.path.join(PART, PART + '.dae')
MILLIMETRE_UNIT = '<unit name="millimeter" meter="0.001"/>'
USAGE_ERROR = 2


class Sandbox(NamedTuple):
    """A copy of the repository's generators/ and meshes/, and an empty folder for one case."""

    repository: str
    work: str


# Prepares a case in the sandbox and returns the folder to try as DIR
Case = Callable[[Sandbox], str]
# Runs the option in the sandbox and returns the problems it finds
Check = Callable[[Sandbox], List[str]]


def meshes(sandbox: Sandbox) -> str:
    """Return the sandbox's meshes/ folder."""
    return os.path.join(sandbox.repository, 'meshes')


def run_option(sandbox: Sandbox, folder: str) -> 'subprocess.CompletedProcess[str]':
    """Run the generators with --millimetre-collada for the cases' part, in the sandbox."""
    return subprocess.run([sys.executable, '-m', 'generators', PART,
                           '--millimetre-collada', folder],
                          cwd=sandbox.repository, check=False, capture_output=True, text=True,
                          env=dict(os.environ, PYTHONDONTWRITEBYTECODE='1'))


def snapshot(sandbox: Sandbox) -> Dict[str, str]:
    """Return what the sandbox's meshes/ and work folder hold: each entry with its content.

    A file's content is its checksum. A symbolic link's content is its target, and the link
    isn't followed.
    """
    entries = {}
    for top in (meshes(sandbox), sandbox.work):
        for parent, folders, files in os.walk(top):
            for name in folders + files:
                path = os.path.join(parent, name)
                if os.path.islink(path):
                    entries[path] = 'link to ' + os.readlink(path)
                elif os.path.isdir(path):
                    entries[path] = 'folder'
                else:
                    with open(path, 'rb') as stream:
                        entries[path] = hashlib.sha256(stream.read()).hexdigest()
    return entries


# ---------------------------------------------------------------------------------------------
# Folders that the option has to refuse
# ---------------------------------------------------------------------------------------------
def meshes_itself(sandbox: Sandbox) -> str:
    """Return meshes/ itself."""
    return meshes(sandbox)


def new_folder_inside_meshes(sandbox: Sandbox) -> str:
    """Return a folder inside meshes/ that doesn't exist."""
    return os.path.join(meshes(sandbox), 'millimetre')


def link_to_meshes(sandbox: Sandbox) -> str:
    """Return a symbolic link to meshes/."""
    link = os.path.join(sandbox.work, 'link')
    os.symlink(meshes(sandbox), link)
    return link


def new_folder_below_a_link_to_meshes(sandbox: Sandbox) -> str:
    """Return a folder that doesn't exist, below a symbolic link to meshes/."""
    return os.path.join(link_to_meshes(sandbox), 'millimetre')


def link_to_nowhere(sandbox: Sandbox) -> str:
    """Return a symbolic link whose target doesn't exist."""
    link = os.path.join(sandbox.work, 'link')
    os.symlink(os.path.join(sandbox.work, 'nowhere'), link)
    return link


def existing_empty_folder(sandbox: Sandbox) -> str:
    """Return an empty folder that exists."""
    folder = os.path.join(sandbox.work, 'millimetre')
    os.mkdir(folder)
    return folder


def folder_with_a_linked_part_folder(sandbox: Sandbox) -> str:
    """Return a folder whose part folder is a symbolic link to the part's folder in meshes/."""
    folder = existing_empty_folder(sandbox)
    os.symlink(os.path.join(meshes(sandbox), PART), os.path.join(folder, PART))
    return folder


def folder_with_a_linked_file(sandbox: Sandbox) -> str:
    """Return a folder whose COLLADA file is a symbolic link to the released file."""
    folder = existing_empty_folder(sandbox)
    os.mkdir(os.path.join(folder, PART))
    os.symlink(os.path.join(meshes(sandbox), PART_FILE), os.path.join(folder, PART_FILE))
    return folder


def folder_with_a_hard_linked_file(sandbox: Sandbox) -> str:
    """Return a folder whose COLLADA file is a hard link to the released file."""
    folder = existing_empty_folder(sandbox)
    os.mkdir(os.path.join(folder, PART))
    os.link(os.path.join(meshes(sandbox), PART_FILE), os.path.join(folder, PART_FILE))
    return folder


def folder_of_an_earlier_run(sandbox: Sandbox) -> str:
    """Return a folder that an earlier run of the option has written."""
    folder = os.path.join(sandbox.work, 'millimetre')
    earlier = run_option(sandbox, folder)
    if earlier.returncode != 0:
        raise RuntimeError(f'the earlier run failed: {earlier.stderr.strip()[-300:]}')
    return folder


def new_folder_below_a_link_into_meshes_without_a_target(sandbox: Sandbox) -> str:
    """Return a folder below a symbolic link to a folder in meshes/ that doesn't exist."""
    link = os.path.join(sandbox.work, 'link')
    os.symlink(os.path.join(meshes(sandbox), 'missing'), link)
    return os.path.join(link, 'millimetre')


def existing_folder_that_ends_with_two_dots(sandbox: Sandbox) -> str:
    """Return an existing empty folder, spelled with a missing folder and '..' at its end."""
    return os.path.join(existing_empty_folder(sandbox), 'missing', os.pardir)


def through_a_missing_folder(folder: str) -> str:
    """Return the folder spelled <parent>/missing/../<name>/. where `missing` doesn't exist.

    The spelling names the same place as the plain path. A function that creates folders
    along the spelled path would create `missing` and could take the existing folder for a
    new one.
    """
    parent, name = os.path.split(folder)
    return os.path.join(parent, 'missing', os.pardir, name, os.curdir)


def spelled_through_a_missing_folder(case: Case) -> Case:
    """Return the case with its folder spelled through a missing folder."""
    def spelled(sandbox: Sandbox) -> str:
        return through_a_missing_folder(case(sandbox))
    spelled.__name__ = f'{case.__name__}_through_a_missing_folder'
    return spelled


EXISTING: List[Case] = [link_to_meshes, link_to_nowhere, existing_empty_folder,
                        folder_with_a_linked_part_folder, folder_with_a_linked_file,
                        folder_with_a_hard_linked_file, folder_of_an_earlier_run]
REFUSED: List[Case] = ([meshes_itself, new_folder_inside_meshes,
                        new_folder_below_a_link_to_meshes,
                        new_folder_below_a_link_into_meshes_without_a_target,
                        existing_folder_that_ends_with_two_dots]
                       + EXISTING
                       + [spelled_through_a_missing_folder(case) for case in EXISTING])
# The new folder of the accepted cases, relative to the sandbox's work folder, and every entry
# that the work folder holds after the run
NEW_FOLDER = os.path.join('new', 'millimetre')
NEW_ENTRIES = ['new', NEW_FOLDER, os.path.join(NEW_FOLDER, PART),
               os.path.join(NEW_FOLDER, PART_FILE)]


# ---------------------------------------------------------------------------------------------
# Checks
# ---------------------------------------------------------------------------------------------
def check_refusal(case: Case, sandbox: Sandbox) -> List[str]:
    """Check that the option refuses the case's folder and leaves every file as it was."""
    folder = case(sandbox)
    before = snapshot(sandbox)
    result = run_option(sandbox, folder)
    problems = []
    if result.returncode != USAGE_ERROR:
        problems.append(f'exit code {result.returncode}, expected {USAGE_ERROR}')
    after = snapshot(sandbox)
    changed = sorted(path for path in set(before) | set(after)
                     if before.get(path) != after.get(path))
    problems += [f'{os.path.relpath(path, os.path.dirname(sandbox.repository))} changed'
                 for path in changed]
    return problems


def check_new_folder(spelling: Callable[[str], str], sandbox: Sandbox) -> List[str]:
    """Check that the option creates the new folder and writes only the millimetre file.

    The folder is passed to the option in the given spelling.
    """
    folder = os.path.join(sandbox.work, NEW_FOLDER)
    released = snapshot(sandbox)
    result = run_option(sandbox, spelling(folder))
    if result.returncode != 0:
        return [f'exit code {result.returncode}, expected 0: {result.stderr.strip()[-300:]}']
    after = snapshot(sandbox)
    created = sorted(os.path.relpath(path, sandbox.work) for path in set(after) - set(released))
    if created != sorted(NEW_ENTRIES):
        return [f'the run created {created}, expected {sorted(NEW_ENTRIES)}']
    problems = []
    with open(os.path.join(folder, PART_FILE), encoding='utf-8') as stream:
        if MILLIMETRE_UNIT not in stream.read():
            problems.append(f'{PART_FILE} does not declare {MILLIMETRE_UNIT}')
    if any(after.get(path) != content for path, content in released.items()):
        problems.append('a file in meshes/ changed')
    return problems


# ---------------------------------------------------------------------------------------------
# Runner
# ---------------------------------------------------------------------------------------------
def new_sandbox(folder: str) -> Sandbox:
    """Copy generators/ and meshes/ into the folder, next to an empty work folder."""
    sandbox = Sandbox(os.path.join(folder, 'repository'), os.path.join(folder, 'work'))
    for name in ('generators', 'meshes'):
        shutil.copytree(os.path.join(ROOT, name), os.path.join(sandbox.repository, name),
                        ignore=shutil.ignore_patterns('__pycache__'))
    os.mkdir(sandbox.work)
    return sandbox


def main() -> int:
    """Run every case in a sandbox of its own and print the results."""
    checks: List[Tuple[str, Check]] = [(f'refuses {case.__name__}',
                                        functools.partial(check_refusal, case))
                                       for case in REFUSED]
    checks.append(('writes into a new folder', functools.partial(check_new_folder, str)))
    checks.append(('writes into a new folder through a missing folder',
                   functools.partial(check_new_folder, through_a_missing_folder)))
    failed = 0
    for name, check in checks:
        with tempfile.TemporaryDirectory() as temporary:
            problems = check(new_sandbox(temporary))
        print(f'{name}: ' + ('; '.join(problems) if problems else 'ok'))
        failed += bool(problems)
    print(f'{len(checks)} cases, {failed} failed')
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
