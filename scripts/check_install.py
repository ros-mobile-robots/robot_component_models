#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 ROS Mobile Robots
# SPDX-License-Identifier: BSD-3-Clause
"""Check the installed package: the manifest's files, the provenance pages and the license.

share/robot_component_models must hold exactly the manifest's mesh files and their .license
files, the manifest, LICENSE and package.xml, for every part its provenance page, its generator
and its frame picture, and every other Python file of the checkout's generators/, in its
sub-folders too. Besides these, it may hold only the files that catkin, ament_cmake and colcon
write for every package. The check takes an inventory from the folders' listings, links to
folders followed, and takes each entry's state once. A folder that can't be listed is a
problem. Each of the files above that is there must be a regular file that can be read: a
symbolic link counts if it leads to one, a folder or a named pipe doesn't. Only such files are
opened, so a named pipe can't make the check wait. Each installed mesh must have the
manifest's checksum, and a mesh that can't be read for the checksum is a problem too. Outside
share/robot_component_models, the check ignores the workspace's own files.

A provenance page links to its generator and to its frame picture with paths relative to the
repository. Both are installed at the same relative paths, so each installed page must be the
checkout's page byte for byte, and every relative link of an installed page must lead to a
file of the inventory, in this package, that can be read. A page of the manifest that the
checkout doesn't have is a problem: an install space keeps a page that was installed before
and removed from the checkout since. Each installed page is read once. A page that can't be
read is reported, and its links aren't checked.

Usage: python3 scripts/check_install.py INSTALL_PREFIX 1|2
Run it from a checkout of the repository: the installed pages are compared with the checkout's.
Needs only Python 3 and PyYAML, as in the ros:noetic and ros:humble images.
"""
import hashlib
import os
import posixpath
import re
import stat
import sys
from typing import Any
from typing import Dict
from typing import Iterable
from typing import List
from typing import NamedTuple
from typing import Optional
from typing import Set

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PACKAGE = 'robot_component_models'
FRAME_PICTURES = 'docs/images/frames'
# A Markdown link or picture: ](target). Targets with a scheme, such as https:, lead out of
# the package and aren't checked here
LINK_TARGET = re.compile(r'\]\(([^)\s]+)\)')
OWN_FILES = {'LICENSE', 'meshes/manifest.yaml', 'package.xml'}
CMAKE_CONFIG = {'cmake/robot_component_modelsConfig.cmake',
                'cmake/robot_component_modelsConfig-version.cmake'}
# colcon's package scripts and hooks, for builds with colcon instead of catkin_make_isolated
COLCON_PACKAGE = {f'package.{shell}' for shell in ('bash', 'dsv', 'ps1', 'sh', 'zsh')}
COLCON_HOOKS_ROS1 = {f'hook/{hook}.{kind}' for kind in ('dsv', 'ps1', 'sh')
                     for hook in ('cmake_prefix_path', 'pkg_config', 'pkg_config_path',
                                  'pkg_config_path_multiarch', 'ros_package_path')}
AMENT_FILES = {'environment/ament_prefix_path.dsv', 'environment/ament_prefix_path.sh',
               'environment/path.dsv', 'environment/path.sh',
               'hook/cmake_prefix_path.dsv', 'hook/cmake_prefix_path.ps1',
               'hook/cmake_prefix_path.sh',
               'local_setup.bash', 'local_setup.dsv', 'local_setup.sh', 'local_setup.zsh'}
BUILD_TOOL_FILES = {
    '1': CMAKE_CONFIG | COLCON_PACKAGE | COLCON_HOOKS_ROS1,
    '2': CMAKE_CONFIG | COLCON_PACKAGE | AMENT_FILES,
}
# What file_state() returns for a regular file that can be read, and for a path with nothing
READABLE = ''
ABSENT = "it isn't there"


def file_state(path: str) -> str:
    """Return READABLE, ABSENT or why the path isn't a file that can be read.

    The path's symbolic links are followed. Only a regular file counts: a folder can't be read
    as a file, and opening a named pipe would wait for a writer.
    """
    try:
        status = os.stat(path)
    except FileNotFoundError:
        return 'a link without its target' if os.path.islink(path) else ABSENT
    except OSError as error:
        return f"it can't be examined: {error.strerror or error}"
    if not stat.S_ISREG(status.st_mode):
        return "it isn't a regular file"
    if not os.access(path, os.R_OK):
        return 'it has no read permission'
    return READABLE


def read_bytes(path: str) -> Optional[bytes]:
    """Return the file's bytes, or None if reading fails. Call it only for a regular file."""
    try:
        with open(path, 'rb') as stream:
            return stream.read()
    except OSError:
        return None


class Inventory(NamedTuple):
    """What the listings of the share folder and its folders hold.

    states maps each listed entry's path, relative to the share folder, to its file_state(),
    taken once. Every later decision about an entry uses that state. unlisted names the
    folders whose entries the check can't account for, with the reason.
    """

    states: Dict[str, str]
    unlisted: List[str]


def take_inventory(share: str, file_paths: Set[str],
                   known: Optional[Dict[str, str]] = None) -> Inventory:
    """List the share folder and its folders, and take each listed entry's state once.

    Only the listings put a path into the inventory. Links to folders are followed, so the
    files behind such a link are listed too. A folder that is reached a second time, through
    a link to it or to a folder above it, isn't listed again and is reported. A folder whose
    path is one of file_paths, the expected and the build tools' files, is an entry too: it
    is there, but it isn't a regular file. A folder that can't be listed is reported with the
    reason. known holds states that the caller has taken already, such as the manifest's: a
    listed entry with such a state keeps it and isn't probed again.
    """
    known = known or {}
    states: Dict[str, str] = {}
    unlisted: List[str] = []

    def record(error: OSError) -> None:
        """Record a folder that os.walk() couldn't list."""
        unlisted.append(f"{os.path.relpath(str(error.filename), share)}: the folder can't be "
                        f'listed: {error.strerror or error}')

    listed = {os.path.realpath(share)}
    for folder, folders, names in os.walk(share, onerror=record, followlinks=True):
        kept = []
        for name in folders:
            path = os.path.join(folder, name)
            relative = os.path.relpath(path, share)
            if relative in file_paths:
                states[relative] = "it isn't a regular file"
            real = os.path.realpath(path)
            if real in listed:
                unlisted.append(f'{relative}: the folder is listed already, through another '
                                'path')
                continue
            listed.add(real)
            kept.append(name)
        folders[:] = kept
        for name in names:
            relative = os.path.relpath(os.path.join(folder, name), share)
            states[relative] = (known[relative] if relative in known
                                else file_state(os.path.join(folder, name)))
    return Inventory(states, unlisted)


def load_manifest(path: str) -> Dict[str, Any]:
    """Return the installed manifest."""
    with open(path, encoding='utf-8') as stream:
        manifest: Dict[str, Any] = yaml.safe_load(stream)
    return manifest


def manifest_checksums(manifest: Dict[str, Any]) -> Dict[str, str]:
    """Return {path relative to share/: sha256} of the manifest's mesh files."""
    return {f'meshes/{file_path}': entry['sha256']
            for part in manifest['parts'].values()
            for mesh in part['meshes'].values()
            for file_path, entry in mesh['files'].items()}


def manifest_documents(manifest: Dict[str, Any]) -> Set[str]:
    """Return the documents of the manifest's parts, relative to share/.

    These are the meshes' license files and, for every part, its provenance page, its
    generator and its frame picture.
    """
    license_files = {f'meshes/{entry["license_file"]}'
                     for part in manifest['parts'].values()
                     for mesh in part['meshes'].values()
                     for entry in mesh['files'].values() if 'license_file' in entry}
    pages = {part['provenance'] for part in manifest['parts'].values()}
    generators = {part['generator'] for part in manifest['parts'].values()}
    pictures = {f'{FRAME_PICTURES}/{name}.png' for name in manifest['parts']}
    return license_files | pages | generators | pictures


def generator_files(root: str = ROOT) -> Set[str]:
    """Return the checkout's Python files in generators/ and its sub-folders, relative to share/.

    The package installs all of them, as CMakeLists.txt's install rule does: the parts'
    generators, which the manifest names, and the modules that they share. Like that rule, the
    list leaves out the __pycache__ folders.
    """
    files = set()
    for folder, folders, names in os.walk(os.path.join(root, 'generators')):
        folders[:] = [name for name in folders if name != '__pycache__']
        files |= {os.path.relpath(os.path.join(folder, name), root).replace(os.sep, '/')
                  for name in names if name.endswith('.py')}
    return files


def check_file_set(found: Set[str], expected: Set[str], allowed: Set[str]) -> List[str]:
    """Check that the installed files are the expected ones and allowed build tool files."""
    return ([f'missing: {path}' for path in sorted(expected - found)]
            + [f'not expected: {path}' for path in sorted(found - expected - allowed)])


def check_files_can_be_read(states: Dict[str, str], paths: Iterable[str]) -> List[str]:
    """Check that each of the paths that the inventory lists is a regular file that can be read.

    states is the inventory's. A symbolic link without a target, a folder, a named pipe or a
    file without read permission has the name that the file set expects, but nobody can read
    it as a file. A path that the inventory doesn't list is left to check_file_set().
    """
    return [f"{path}: isn't a file that can be read: {states[path]}" for path in sorted(paths)
            if path in states and states[path] != READABLE]


def check_checksums(share: str, checksums: Dict[str, str], readable: Set[str]) -> List[str]:
    """Check every installed mesh against the manifest's checksum.

    readable holds the installed paths that the inventory found to be regular files that can
    be read. Only these are opened. The other meshes are reported by check_file_set() and
    check_files_can_be_read(). A mesh that can't be read now, for whatever reason, is
    reported with the error, and the other meshes are still checked.
    """
    problems = []
    for path, expected in sorted(checksums.items()):
        full = os.path.join(share, path)
        if path not in readable:
            continue
        try:
            with open(full, 'rb') as stream:
                contents = stream.read()
        except OSError as error:
            problems.append(f"{path}: can't be read for its checksum: {error.strerror or error}")
            continue
        if hashlib.sha256(contents).hexdigest() != expected:
            problems.append(f'checksum differs: {path}')
    return problems


def read_installed_pages(share: str, pages: Set[str],
                         states: Dict[str, str]) -> Dict[str, Optional[bytes]]:
    """Return {page: its bytes} of the installed pages, None for a page that can't be read.

    states is the inventory's. Only a page that it found to be a regular file that can be
    read is opened. A page that the inventory doesn't list is left out: check_file_set()
    reports it.
    """
    return {page: read_bytes(os.path.join(share, page)) if states[page] == READABLE else None
            for page in sorted(pages) if page in states}


def check_pages_are_the_checkouts(installed: Dict[str, Optional[bytes]], pages: Set[str],
                                  root: str = ROOT) -> List[str]:
    """Check that each installed provenance page is the checkout's page, byte for byte.

    installed is what read_installed_pages() returned. A page of the manifest that the
    checkout doesn't have is a problem, whether or not the install space still holds it.
    """
    problems = []
    for page in sorted(pages):
        source_state = file_state(os.path.join(root, page))
        source_bytes = (read_bytes(os.path.join(root, page)) if source_state == READABLE
                        else None)
        if source_state == ABSENT:
            problems.append(f"{page}: the checkout doesn't have this page of the manifest")
        elif source_bytes is None:
            problems.append(f"{page}: the checkout's page can't be read")
        elif page not in installed:
            continue
        elif installed[page] is None:
            problems.append(f"{page}: the installed page can't be read")
        elif installed[page] != source_bytes:
            problems.append(f"{page}: differs from the checkout's page")
    return problems


def check_links_lead_to_installed_files(installed: Dict[str, Optional[bytes]],
                                        readable: Set[str]) -> List[str]:
    """Check that every relative link of the installed provenance pages leads to a file.

    installed is what read_installed_pages() returned, readable the inventory's paths of
    regular files that can be read, relative to the share folder. A link's target is taken
    relative to its page's folder in the package and has to be one of these files: not a
    folder, not a file that is missing or can't be read, and nothing outside the package. A
    page that can't be read has no links to check: check_pages_are_the_checkouts() reports it.
    """
    problems = []
    for page, contents in sorted(installed.items()):
        if contents is None:
            continue
        targets = LINK_TARGET.findall(contents.decode('utf-8', errors='replace'))
        for target in targets:
            path = target.split('#')[0]
            if not path or re.match(r'[a-z][a-z0-9+.-]*:', path):
                continue
            if posixpath.normpath(posixpath.join(posixpath.dirname(page), path)) not in readable:
                problems.append(f"{page}: the link's target {target} isn't an installed file "
                                'that can be read')
    return problems


def main() -> int:
    """Run the checks on an install prefix and print the result."""
    if len(sys.argv) != 3 or sys.argv[2] not in BUILD_TOOL_FILES:
        print(__doc__)
        return 2
    prefix, ros_version = sys.argv[1:3]
    share = os.path.join(prefix, 'share', PACKAGE)
    manifest = os.path.join(share, 'meshes', 'manifest.yaml')
    manifest_state = file_state(manifest)
    if manifest_state != READABLE:
        print(f'{manifest}: {manifest_state}')
        return 1
    contents = load_manifest(manifest)
    checksums = manifest_checksums(contents)
    expected = set(checksums) | manifest_documents(contents) | generator_files() | OWN_FILES
    allowed = BUILD_TOOL_FILES[ros_version]
    inventory = take_inventory(share, expected | allowed,
                               {os.path.relpath(manifest, share): manifest_state})
    found = set(inventory.states)
    readable = {path for path, state in inventory.states.items() if state == READABLE}
    pages = {part['provenance'] for part in contents['parts'].values()}
    installed_pages = read_installed_pages(share, pages, inventory.states)
    problems = list(inventory.unlisted)
    problems += check_file_set(found, expected, allowed)
    problems += check_files_can_be_read(inventory.states, expected | allowed)
    problems += check_checksums(share, checksums, readable)
    problems += check_pages_are_the_checkouts(installed_pages, pages)
    problems += check_links_lead_to_installed_files(installed_pages, readable)
    for problem in problems:
        print(problem)
    print(f'{share}: {len(found)} files, {len(checksums)} meshes from the manifest, '
          f'{len(problems)} problems')
    return 1 if problems else 0


if __name__ == '__main__':
    sys.exit(main())
