#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 ROS Mobile Robots
# SPDX-License-Identifier: BSD-3-Clause
"""Check the installed package: the manifest's files, the provenance pages and the license.

share/robot_component_models must hold exactly the manifest's mesh files and their .license
files, the manifest, LICENSE and package.xml, for every part its provenance page, its generator
and its frame picture, and every other Python file of the checkout's generators/, in its
sub-folders too. Besides these, it may hold only the files that catkin, ament_cmake and colcon
write for every package. Each of these files must be a file that can be read: a symbolic link
counts if it leads to one. Each installed mesh must have the manifest's checksum. Outside
share/robot_component_models, the check ignores the workspace's own files.

A provenance page links to its generator and to its frame picture with paths relative to the
repository. Both are installed at the same relative paths, so each installed page must be the
checkout's page byte for byte, and every relative link of an installed page must lead to an
installed file. A page of the manifest that the checkout doesn't have is a problem: an install
space keeps a page that was installed before and removed from the checkout since. Each
installed page is read once. A page that can't be read is reported, and its links aren't
checked.

Usage: python3 scripts/check_install.py INSTALL_PREFIX 1|2
Run it from a checkout of the repository: the installed pages are compared with the checkout's.
Needs only Python 3 and PyYAML, as in the ros:noetic and ros:humble images.
"""
import hashlib
import os
import re
import sys
from typing import Any
from typing import Dict
from typing import Iterable
from typing import List
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


def installed_files(share: str) -> Set[str]:
    """Return the files under the package's share folder, relative to it."""
    return {os.path.relpath(os.path.join(folder, name), share)
            for folder, _, names in os.walk(share) for name in names}


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


def file_bytes(path: str) -> Optional[bytes]:
    """Return the file's bytes, or None if there is no such file or it can't be read."""
    try:
        with open(path, 'rb') as stream:
            return stream.read()
    except OSError:
        return None


def check_files_can_be_read(share: str, paths: Iterable[str]) -> List[str]:
    """Check that each installed path is a file that can be read, through its links.

    A symbolic link without a target, a folder or a file without read permission has the
    name that the file set expects, but nobody can read it.
    """
    return [f"{path}: isn't a file that can be read" for path in sorted(paths)
            if not (os.path.isfile(os.path.join(share, path))
                    and os.access(os.path.join(share, path), os.R_OK))]


def check_checksums(share: str, checksums: Dict[str, str]) -> List[str]:
    """Check every installed mesh file against the manifest's checksum.

    A file that is missing or can't be read is left to check_file_set() and
    check_files_can_be_read().
    """
    problems = []
    for path, expected in sorted(checksums.items()):
        contents = file_bytes(os.path.join(share, path))
        if contents is not None and hashlib.sha256(contents).hexdigest() != expected:
            problems.append(f'checksum differs: {path}')
    return problems


def read_installed_pages(share: str, pages: Set[str]) -> Dict[str, Optional[bytes]]:
    """Return {page: its bytes} of the installed pages, None for a page that can't be read.

    A page that isn't installed is left out: check_file_set() reports it.
    """
    return {page: file_bytes(os.path.join(share, page)) for page in sorted(pages)
            if os.path.lexists(os.path.join(share, page))}


def check_pages_are_the_checkouts(installed: Dict[str, Optional[bytes]], pages: Set[str],
                                  root: str = ROOT) -> List[str]:
    """Check that each installed provenance page is the checkout's page, byte for byte.

    installed is what read_installed_pages() returned. A page of the manifest that the
    checkout doesn't have is a problem, whether or not the install space still holds it.
    """
    problems = []
    for page in sorted(pages):
        source_bytes = file_bytes(os.path.join(root, page))
        if source_bytes is None:
            problems.append(f"{page}: the checkout doesn't have this page of the manifest")
        elif page not in installed:
            continue
        elif installed[page] is None:
            problems.append(f"{page}: the installed page can't be read")
        elif installed[page] != source_bytes:
            problems.append(f"{page}: differs from the checkout's page")
    return problems


def check_links_lead_to_installed_files(share: str,
                                        installed: Dict[str, Optional[bytes]]) -> List[str]:
    """Check that every relative link of the installed provenance pages has its target.

    installed is what read_installed_pages() returned. A page that can't be read has no links
    to check: check_pages_are_the_checkouts() reports it.
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
            if not os.path.exists(os.path.join(share, os.path.dirname(page), path)):
                problems.append(f"{page}: the link's target {target} isn't installed")
    return problems


def main() -> int:
    """Run the checks on an install prefix and print the result."""
    if len(sys.argv) != 3 or sys.argv[2] not in BUILD_TOOL_FILES:
        print(__doc__)
        return 2
    prefix, ros_version = sys.argv[1:3]
    share = os.path.join(prefix, 'share', PACKAGE)
    manifest = os.path.join(share, 'meshes', 'manifest.yaml')
    if not (os.path.isfile(manifest) and os.access(manifest, os.R_OK)):
        print(f"{manifest}: missing, or it can't be read")
        return 1
    contents = load_manifest(manifest)
    checksums = manifest_checksums(contents)
    found = installed_files(share)
    expected = set(checksums) | manifest_documents(contents) | generator_files() | OWN_FILES
    pages = {part['provenance'] for part in contents['parts'].values()}
    installed_pages = read_installed_pages(share, pages)
    problems = check_file_set(found, expected, BUILD_TOOL_FILES[ros_version])
    problems += check_files_can_be_read(share, found & expected)
    problems += check_checksums(share, checksums)
    problems += check_pages_are_the_checkouts(installed_pages, pages)
    problems += check_links_lead_to_installed_files(share, installed_pages)
    for problem in problems:
        print(problem)
    print(f'{share}: {len(found)} files, {len(checksums)} meshes from the manifest, '
          f'{len(problems)} problems')
    return 1 if problems else 0


if __name__ == '__main__':
    sys.exit(main())
