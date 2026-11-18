#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 ROS Mobile Robots
# SPDX-License-Identifier: BSD-3-Clause
"""Check the installed package: the manifest's files, the provenance pages and the license.

share/robot_component_models must hold exactly the manifest's mesh files and their .license
files, the manifest, LICENSE and package.xml, for every part its provenance page, its generator
and its frame picture, and every other Python file of the checkout's generators/. Besides
these, it may hold only the files that catkin, ament_cmake and colcon write for every package.
Each installed mesh must have the manifest's checksum. Outside share/robot_component_models,
the check ignores the workspace's own files.

A provenance page links to its generator and to its frame picture with paths relative to the
repository. Both are installed at the same relative paths, so each installed page must be the
checkout's page byte for byte, and every relative link of an installed page must lead to an
installed file. A page of the manifest that the checkout doesn't have is a problem: an install
space keeps a page that was installed before and removed from the checkout since.

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
    """Return the checkout's Python files in generators/, relative to share/.

    The package installs all of them: the parts' generators, which the manifest names, and
    the modules that they share.
    """
    return {f'generators/{name}' for name in os.listdir(os.path.join(root, 'generators'))
            if name.endswith('.py')}


def check_file_set(found: Set[str], expected: Set[str], allowed: Set[str]) -> List[str]:
    """Check that the installed files are the expected ones and allowed build tool files."""
    return ([f'missing: {path}' for path in sorted(expected - found)]
            + [f'not expected: {path}' for path in sorted(found - expected - allowed)])


def check_checksums(share: str, checksums: Dict[str, str]) -> List[str]:
    """Check every installed mesh file against the manifest's checksum."""
    problems = []
    for path, expected in sorted(checksums.items()):
        full = os.path.join(share, path)
        if not os.path.exists(full):
            continue
        with open(full, 'rb') as stream:
            if hashlib.sha256(stream.read()).hexdigest() != expected:
                problems.append(f'checksum differs: {path}')
    return problems


def file_bytes(path: str) -> Optional[bytes]:
    """Return the file's bytes, or None if there is no such file or it can't be read."""
    try:
        with open(path, 'rb') as stream:
            return stream.read()
    except OSError:
        return None


def check_pages_are_the_checkouts(share: str, pages: Set[str], root: str = ROOT) -> List[str]:
    """Check that each installed provenance page is the checkout's page, byte for byte.

    A page of the manifest that the checkout doesn't have is a problem, whether or not the
    install space still holds it. A page that isn't installed is reported by check_file_set().
    """
    problems = []
    for page in sorted(pages):
        installed = os.path.join(share, page)
        source_bytes = file_bytes(os.path.join(root, page))
        if source_bytes is None:
            problems.append(f"{page}: the checkout doesn't have this page of the manifest")
        elif not os.path.lexists(installed):
            continue
        elif file_bytes(installed) is None:
            problems.append(f"{page}: the installed page can't be read")
        elif file_bytes(installed) != source_bytes:
            problems.append(f"{page}: differs from the checkout's page")
    return problems


def check_links_lead_to_installed_files(share: str, pages: Set[str]) -> List[str]:
    """Check that every relative link of the installed provenance pages has its target."""
    problems = []
    for page in sorted(pages):
        installed = os.path.join(share, page)
        if not os.path.exists(installed):
            continue
        with open(installed, encoding='utf-8') as stream:
            targets = LINK_TARGET.findall(stream.read())
        for target in targets:
            path = target.split('#')[0]
            if not path or re.match(r'[a-z][a-z0-9+.-]*:', path):
                continue
            if not os.path.exists(os.path.join(os.path.dirname(installed), path)):
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
    if not os.path.exists(manifest):
        print(f'{manifest}: missing')
        return 1
    contents = load_manifest(manifest)
    checksums = manifest_checksums(contents)
    found = installed_files(share)
    expected = set(checksums) | manifest_documents(contents) | generator_files() | OWN_FILES
    pages = {part['provenance'] for part in contents['parts'].values()}
    problems = check_file_set(found, expected, BUILD_TOOL_FILES[ros_version])
    problems += check_checksums(share, checksums)
    problems += check_pages_are_the_checkouts(share, pages)
    problems += check_links_lead_to_installed_files(share, pages)
    for problem in problems:
        print(problem)
    print(f'{share}: {len(found)} files, {len(checksums)} meshes from the manifest, '
          f'{len(problems)} problems')
    return 1 if problems else 0


if __name__ == '__main__':
    sys.exit(main())
