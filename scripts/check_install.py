#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 ROS Mobile Robots
# SPDX-License-Identifier: BSD-3-Clause
"""Check the installed package: the manifest's files, the provenance pages and the license.

share/robot_component_models must hold exactly the manifest's mesh files and their .license
files, the manifest, a provenance page per part, LICENSE and package.xml. Besides these, it may
hold only the files that catkin, ament_cmake and colcon write for every package. Each installed
mesh must have the manifest's checksum. Outside share/robot_component_models, the check ignores
the workspace's own files.

A provenance page in the repository links to its generator and pictures with paths relative to
the repository. The installed package holds neither, so CMakeLists.txt gives each installed
page links to the repository on GitHub, at the tag of the installed version. Each installed
page must be the repository's page with exactly these links replaced, and must have no link
left that leads out of the installed provenance folder.

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
from typing import Set

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PACKAGE = 'robot_component_models'
REPOSITORY_URL = 'https://github.com/ros-mobile-robots/robot_component_models'
# A Markdown link or picture whose target leads out of the page's folder: ](../
LINK_OUT_OF_FOLDER = '](../'
PICTURES = 'docs/images/'
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
    """Return the license files and provenance pages the manifest names, relative to share/."""
    license_files = {f'meshes/{entry["license_file"]}'
                     for part in manifest['parts'].values()
                     for mesh in part['meshes'].values()
                     for entry in mesh['files'].values() if 'license_file' in entry}
    return license_files | {part['provenance'] for part in manifest['parts'].values()}


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


def installed_version(share: str) -> str:
    """Return the version in the installed package.xml."""
    with open(os.path.join(share, 'package.xml'), encoding='utf-8') as stream:
        found = re.search(r'<version>([^<]+)</version>', stream.read())
    return found.group(1) if found else ''


def with_repository_links(page: str, version: str) -> str:
    """Return a provenance page's text with its links into the repository as GitHub links.

    A picture in docs/images/ becomes a link to the file itself at the version's tag, every
    other path that leaves the provenance folder a link to the file's page at that tag. These
    are the replacements of CMakeLists.txt, in the same order.
    """
    pictures = f'{LINK_OUT_OF_FOLDER}{PICTURES}'
    page = page.replace(pictures, f']({REPOSITORY_URL}/raw/{version}/{PICTURES}')
    return page.replace(LINK_OUT_OF_FOLDER, f']({REPOSITORY_URL}/blob/{version}/')


def check_provenance_links(share: str, pages: Set[str]) -> List[str]:
    """Check that each installed provenance page is the repository's page with GitHub links."""
    version = installed_version(share)
    if not version:
        return ['package.xml: no <version> element']
    problems = []
    for page in sorted(pages):
        installed_path, source_path = os.path.join(share, page), os.path.join(ROOT, page)
        if not os.path.exists(installed_path) or not os.path.exists(source_path):
            continue
        with open(installed_path, encoding='utf-8') as stream:
            installed = stream.read()
        with open(source_path, encoding='utf-8') as stream:
            expected = with_repository_links(stream.read(), version)
        if LINK_OUT_OF_FOLDER in installed:
            problems.append(f'{page}: a link leads out of the installed provenance folder')
        if installed != expected:
            problems.append(f"{page}: differs from the repository's page with its links at "
                            f'{REPOSITORY_URL}/blob/{version}/')
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
    expected = set(checksums) | manifest_documents(contents) | OWN_FILES
    problems = check_file_set(found, expected, BUILD_TOOL_FILES[ros_version])
    problems += check_checksums(share, checksums)
    problems += check_provenance_links(share, {part['provenance']
                                               for part in contents['parts'].values()})
    for problem in problems:
        print(problem)
    print(f'{share}: {len(found)} files, {len(checksums)} meshes from the manifest, '
          f'{len(problems)} problems')
    return 1 if problems else 0


if __name__ == '__main__':
    sys.exit(main())
