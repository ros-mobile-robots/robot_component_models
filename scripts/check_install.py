#!/usr/bin/env python3
# Copyright (c) 2026, ROS Mobile Robots
# SPDX-License-Identifier: BSD-3-Clause
"""Check the installed package: exactly the manifest's files, the manifest, the license.

Besides these, share/robot_component_models may hold only the files that catkin (ROS 1) or
ament_cmake and colcon (ROS 2) write for every package. Each installed mesh must have the
manifest's checksum. Outside share/robot_component_models, the check ignores the workspace's
own files.

Usage: python3 scripts/check_install.py INSTALL_PREFIX 1|2
Needs only Python 3 and PyYAML, as in the ros:noetic and ros:humble images.
"""
import hashlib
import os
import sys
from typing import Any
from typing import Dict
from typing import List
from typing import Set

import yaml

PACKAGE = 'robot_component_models'
OWN_FILES = {'LICENSE', 'meshes/manifest.yaml', 'package.xml'}
BUILD_TOOL_FILES = {
    '1': {'cmake/robot_component_modelsConfig.cmake',
          'cmake/robot_component_modelsConfig-version.cmake'},
    '2': {'cmake/robot_component_modelsConfig.cmake',
          'cmake/robot_component_modelsConfig-version.cmake',
          'environment/ament_prefix_path.dsv', 'environment/ament_prefix_path.sh',
          'environment/path.dsv', 'environment/path.sh',
          'hook/cmake_prefix_path.dsv', 'hook/cmake_prefix_path.ps1',
          'hook/cmake_prefix_path.sh',
          'local_setup.bash', 'local_setup.dsv', 'local_setup.sh', 'local_setup.zsh',
          'package.bash', 'package.dsv', 'package.ps1', 'package.sh', 'package.zsh'},
}


def installed_files(share: str) -> Set[str]:
    """Return the files under the package's share folder, relative to it."""
    return {os.path.relpath(os.path.join(folder, name), share)
            for folder, _, names in os.walk(share) for name in names}


def manifest_checksums(path: str) -> Dict[str, str]:
    """Return {path relative to share/: sha256} of the manifest's mesh files."""
    with open(path, encoding='utf-8') as stream:
        manifest: Dict[str, Any] = yaml.safe_load(stream)
    return {f'meshes/{file_path}': entry['sha256']
            for part in manifest['parts'].values()
            for mesh in part['meshes'].values()
            for file_path, entry in mesh['files'].items()}


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
    checksums = manifest_checksums(manifest)
    found = installed_files(share)
    problems = check_file_set(found, set(checksums) | OWN_FILES, BUILD_TOOL_FILES[ros_version])
    problems += check_checksums(share, checksums)
    for problem in problems:
        print(problem)
    print(f'{share}: {len(found)} files, {len(checksums)} meshes from the manifest, '
          f'{len(problems)} problems')
    return 1 if problems else 0


if __name__ == '__main__':
    sys.exit(main())
