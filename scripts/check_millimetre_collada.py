#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 ROS Mobile Robots
# SPDX-License-Identifier: BSD-3-Clause
"""Check a folder of millimetre COLLADA files against the files in metres in meshes/.

`python3 -m generators --millimetre-collada DIR` writes every COLLADA file once more into DIR,
with the positions in millimetres. A program that applies a COLLADA file's <unit> element must
load the same mesh from the file in DIR and from the file in meshes/. The checks read the
files with lxml, not with the generators' code:

- DIR holds one file for each COLLADA file in meshes/, at the same relative path, and nothing
  else,
- every file in DIR declares <unit name="millimeter" meter="0.001"/>, and its file in meshes/
  declares the metre,
- every position in DIR and in meshes/ is a finite number, and every position in DIR is 1000
  times the position in meshes/, within the precision of the seven significant digits that
  the generators write,
- apart from the <unit> element and the positions, the two files have the same text.

Usage: python3 scripts/check_millimetre_collada.py DIR
Exit code 0 if every check passes.
"""
import argparse
import copy
import glob
import os
import sys
from typing import Any
from typing import Callable
from typing import Dict
from typing import List
from typing import NamedTuple

from lxml import etree
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MESHES = os.path.join(ROOT, 'meshes')
NAMESPACE = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
METRE = {'name': 'meter', 'meter': '1'}
MILLIMETRE = {'name': 'millimeter', 'meter': '0.001'}
MILLIMETRES_PER_METRE = 1000.0
# The generators write each position with seven significant digits, so a written number lies
# within half a unit of its seventh digit of the exact value. The number in millimetres and
# 1000 times the number in metres then differ by at most one unit of the seventh digit, which
# is a relative difference of 1e-6. The last percent covers the comparison's own arithmetic.
RELATIVE_TOLERANCE = 1.01e-6


class FilePair(NamedTuple):
    """A COLLADA file's path in meshes/ with the parsed file in metres and in millimetres."""

    path: str
    metre: Any
    millimetre: Any


Check = Callable[[FilePair], List[str]]


def collada_files() -> List[str]:
    """Return the COLLADA files in meshes/, relative to it."""
    return sorted(os.path.relpath(path, MESHES)
                  for path in glob.glob(os.path.join(MESHES, '*', '*.dae')))


def declared_unit(document: Any) -> Dict[str, str]:
    """Return the attributes of the file's <unit> element."""
    return {name: value
            for element in document.xpath('/c:COLLADA/c:asset/c:unit', namespaces=NAMESPACE)
            for name, value in element.attrib.items()}


def position_arrays(document: Any) -> List[Any]:
    """Return the <float_array> elements that the file's <vertices> use as positions."""
    sources = document.xpath('//c:mesh/c:vertices/c:input[@semantic="POSITION"]/@source',
                             namespaces=NAMESPACE)
    return [array for source in sources
            for array in document.xpath('//c:source[@id=$id]/c:float_array',
                                        id=source.lstrip('#'), namespaces=NAMESPACE)]


def numbers(array: Any) -> np.ndarray:
    """Return the numbers of a <float_array> element."""
    return np.array((array.text or '').split(), dtype=float)


def text_without_unit_and_positions(document: Any) -> bytes:
    """Return the file's text with the <unit> attributes and the positions left out."""
    rest = copy.deepcopy(document)
    for element in rest.xpath('/c:COLLADA/c:asset/c:unit', namespaces=NAMESPACE):
        element.attrib.clear()
    for array in position_arrays(rest):
        array.text = None
    return etree.tostring(rest)


# ---------------------------------------------------------------------------------------------
# Checks
# ---------------------------------------------------------------------------------------------
def check_folder_contents(folder: str) -> List[str]:
    """Check that the folder holds exactly one file for each COLLADA file in meshes/."""
    expected = set(collada_files())
    found = {os.path.relpath(os.path.join(parent, name), folder)
             for parent, _, names in os.walk(folder) for name in names}
    return ([f'{path}: no COLLADA file of meshes/ has this path'
             for path in sorted(found - expected)]
            + [f'{path}: missing' for path in sorted(expected - found)])


def check_units(pair: FilePair) -> List[str]:
    """Check that the file in meshes/ declares the metre and the other file the millimetre."""
    problems = []
    metre_unit, millimetre_unit = declared_unit(pair.metre), declared_unit(pair.millimetre)
    if metre_unit != METRE:
        problems.append(f'the file in meshes/ declares the unit {metre_unit}, not {METRE}')
    if millimetre_unit != MILLIMETRE:
        problems.append(f'declares the unit {millimetre_unit}, not {MILLIMETRE}')
    return problems


def check_positions(pair: FilePair) -> List[str]:
    """Check that every position is finite and 1000 times the position in meshes/.

    NaN and infinity are valid numbers for a parser but no positions. A comparison with NaN
    is false whatever the other number is, so the finite numbers are checked first.
    """
    metre_arrays = position_arrays(pair.metre)
    millimetre_arrays = position_arrays(pair.millimetre)
    if not metre_arrays:
        return ['the file in meshes/ has no positions']
    if ([array.get('id') for array in millimetre_arrays]
            != [array.get('id') for array in metre_arrays]):
        return ['the position arrays have other names than in meshes/']
    problems = []
    for metre_array, millimetre_array in zip(metre_arrays, millimetre_arrays):
        expected = numbers(metre_array) * MILLIMETRES_PER_METRE
        found = numbers(millimetre_array)
        if found.shape != expected.shape:
            problems.append(f'{millimetre_array.get("id")}: {found.size} numbers, '
                            f'{expected.size} in meshes/')
            continue
        finite = np.isfinite(found) & np.isfinite(expected)
        if not finite.all():
            problems.append(f'{millimetre_array.get("id")}: {int((~finite).sum())} of '
                            f'{found.size} numbers are not finite, here or in meshes/')
        difference = np.abs(found[finite] - expected[finite])
        off = difference > RELATIVE_TOLERANCE * np.abs(expected[finite])
        if off.any():
            problems.append(f'{millimetre_array.get("id")}: {int(off.sum())} of {found.size} '
                            'numbers are not 1000 times the numbers in meshes/')
    return problems


def check_rest(pair: FilePair) -> List[str]:
    """Check that the two files have the same text apart from the unit and the positions."""
    same = (text_without_unit_and_positions(pair.millimetre)
            == text_without_unit_and_positions(pair.metre))
    return [] if same else ['differs from the file in meshes/ in more than the unit and the '
                            'positions']


PAIR_CHECKS: List[Check] = [check_units, check_positions, check_rest]


# ---------------------------------------------------------------------------------------------
# Runner
# ---------------------------------------------------------------------------------------------
def main() -> int:
    """Run every check and print the results."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('folder', metavar='DIR',
                        help='the folder that `python3 -m generators --millimetre-collada DIR` '
                        'wrote')
    args = parser.parse_args()
    problems = check_folder_contents(args.folder)
    for problem in problems:
        print(problem)
    checked = 0
    for path in collada_files():
        if not os.path.exists(os.path.join(args.folder, path)):
            continue
        pair = FilePair(path, etree.parse(os.path.join(MESHES, path)),
                        etree.parse(os.path.join(args.folder, path)))
        found = [problem for check in PAIR_CHECKS for problem in check(pair)]
        count = sum(numbers(array).size for array in position_arrays(pair.millimetre))
        print(f'{path}: ' + ('; '.join(found) if found else f'ok, {count} position numbers'))
        problems += [f'{path}: {problem}' for problem in found]
        checked += 1
    print(f'{checked} millimetre files checked, {len(problems)} problems')
    return 1 if problems else 0


if __name__ == '__main__':
    sys.exit(main())
