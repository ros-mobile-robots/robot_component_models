# SPDX-FileCopyrightText: 2026 ROS Mobile Robots
# SPDX-License-Identifier: BSD-3-Clause
"""Tests that every part of the generators is registered wherever a part has an entry.

A part is listed in generators/__init__.py. Its pictures, its frame picture, its provenance
page and its row in the README each need an entry of their own. A part that misses one would
be generated without it, and no other check would notice. Run from the repository's root,
with the packages of generators/requirements.txt:

  python3 -m unittest discover -s tests
"""
import os
import unittest

from generators import PARTS
from scripts import render_frames
from scripts import render_parts

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class RegistrationsTest(unittest.TestCase):
    """Each list of parts outside the generators names exactly the generators' parts."""

    def test_render_parts_has_a_view_of_every_part(self) -> None:
        """Compare render_parts.PARTS with the generators' parts: the same names."""
        self.assertEqual(sorted(render_parts.PARTS), sorted(PARTS))

    def test_overview_picture_places_every_part_once(self) -> None:
        """Compare the overview's rows with the generators' parts: each name once."""
        placed = [name for row in render_parts.OVERVIEW_ROWS for name, _ in row]

        self.assertEqual(sorted(placed), sorted(PARTS))

    def test_render_frames_has_a_figure_of_every_part(self) -> None:
        """Compare render_frames.FIGURES with the generators' parts: the same names."""
        self.assertEqual(sorted(render_frames.FIGURES), sorted(PARTS))

    def test_every_part_has_its_provenance_page_and_pictures(self) -> None:
        """Look for each part's page, picture and frame picture: none is missing."""
        expected = [os.path.join(folder, part + extension) for part in PARTS
                    for folder, extension in (('provenance', '.md'),
                                              (os.path.join('docs', 'images'), '.png'),
                                              (os.path.join('docs', 'images', 'frames'), '.png'))]

        missing = [path for path in expected if not os.path.exists(os.path.join(ROOT, path))]

        self.assertEqual(missing, [])

    def test_readme_links_every_part_to_its_provenance_page(self) -> None:
        """Look for each part's provenance link in the README's table: none is missing."""
        with open(os.path.join(ROOT, 'README.md'), encoding='utf-8') as readme:
            text = readme.read()

        missing = [part for part in PARTS if f'](provenance/{part}.md)' not in text]

        self.assertEqual(missing, [])


if __name__ == '__main__':
    unittest.main()
