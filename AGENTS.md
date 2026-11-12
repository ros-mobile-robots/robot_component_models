<!-- SPDX-FileCopyrightText: 2026 ROS Mobile Robots
     SPDX-License-Identifier: BSD-3-Clause -->
# Agent instructions for robot_component_models

Read [CONTRIBUTING.md](CONTRIBUTING.md) first. This file adds what an agent needs to work here without guessing. The same rules apply to people and agents.

## What this is

Display models of off-the-shelf components that robots carry, such as a laser scanner or a single-board computer. Robot description packages, such as [remo_description](https://github.com/ros-mobile-robots/remo_description), use the meshes. Joints, collision shapes and inertias are not here: they belong to each robot's description.

| Path | What's in it |
|---|---|
| `generators/<part>.py` | One CadQuery generator per part: its parameters with their sources at the top, and `build()`, which returns the part's meshes |
| `generators/mesh_export.py`, `generators/manifest.py` | The writers of the STL, COLLADA and glTF files, and of `meshes/manifest.yaml` |
| `meshes/` | Generated files. Change the generator, never the file |
| `provenance/<part>.md` | Each model's sources, its frame, and what is simplified or estimated |
| `scripts/` | The checks that CI runs (`check_meshes.py`, `check_install.py`) and the README's pictures (`render_parts.py`) |

## Commands

With the pinned Python packages (`python3 -m venv .venv && .venv/bin/pip install -r generators/requirements.txt`):

```bash
.venv/bin/python -m generators            # write every mesh and the manifest
.venv/bin/python -m generators --check    # regenerate and compare with the manifest
.venv/bin/python scripts/check_meshes.py  # check the files against the manifest
.venv/bin/python scripts/render_parts.py  # the README's pictures
python3 -m venv .lint && .lint/bin/pip install -r scripts/requirements-lint.txt
.lint/bin/flake8 generators scripts && .lint/bin/mypy generators scripts
```

## Before you call a change done

- `python -m generators --check` passes twice in a row: the files hold no time stamps or paths, so a second run writes the same bytes.
- `scripts/check_meshes.py`, flake8 and mypy pass, as in CI.
- You loaded the mesh in a robot's model and looked at it in RViz and Gazebo. Say in the pull request what you saw.
- The part's provenance page and the README's parts table are up to date.

## Code

- **Parameters at the top:** every dimension is a named constant in the generator's parameter block, with its source: the datasheet's figure, the drawing, a measurement, or "choice". No dimension is typed inside a function.
- **`build()` is the list:** it reads as the list of the part's sub-parts, and each sub-part has one function that builds it. A function that builds seven things becomes seven functions.
- **Checks apart from geometry:** a check of the parameters is a `check_…()` function, and one runner calls them. A check that needs a shape runs after that shape exists. The scripts do the same: one function per check and a table that runs them, as `FILE_CHECKS` in `scripts/check_meshes.py`.
- **Types:** what belongs together (a footprint, a path piece, a groove) is a `NamedTuple` with named fields, not a tuple read by index. A closed set of kinds, such as a connector's kind, is an `Enum`, not a string or a float compared for equality.
- **Shared shapes:** a shape or a helper that several generators use with the same meaning belongs in a shared module, not in a copy per generator. Look for a helper in the generators, in CadQuery and in the pinned packages before you write one.
- **Names:** written out, `body_height`, not `BODY_H`. One word for one thing in all generators.
- **Type hints and docstrings:** type hints on every function. PEP 257 docstrings: a function's first line is a command. A generator's module docstring names the sources its dimensions come from.
- **Comments:** they say why, or where a value comes from. A source that a comment cites is listed on the part's provenance page.
- **Fixes:** a fix of a generator or a check comes with a check that fails without it.

The first two lines below leave the reader to remember which number is the height. The type under them names it:

```python
SCREW_HEAD = (5.6, 2.1)
head_height = SCREW_HEAD[1]
```

```python
from typing import NamedTuple


class ScrewHead(NamedTuple):
    """The head of a pan head screw (mm)."""

    diameter: float
    height: float


SCREW_HEAD = ScrewHead(diameter=5.6, height=2.1)
head_height = SCREW_HEAD.height
```

## Documentation

- **Here:** a new or changed part updates its provenance page and the README's parts table.
- **On ros-mobile-robots.com:** a new part, a change to the generators' shared code, a new check and a CI change come with a page or an update on the documentation site, as a pull request in [ros-mobile-robots.github.io](https://github.com/ros-mobile-robots/ros-mobile-robots.github.io) that is planned with the change. The page says what the change does and how to use it, and how the code works. It walks through each check in the order a reader would run it: what the check sets up, what goes in, what the script does, and what is compared with which tolerance, each with its code excerpt, and one picture per check. The writing rules are in that repository's `AGENTS.md`.

## Things that are easy to get wrong

- **Units:** generators work in millimetres, and the files are written in metres.
- **A mesh's frame is a contract:** a change that moves a mesh's origin, turns its axes, changes its units or moves an attachment point gets a new file name. The old file stays.
- **Independent creation:** read that section of CONTRIBUTING.md before you touch a generator.
- **Versions:** raise the version in `package.xml` and add a section to `CHANGELOG.rst`. A pull request that only changes the guides or the README keeps the version. Don't create tags.

## Commits and pull requests

- One change per pull request and per commit, on a branch named `feat/…`, `fix/…`, `docs/…` or `chore/…`.
- Squash-merged by a maintainer, who uses the title as the commit subject and the description as its message. Title `Area: what changed`, 65 characters or fewer. Use the pull request template.
- If a robot's model has to change with yours, open the matching pull request in the robot's repository and link the two.
- No attribution lines for AI tools in commits or pull requests (`Co-Authored-By:` for a tool, "Generated with …"). No disclosure of AI use is needed.
