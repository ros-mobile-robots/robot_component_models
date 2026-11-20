<!-- SPDX-FileCopyrightText: 2026 ROS Mobile Robots
     SPDX-License-Identifier: BSD-3-Clause -->
# Agent instructions for robot_component_models

Read [CONTRIBUTING.md](CONTRIBUTING.md) first. This file adds what an agent needs to work here without guessing. The same rules apply to people and agents.

## Before you push

Find your change in this list. Each line says what to run or check, and links the section with the details.

- **A new or changed part:**
    1. Read [Modelling a part from its sources](CONTRIBUTING.md#modelling-a-part-from-its-sources) before you touch a generator. Every dimension has its source in `provenance/<part>.md`: a public specification, your own measurement with how you took it, or an estimate or a choice that the page calls one.
    2. Change the generator and regenerate the meshes ([After a change to a generator](#after-a-change-to-a-generator)).
    3. The two regeneration checks, the millimetre checks, the mesh check, the unit tests, flake8 and mypy pass ([Check a checkout](#check-a-checkout)).
    4. Load the mesh in a robot's model and look at the robot in RViz and Gazebo. Say in the pull request what you saw.
    5. The part's provenance page, the README's parts table, the version and the changelog are up to date ([Things that are easy to get wrong](#things-that-are-easy-to-get-wrong)).
- **New logic in a generator, the shared code or a script:** the change brings a check of its own, and a fix brings a check that fails without the fix ([Code](#code)).
- **A branch you only want to verify:** run the checks, and never the regeneration ([Check a checkout](#check-a-checkout)).
- **A new part, the shared code, a new check, CI or the development setup:** the documentation pull request is planned with the change ([Documentation](#documentation)).
- **Only the guides or the README:** the version stays. Run every command and example that you add or change, as written.
- **The commits and the pull request:** one topic, signed commits, and no attribution lines for AI tools ([Commits and pull requests](#commits-and-pull-requests)).

## What this is

Display models of off-the-shelf components that robots carry, such as a laser scanner or a single-board computer. Robot description packages, such as [remo_description](https://github.com/ros-mobile-robots/remo_description), use the meshes. Joints, collision shapes and inertias are not here: they belong to each robot's description.

| Path | What's in it |
|---|---|
| `generators/<part>.py` | One CadQuery generator per part: its parameters with their sources at the top, and `build()`, which returns the part's meshes |
| `generators/mesh_export.py`, `generators/manifest.py` | The writers of the STL, COLLADA and glTF files, and of `meshes/manifest.yaml` |
| `meshes/` | Generated files. Change the generator, never the file |
| `provenance/<part>.md` | Each model's sources, its frame, and what is simplified or estimated |
| `scripts/` | The checks that CI runs (`check_meshes.py`, `check_install.py`, `check_millimetre_collada.py`, `check_millimetre_option.py`) and the README's pictures (`render_parts.py`) |

## Commands

On your PC, in the repository's root. The pinned Python packages of the generators go into `.venv`, and the pinned code style tools into `.lint`:

```bash
python3 -m venv .venv && .venv/bin/pip install -r generators/requirements.txt
python3 -m venv .lint && .lint/bin/pip install -r scripts/requirements-lint.txt
```

### Check a checkout

These commands change no tracked file. Run them on your own change, and on someone else's branch that you only want to verify:

```bash
.venv/bin/python -m generators --check           # regeneration check: all files byte-identical
.venv/bin/python -m generators --check           # a second run gives the same line
mm="$(mktemp -d)/meshes_mm"                      # a folder name that doesn't exist yet
.venv/bin/python -m generators --millimetre-collada "$mm"   # COLLADA files written in millimetres: …
.venv/bin/python scripts/check_millimetre_collada.py "$mm"  # … millimetre files checked, 0 problems
.venv/bin/python scripts/check_millimetre_option.py         # … cases, 0 failed
.venv/bin/python scripts/check_meshes.py         # … files checked, 0 problems
.venv/bin/python -m unittest discover -s tests   # OK
.lint/bin/flake8 generators scripts tests        # prints nothing
.lint/bin/mypy generators scripts tests          # Success: no issues found in … source files
```

- **`--check`:** regenerates every mesh into a temporary folder and compares the files and the manifest with `meshes/`. It passes twice in a row, because the files hold no time stamps or paths.
- **The millimetre files:** `--millimetre-collada` writes every COLLADA file once more, in millimetres, into a new folder outside `meshes/` ([README, Millimetre COLLADA files](README.md#millimetre-collada-files-for-close-ups-in-gazebo-classic)). `check_millimetre_collada.py` compares that folder with `meshes/`, and `check_millimetre_option.py` tries the folders that the option has to refuse, each on a copy in a temporary folder. CI's job "Regenerate meshes" runs the three commands after the two regeneration checks.
- **`check_meshes.py`:** its first run downloads the COLLADA 1.4.1 schema from khronos.org into `.cache/collada` and verifies the file by its checksum, so that run needs network access.
- **Where the bytes are the same as CI's:** CI regenerates in one environment, the Ubuntu 24.04 image that `.github/workflows/ci.yml` pins by its digest, on an x64 (amd64) machine, with Python 3.12.3, glibc 2.39 and the graphics libraries that the workflow installs ([README, Regenerating the meshes](README.md#regenerating-the-meshes)). The pinned Python packages alone don't give that environment. On another system, `--check` can report differing files for an unchanged checkout. Run the check in CI's image then: [Run CI's checks locally](CONTRIBUTING.md#run-cis-checks-locally) has the docker command.

### After a change to a generator

The first command below rewrites every file in `meshes/` and the manifest. Run it only after you changed a generator on purpose, never to check a checkout: `--check` compares a regeneration with the files in the working tree, and after a rewrite those are the files you just wrote.

```bash
.venv/bin/python -m generators            # write every mesh and meshes/manifest.yaml
git status --short meshes                 # the files you meant to change, and no others
```

When a part's look changed, render the README's pictures again. These commands rewrite the PNG files in `docs/images/`, which are tracked: one picture per part, the overview `all_parts.png`, which the README shows first, and one frame picture per part in `docs/images/frames/`. CI doesn't compare their pixels, and another graphics driver gives slightly other pixels. Commit the pictures of each part you changed together with the overview, and put back the pictures that differ without a change to their part. A new part gets its pictures from three entries: in `scripts/render_parts.py`, the part in `PARTS` for its own picture and in a row of `OVERVIEW_ROWS` for the overview, and in `scripts/render_frames.py`, a function for the frame picture and its entry in `FIGURES`. CI's unit tests (`tests/test_registrations.py`) fail for a new part without these entries and pictures, so render them before you run the checks:

```bash
.venv/bin/python scripts/render_parts.py          # docs/images/<part>.png and all_parts.png
.venv/bin/python -m scripts.render_frames         # docs/images/frames/<part>.png
git status --short docs/images                    # the changed part's pictures and all_parts.png, and no others
```

Then run the commands of [Check a checkout](#check-a-checkout). Meshes for a pull request have to come from CI's environment, or CI's regeneration check fails: [The generator environment in a container](CONTRIBUTING.md#the-generator-environment-in-a-container) shows how to regenerate in CI's image.

## Code

- **Parameters at the top:** every dimension is a named constant in the generator's parameter block, with its source: the datasheet's figure, the drawing, a measurement, or "choice". No dimension is typed inside a function.
- **`build()` is the list:** it reads as the list of the part's sub-parts, and each sub-part has one function that builds it. A function that builds seven things becomes seven functions.
- **Checks apart from geometry:** a check of the parameters is a `check_…()` function, and one runner calls them. A check that needs a shape runs after that shape exists. The scripts do the same: one function per check and a table that runs them, as `FILE_CHECKS` in `scripts/check_meshes.py`.
- **Types:** what belongs together (a footprint, a path piece, a groove) is a `NamedTuple` with named fields, not a tuple read by index. A closed set of kinds, such as a connector's kind, is an `Enum`, not a string or a float compared for equality.
- **Shared shapes:** a shape or a helper that several generators use with the same meaning belongs in a shared module, not in a copy per generator. Look for a helper in the generators, in CadQuery and in the pinned packages before you write one.
- **Names:** written out, `body_height`, not `BODY_H`. One word for one thing in all generators.
- **Type hints and docstrings:** type hints on every function you add or change. PEP 257 docstrings: a function's first line is a command. A generator's module docstring names the sources its dimensions come from.
- **What the style check covers:** CI's job "Code style" runs flake8 with the ament_flake8 settings and mypy for Python 3.8 on the generators, scripts and tests, with the tools pinned in `scripts/requirements-lint.txt` and the settings in `setup.cfg`. `setup.cfg` sets `disallow_untyped_defs`, so mypy reports every function without type hints.
- **Comments:** they say why, or where a value comes from. A source that a comment cites is listed on the part's provenance page.
- **Checks for new logic:** a new shape, a new step of the export or a new function of a script comes with a check of a property that a reader can see: a dimension, the position of a frame or a count. The expected value comes from the part's source or from the generator's parameter block, never from the generated files. A wrong mesh that is regenerated with its manifest agrees with its manifest. Put the check where CI runs it: as a `check_…()` function that the generator calls, which the job "Regenerate meshes" runs with every regeneration, in `scripts/check_meshes.py`, or as a unit test in `tests/test_<topic>.py`, which the job "Mesh checks" runs with `python -m unittest discover -s tests`. A unit test suits a known input and its expected result, such as `tests/test_gltf_colours.py` with an sRGB colour and its linear value, and a broken input that a check has to report.
- **Fixes:** a fix of a generator or a check comes with a check that fails without it.
- **Test names:** when you add or change a unit test, give it a name that says what the test checks, as a sentence: the subject, the observable result, then the condition, as in `test_collar_rounds_raises_a_value_error_for_a_width_past_the_bulges_round`. Leave out the condition when the subject and the result say enough (`test_srgb_to_linear_keeps_black_and_white`). No numbers that put tests in an order (`test_1_…`), and no name without a result (`test_collar`).

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
- **On ros-mobile-robots.com:** a new part, a change to the generators' shared code, a new check, a CI change and a change to the development setup come with a page or an update on the documentation site, as a pull request in [ros-mobile-robots.github.io](https://github.com/ros-mobile-robots/ros-mobile-robots.github.io) that is planned with the change and linked under `## Related`. The page says what the change does and how to use it, and how the code works. It walks through each check in the order a reader would run it: what the check sets up, what goes in, what the script does, and what is compared with which tolerance, each with its code excerpt, and one picture per check. That repository's `AGENTS.md` has the [writing rules](https://github.com/ros-mobile-robots/ros-mobile-robots.github.io/blob/main/AGENTS.md#writing-rules). The [code walkthrough of the Simulation Test](https://ros-mobile-robots.com/development/simulation-test/#code-walkthrough) is an example of a walk through a test's code.
- **If you can't open that pull request:** say under `## Related` what the page needs. A maintainer then takes the page over before the change is merged: with a documentation pull request, or with an issue in that repository that names the page and has an assignee.

## Things that are easy to get wrong

- **Units:** generators work in millimetres, and the files in `meshes/` are written in metres. Only `--millimetre-collada` writes files in millimetres, and only into a new folder outside `meshes/`.
- **A mesh's frame is a contract:** a change that moves a mesh's origin, turns its axes, changes its units or moves an attachment point gets a new file name. The old file stays.
- **Sources:** read "Modelling a part from its sources" in CONTRIBUTING.md before you touch a generator.
- **Versions:** raise the version in `package.xml` and add a section to `CHANGELOG.rst`. A pull request that only changes the guides or the README keeps the version. Don't create tags.
- **CI's job names:** the default branch requires each job of `.github/workflows/ci.yml` as a check by the job's name, such as "Regenerate meshes" or "ROS noetic, catkin". A change that renames or removes a job, also through the `name` of an entry in the ROS job's matrix, can't be merged into the default branch while the required checks still list the old name. List each old name with its new name, or its removal, in the pull request. Before the change reaches the default branch, a maintainer updates the required checks in the repository's settings and confirms that each new check has passed.

## Commits and pull requests

- **One topic:** one change per pull request and per commit.
- **Branch:** `feat/…`, `fix/…`, `docs/…`, `chore/…`, `refactor/…` or `test/…`, followed by a short topic in kebab-case, such as `feat/rplidar-a1`.
- **Title and description:** squash-merged by a maintainer, who uses the title as the commit subject and the description as its message. Title `Area: what changed`, 65 characters or fewer. Use the pull request template.
- **Issues:** end the description with `Closes #N` when the pull request resolves issue N. For an issue that stays open, name the issue without a closing keyword, as in `Part of #N`.
- **Signed commits:** sign every commit with GPG, `git commit -S`. `git log --show-signature -1` shows the signature of the last commit. A coding agent that can't sign stops and reports that, and doesn't commit unsigned.
- **A robot's model:** if a robot's model has to change with yours, open the matching pull request in the robot's repository and link the two.
- **AI tools:** no attribution lines for AI tools in commits or pull requests (`Co-Authored-By:` for a tool, "Generated with …"). No disclosure of AI use is needed.
