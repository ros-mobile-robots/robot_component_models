<!-- SPDX-FileCopyrightText: 2026 ROS Mobile Robots
     SPDX-License-Identifier: BSD-3-Clause -->
# Contributing to robot_component_models

This package holds display models of off-the-shelf components that robots carry: their meshes, the CadQuery generators that make them, and the sources of every model. Robot assembly, joints, collision shapes and inertias belong in each robot's description package, such as [remo_description](https://github.com/ros-mobile-robots/remo_description) or `diffbot_description` in the [diffbot repository](https://github.com/ros-mobile-robots/diffbot).

Corrections are welcome, especially measurements of the real parts: each part's provenance page lists the measurements that would improve it. Open an [issue](https://github.com/ros-mobile-robots/robot_component_models/issues) or a pull request. Ask questions in the diffbot repository's [Discussions](https://github.com/ros-mobile-robots/diffbot/discussions/categories/q-a), which cover both robots and their packages.

## Independent creation

Every model here is the project's own work, under this repository's [license](LICENSE). To keep it that way:

- Model each part from scratch, from public facts about the physical product: datasheets, mechanical drawings, product photos, standards such as the 0.1 in pin pitch, and your own measurements and photos.
- Record every source in the part's page in `provenance/`: the document, its page or figure, and the values taken from it. Link public documents and photos. Don't copy them into the repository.
- Never import, convert, trace, sample or otherwise derive geometry from an existing third-party 3D model, and don't download vendor or third-party 3D models for the work.
- Document each mesh's origin, axes, units and consuming joints in its Frame section. If an existing model must be inspected so that a mesh fits a robot's frame, limit the inspection to the placement information needed: its bounding box, origin and axis directions, or the positions of attachment features such as mounting holes. Keep the source's identity, the measurements and comparisons in the pull request's review.
- No logos, brand names or printed text on the models. Generic shapes of connectors and chips are fine.

## Adding or changing a part

1. Choose the part's identifier, lowercase snake_case after the product, such as `rplidar_a2` (see the README's file name rules). Write or change the generator `generators/<part>.py`. Parameters go at the top, each with its source. `build()` returns the part's meshes as `mesh_export.Model` objects, in millimetres, with named sub-parts and materials. The export writes the files in metres. Add a new part to `PARTS` in `generators/__init__.py`. `build()` reads as the list of the part's sub-parts, with one function per sub-part, and every dimension is a named constant in the parameter block, not a number inside a function. [AGENTS.md](AGENTS.md#code) has the code rules.
2. Write or update `provenance/<part>.md`: sources, the Frame section, what is simplified or estimated, and which measurements would help.
3. Regenerate and check, with the pinned Python packages (see the README):

   ```bash
   .venv/bin/python -m generators
   .venv/bin/python -m generators --check
   .venv/bin/python scripts/check_meshes.py
   .venv/bin/python scripts/render_parts.py
   ```

4. Load the mesh in a robot model and look at it in RViz and Gazebo.
5. Update the README's parts table: the picture, triangles and file sizes.
6. Update the documentation site: a new part, a change to the generators' shared code and a new check come with a page or an update on ros-mobile-robots.com, as a pull request in [ros-mobile-robots.github.io](https://github.com/ros-mobile-robots/ros-mobile-robots.github.io) that you plan with the change and link under `## Related`. [AGENTS.md](AGENTS.md#documentation) says what the page holds.

**License lines:** every new text file starts with the two SPDX lines `SPDX-FileCopyrightText: 2026 ROS Mobile Robots` and `SPDX-License-Identifier: BSD-3-Clause`, in the file's comment syntax. The generators write these lines into each COLLADA file and the manifest, and into a `<file>.license` next to each STL and glTF file.

**File names:** a change that moves a mesh's origin, turns its axes, changes its units or moves an attachment point, such as mounting holes or an axle, gets a new file name. The old file stays, so the robots that use it keep working. Other changes keep the file name.

**Versions:** raise the version in `package.xml` and add a section to [`CHANGELOG.rst`](CHANGELOG.rst) with a line per change. Raise the first number when a file is removed or renamed, the second when files are added or a mesh changes, the third for fixes that leave the meshes as they are. A pull request that only changes the guides or the README keeps the version. After the merge, a maintainer tags the merge commit with the version, for example `1.1.0`.

## Run CI's checks locally

[`.github/workflows/ci.yml`](.github/workflows/ci.yml) runs every job in a container. The same commands give the same result with docker, for example the ROS 2 package check:

```bash
docker run --rm -v "$PWD":/src:ro ros:humble-ros-base bash -c '
  mkdir -p /ws/src && cp -r /src /ws/src/robot_component_models && cd /ws &&
  source /opt/ros/humble/setup.bash && colcon build --merge-install --install-base /tmp/install &&
  python3 /ws/src/robot_component_models/scripts/check_install.py /tmp/install 2'
```

The code style check: flake8 with ROS 2's ament_flake8 settings and mypy, both configured in `setup.cfg`, with the versions in `scripts/requirements-lint.txt`:

```bash
python3 -m venv .lint && .lint/bin/pip install -r scripts/requirements-lint.txt
.lint/bin/flake8 generators scripts && .lint/bin/mypy generators scripts
```

## Pull requests

- **One change per pull request**, on a branch named `feat/…`, `fix/…`, `docs/…` or `chore/…`.
- **Pull requests are squash-merged.** Maintainers use the PR title as the commit's subject and the description as its message, so write the title as `Area: what changed`, 65 characters or fewer, and start the description with what changes and why.
- Say how you tested it, and link the issue with `Closes #N`.
- If a robot's model has to change with yours, open the matching pull request in the robot's repository and link the two.
- **Fixes:** a fix of a generator or a check comes with a check that fails without it, in `scripts/check_meshes.py` or a script of its own that CI runs.

## AI-assisted contributions

You're welcome to use AI tools. Whatever produced the change, you're its author and responsible for it:

- Understand every change and be able to explain it in review, in your own words.
- Test it as described above, and report what you actually ran. Don't claim tests that didn't run.
- Check every source you cite. Pull requests with invented sources or values are closed.
- Keep the PR description and your review replies short and specific.

You don't need to say whether you used AI. Please don't add attribution lines for AI tools to commits or pull requests, such as `Co-Authored-By:` for a tool or "Generated with …".

## License

By contributing, you agree that your contributions are licensed under this repository's [license](LICENSE).
