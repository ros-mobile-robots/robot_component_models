<!-- SPDX-FileCopyrightText: 2026 ROS Mobile Robots
     SPDX-License-Identifier: BSD-3-Clause -->
# Contributing to robot_component_models

This package holds display models of off-the-shelf components that robots carry: their meshes, the CadQuery generators that make them, and the source references of every model. Robot assembly, joints, collision shapes and inertias belong in each robot's description package, such as [remo_description](https://github.com/ros-mobile-robots/remo_description) or `diffbot_description` in the [diffbot repository](https://github.com/ros-mobile-robots/diffbot).

Corrections are welcome, especially measurements of the real parts: each part's provenance page lists the measurements that would improve it. Open an [issue](https://github.com/ros-mobile-robots/robot_component_models/issues) or a pull request. Ask questions in the diffbot repository's [Discussions](https://github.com/ros-mobile-robots/diffbot/discussions/categories/q-a), which cover both robots and their packages.

## Modelling a part from its sources

Every model here is the project's own work, under this repository's [license](LICENSE). A model is made like this:

- Model each part from facts about the physical product: the dimensions in datasheets and mechanical drawings, the look in product photos, standards such as the 0.1 in pin pitch, and your own measurements and photos of the part.
- Record every source in the part's page in `provenance/`: the document, its page or figure, and the values taken from it. Link documents and photos where they are published. Say how a measured value was measured, and mark a value that is estimated or chosen.
- Put each dimension into the generator's parameter block, with its source, and build the shapes from these parameters with CadQuery.
- Document each mesh's origin, axes, units and consuming joints in the page's Frame section, with the joint poses from the robots' published descriptions.
- No logos, brand names or printed text on the models. Generic shapes of connectors and chips are fine.

## Adding or changing a part

Adding a part takes six steps. The laser scanner `rplidar_a2`, a part with one mesh, shows each of them:

1. **Identifier:** lowercase snake_case after the product, such as `rplidar_a2` (see the README's [file names](README.md#file-names)). The generator module, the folder in `meshes/`, the provenance page and the pictures share it.
2. **Generator:** `generators/rplidar_a2.py`. The parameters stand at the top, in millimetres, each with its source. In this excerpt, `DS` marks a value of the datasheet:

   ```python
   BASE_DIAMETER = 75.70        # DS
   MOUNT_HOLE_RADIUS = 29.50    # DS: 4 x M3 on R29.50, at 45 degrees to the cable axis
   ```

   `build()` returns the part's meshes as `mesh_export.Model` objects, with named sub-parts and materials. The export writes the files in metres. The scanner's `build()` returns one mesh:

   ```python
   def build() -> List[Model]:
       """Return the scanner's mesh: housing, copper-coloured trim and dark windows."""
       solids = make_parts()
       sub_parts = [SubPart(part, {material: tessellate_faces(solids[material], LINEAR_DEFLECTION,
                                                              ANGULAR_DEFLECTION)})
                    for part, material in SUB_PARTS.items()]
       return [Model('rplidar_a2', TITLE, FRAME, MATERIALS, sub_parts)]
   ```

3. **List of parts:** the identifier in `PARTS` in [`generators/__init__.py`](generators/__init__.py). The manifest follows this order.
4. **Provenance page:** `provenance/rplidar_a2.md` with the sources, the Frame section, what is simplified or estimated, and which measurements would help.
5. **Pictures:** two scripts draw them.
   - [`scripts/render_parts.py`](scripts/render_parts.py) has the part in `PARTS`, with its caption, its mesh files and the camera's azimuth and elevation in degrees, and in `OVERVIEW_ROWS`, with the part's turn about z in the overview picture. The scanner's entry in `PARTS`:

     ```python
     'rplidar_a2': PartView('Laser scanner (RPLIDAR A2 size)',
                            [Placement('rplidar_a2/rplidar_a2.dae')], 25.0, 22.0),
     ```

   - [`scripts/render_frames.py`](scripts/render_frames.py) has a function for the part, here `scanner_figure()`, and its entry in `FIGURES`. The function returns the two views of the frame picture, and the mounting holes and the mounting plane, computed from the generator's parameters.
6. **README:** a row in the parts table with the picture, the triangles and the file sizes.

[`tests/test_registrations.py`](tests/test_registrations.py) fails in CI if a part of `PARTS` lacks its entry in one of the two scripts, its provenance page, one of its pictures or its link in the README.

Then generate and check, with the pinned Python packages in a virtual environment (see the README's [Regenerating the meshes](README.md#regenerating-the-meshes)):

```bash
.venv/bin/python -m generators rplidar_a2        # write the part's meshes while you work on it
.venv/bin/python -m generators                   # when the part is ready: all meshes and the manifest
.venv/bin/python scripts/check_meshes.py         # read the files back, check them against the manifest
.venv/bin/python -m unittest discover -s tests   # the unit tests
.venv/bin/python scripts/render_parts.py         # docs/images/rplidar_a2.png and all_parts.png
.venv/bin/python -m scripts.render_frames        # docs/images/frames/rplidar_a2.png
git status --short                               # the part's files, the manifest and the pictures
```

Look at what the commands wrote:

- `meshes/manifest.yaml` has the part under `parts:`, with each file's checksum, size, triangles and bounding box.
- `docs/images/rplidar_a2.png` shows the part as the README's table will, and `docs/images/frames/rplidar_a2.png` its frame. Compare both with the sources.
- Load the mesh in a robot model and look at it in RViz and Gazebo.

Before you open the pull request, write the files once more in CI's generator environment, so that CI regenerates the same bytes: see [The generator environment in a container](#the-generator-environment-in-a-container).

**License lines:** every new text file starts with the two SPDX lines `SPDX-FileCopyrightText: 2026 ROS Mobile Robots` and `SPDX-License-Identifier: BSD-3-Clause`, in the file's comment syntax. The generators write these lines into each COLLADA file and the manifest, and into a `<file>.license` next to each STL and glTF file.

**File names:** a change that moves a mesh's origin, turns its axes, changes its units or moves an attachment point, such as mounting holes or an axle, gets a new file name. The old file stays, so the robots that use it keep working. Other changes keep the file name.

**Versions:** raise the version in `package.xml` and add a section to [`CHANGELOG.rst`](CHANGELOG.rst) with a line per change. Raise the first number when a file is removed or renamed, the second when files are added or a mesh changes, the third for fixes that leave the meshes as they are. A pull request that only changes the guides or the README keeps the version. After the merge, a maintainer tags the merge commit with the version, for example `1.1.0`.

## Run CI's checks locally

[`.github/workflows/ci.yml`](.github/workflows/ci.yml) runs every job in a container. The same commands give the same result with docker.

### The generator environment in a container

The meshes are reproducible byte for byte in the environment of CI's job "Regenerate meshes". The first command builds that environment once as a local image: CI's Ubuntu image by its digest, the system libraries that the job installs, and the pinned Python packages. The second command checks the meshes. It mounts the repository read-only, and `--check` writes its regenerated files into the container's temporary folder. CI's machine is an x64 one, and `--platform linux/amd64` selects that variant of the image. A computer with an ARM processor needs Docker's emulation of amd64 for these commands:

```bash
docker build --platform linux/amd64 -t robot-component-models-generators -f - generators <<'EOF'
FROM ubuntu:24.04@sha256:fe81560990fae86dc4743202b2cb4d3df98d0bcf949173cecef15ce1127ef852
RUN apt-get update && apt-get install -y --no-install-recommends python3 python3-venv \
    ca-certificates git libgl1 libxrender1 libxext6 libsm6 libice6 libegl1
COPY requirements.txt /tmp/requirements.txt
RUN python3 -m venv /opt/venv && /opt/venv/bin/pip install -r /tmp/requirements.txt
EOF
docker run --rm --platform linux/amd64 -v "$PWD":/src:ro -w /src \
  robot-component-models-generators /opt/venv/bin/python -m generators --check
```

The check ends with this line when the committed files are current:

```text
regeneration check: all files byte-identical
```

To write the meshes and the manifest in that environment, mount the repository writable and run the generators as your own user:

```bash
docker run --rm --platform linux/amd64 --user "$(id -u):$(id -g)" -v "$PWD":/src -w /src \
  robot-component-models-generators /opt/venv/bin/python -m generators
```

The millimetre COLLADA files go into a new folder outside `meshes/`, here in the container's temporary folder, and the second command compares that folder with `meshes/`:

```bash
docker run --rm --platform linux/amd64 -v "$PWD":/src:ro -w /src \
  robot-component-models-generators bash -c '
  /opt/venv/bin/python -m generators --millimetre-collada /tmp/meshes_mm &&
  /opt/venv/bin/python scripts/check_millimetre_collada.py /tmp/meshes_mm'
```

The comparison's last line gives the number of millimetre files that it checked and ends with `0 problems`.

### The ROS package check

The ROS 2 job builds and installs the package and checks the installed files:

```bash
docker run --rm -v "$PWD":/src:ro ros:humble-ros-base bash -c '
  mkdir -p /ws/src && cp -r /src /ws/src/robot_component_models && cd /ws &&
  source /opt/ros/humble/setup.bash && colcon build --merge-install --install-base /tmp/install &&
  python3 /ws/src/robot_component_models/scripts/check_install.py /tmp/install 2'
```

### The code style check

flake8 with ROS 2's ament_flake8 settings and mypy are configured in `setup.cfg`, with the versions in `scripts/requirements-lint.txt`:

```bash
python3 -m venv .lint && .lint/bin/pip install -r scripts/requirements-lint.txt
.lint/bin/flake8 generators scripts tests && .lint/bin/mypy generators scripts tests
```

## Pull requests

- **One change per pull request**, on a branch named `feat/…`, `fix/…`, `docs/…` or `chore/…`.
- **Pull requests are squash-merged.** Maintainers use the PR title as the commit's subject and the description as its message, so write the title as `Area: what changed`, 65 characters or fewer, and start the description with what changes and why.
- Say how you tested it, and link the issue with `Closes #N`.
- If a robot's model has to change with yours, open the matching pull request in the robot's repository and link the two.

## AI-assisted contributions

You're welcome to use AI tools. Whatever produced the change, you're its author and responsible for it:

- Understand every change and be able to explain it in review, in your own words.
- Test it as described above, and report what you actually ran. Don't claim tests that didn't run.
- Check every source you cite. Pull requests with invented sources or values are closed.
- Keep the PR description and your review replies short and specific.

You don't need to say whether you used AI. Please don't add attribution lines for AI tools to commits or pull requests, such as `Co-Authored-By:` for a tool or "Generated with …".

## License

By contributing, you agree that your contributions are licensed under this repository's [license](LICENSE).
