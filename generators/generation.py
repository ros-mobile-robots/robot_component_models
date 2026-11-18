# SPDX-FileCopyrightText: 2026 ROS Mobile Robots
# SPDX-License-Identifier: BSD-3-Clause
"""Build the parts and write their mesh files, one process per part.

The functions that a part's process runs live in this module, not in generators/__main__.py.
A process that is started fresh, as Python's 'spawn' and 'forkserver' start methods do,
imports each function from its module by name. It can import this module, but it doesn't run
a package's __main__.py again, so it would find no function there.
"""
from concurrent.futures import ProcessPoolExecutor
import importlib
import os
import tempfile
from typing import List
from typing import Optional
from typing import Sequence

from generators.manifest import file_facts
from generators.manifest import MeshEntry
from generators.manifest import PartEntry
from generators.mesh_export import require_model_materials
from generators.mesh_export import used_materials
from generators.mesh_export import write_dae
from generators.mesh_export import write_glb
from generators.mesh_export import write_license_file
from generators.mesh_export import write_stl

EXTENSIONS = ('.stl', '.dae', '.glb')
BINARY_EXTENSIONS = ('.stl', '.glb')  # these get a <file>.license next to them


def generate_part(name: str, out: str) -> PartEntry:
    """Write one part's meshes into out/<part>/ and return their manifest entry."""
    module = importlib.import_module(f'generators.{name}')
    os.makedirs(os.path.join(out, name), exist_ok=True)
    meshes = []
    for model in module.build():
        require_model_materials(model)
        relative = f'{name}/{model.name}'
        write_stl(model, os.path.join(out, relative + '.stl'))
        write_dae(model, os.path.join(out, relative + '.dae'), f'generators/{name}.py')
        write_glb(model, os.path.join(out, relative + '.glb'))
        for extension in BINARY_EXTENSIONS:
            write_license_file(os.path.join(out, relative + extension))
        files = [file_facts(out, relative + extension) for extension in EXTENSIONS]
        meshes.append(MeshEntry(model.name, model.frame, [part.name for part in model.sub_parts],
                                used_materials(model), files))
        print(f'{relative}: ' + ', '.join(f'{os.path.basename(facts.path)} {facts.triangles} '
                                          f'triangles {facts.size} bytes' for facts in files),
              flush=True)
    return PartEntry(name, module.TITLE, meshes)


def use_temporary_folder(folder: Optional[str]) -> None:
    """Make this process, and the processes that it starts, put temporary files into the folder.

    tempfile takes the folder as it is, without the test write of its own search. TMPDIR
    hands the folder to processes that are started later. None leaves tempfile's own choice.
    """
    if folder is not None:
        tempfile.tempdir = folder
        os.environ['TMPDIR'] = folder


def generate(names: Sequence[str], out: str, temporary: Optional[str] = None) -> List[PartEntry]:
    """Write the parts' meshes into out/, one process per part, return them in order.

    Each process puts its temporary files into the temporary folder, if one is given. So a
    process that is started fresh, without the parent's state, doesn't search for a folder of
    its own.
    """
    with ProcessPoolExecutor(max_workers=min(len(names), os.cpu_count() or 1),
                             initializer=use_temporary_folder, initargs=(temporary,)) as pool:
        return list(pool.map(generate_part, names, [out] * len(names)))
