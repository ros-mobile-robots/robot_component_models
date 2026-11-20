# SPDX-FileCopyrightText: 2026 ROS Mobile Robots
# SPDX-License-Identifier: BSD-3-Clause
"""Tests of the generators' command line: checking never writes into meshes/.

`python3 -m generators --check` regenerates the meshes and compares them with the files in
meshes/, the reference. The reference must stay as it is, whatever the check finds and wherever
its regenerated files are kept. Each test runs the real command in a sandbox of its own: a copy
of generators/ whose list of parts holds only the camera, the part with the fewest triangles,
and a meshes/ folder that the command wrote for that list. So a test takes seconds, and a
failing test can't change the repository's own files.

Run from the repository's root, with the packages of generators/requirements.txt:

  python3 -m unittest discover -s tests
"""
import hashlib
import os
import shutil
import subprocess
import sys
import tempfile
from typing import Dict
from typing import List
from typing import Optional
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# The sandbox's only part: the one with the fewest triangles, which is the quickest to generate
PART = 'raspberry_pi_camera_v2'
# A reference file that a test changes, and the text it puts there
MARKED_FILE = os.path.join(PART, PART + '.stl.license')
MARKER = 'changed by a test\n'
SPDX_LINE = 'SPDX-License-Identifier: BSD-3-Clause'
PASSED, FAILED, USAGE_ERROR = 0, 1, 2
ENVIRONMENT = dict(os.environ, PYTHONDONTWRITEBYTECODE='1')
# `python3 -m generators` with its part processes started by the method that the first argument
# names: runpy runs the package's __main__.py as `-m` does, after the start method is set
START_METHOD_ENTRY = '\n'.join([
    'import multiprocessing, runpy, sys',
    'multiprocessing.set_start_method(sys.argv.pop(1))',
    "sys.argv[0] = 'generators'",
    "runpy.run_module('generators', run_name='__main__', alter_sys=True)",
])


def run_generators(repository: str, arguments: List[str], environment: Dict[str, str],
                   start_method: Optional[str] = None) -> 'subprocess.CompletedProcess[str]':
    """Run `python3 -m generators` with the arguments in the repository's copy.

    With a start method, such as 'spawn', multiprocessing starts the part processes with it.
    Without one, it uses the platform's default.
    """
    command = ([sys.executable, '-c', START_METHOD_ENTRY, start_method] if start_method
               else [sys.executable, '-m', 'generators'])
    return subprocess.run(command + arguments, cwd=repository, check=False,
                          capture_output=True, text=True, env=environment)


def snapshot(folder: str) -> Dict[str, str]:
    """Return what the folder holds: each entry's path, relative to it, with its content.

    A file's content is its checksum. A symbolic link's content is its target, and the link
    isn't followed.
    """
    entries = {}
    for parent, folders, files in os.walk(folder):
        for name in folders + files:
            path = os.path.join(parent, name)
            relative = os.path.relpath(path, folder)
            if os.path.islink(path):
                entries[relative] = 'link to ' + os.readlink(path)
            elif os.path.isdir(path):
                entries[relative] = 'folder'
            else:
                with open(path, 'rb') as stream:
                    entries[relative] = hashlib.sha256(stream.read()).hexdigest()
    return entries


def folder_times(folder: str) -> Dict[str, int]:
    """Return the time of the last change of the folder and of every folder inside it.

    A folder's time changes when an entry is created or removed in it. So a file that a
    command writes into the folder and deletes again still shows here, where a snapshot of
    the contents after the command would miss it.
    """
    times = {'.': os.stat(folder).st_mtime_ns}
    for parent, folders, _ in os.walk(folder):
        for name in folders:
            path = os.path.join(parent, name)
            times[os.path.relpath(path, folder)] = os.stat(path).st_mtime_ns
    return times


class GeneratorsCommandTest(unittest.TestCase):
    """The base of the tests: a sandbox with the camera as the only part."""

    template: str

    @classmethod
    def setUpClass(cls) -> None:
        """Write the sandbox's template once: generators/ and the meshes/ of its one part."""
        cls.template = tempfile.mkdtemp()
        shutil.copytree(os.path.join(ROOT, 'generators'), os.path.join(cls.template, 'generators'),
                        ignore=shutil.ignore_patterns('__pycache__'))
        with open(os.path.join(cls.template, 'generators', '__init__.py'), 'w',
                  encoding='utf-8') as init:
            init.write(f'PARTS = ({PART!r},)\n')
        os.mkdir(os.path.join(cls.template, 'meshes'))
        written = run_generators(cls.template, [], ENVIRONMENT)
        if written.returncode != PASSED:
            raise RuntimeError(f"the sandbox's meshes weren't written: {written.stderr[-500:]}")

    @classmethod
    def tearDownClass(cls) -> None:
        """Remove the template."""
        shutil.rmtree(cls.template)

    def setUp(self) -> None:
        """Copy the template into a sandbox of this test and mark one reference file."""
        self.sandbox = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.sandbox)
        self.repository = os.path.join(self.sandbox, 'repository')
        shutil.copytree(self.template, self.repository)
        self.meshes = os.path.join(self.repository, 'meshes')
        self.work = os.path.join(self.sandbox, 'work')
        os.mkdir(self.work)
        with open(os.path.join(self.meshes, MARKED_FILE), 'w', encoding='utf-8') as marked:
            marked.write(MARKER)
        self.reference = snapshot(self.meshes)
        self.reference_times = folder_times(self.meshes)

    def run_command(self, *arguments: str) -> 'subprocess.CompletedProcess[str]':
        """Run the command in the sandbox."""
        return run_generators(self.repository, list(arguments), ENVIRONMENT)

    def assert_reference_unchanged(self) -> None:
        """Fail if the command changed meshes/ or wrote anything there, even for a moment.

        The contents must be as before the command, marker included, and no folder of
        meshes/ may have had an entry created or removed.
        """
        self.assertEqual(snapshot(self.meshes), self.reference)
        self.assertEqual(folder_times(self.meshes), self.reference_times)


class KeepRefusalTest(GeneratorsCommandTest):
    """The command refuses a --keep that could write into meshes/, before it generates."""

    def assert_refused(self, result: 'subprocess.CompletedProcess[str]') -> None:
        """Fail unless the command ended with a usage error and left meshes/ and work/ alone."""
        self.assertEqual(result.returncode, USAGE_ERROR, result.stderr)
        self.assert_reference_unchanged()
        self.assertEqual(snapshot(self.work), self.work_before)

    def setUp(self) -> None:
        """Remember the work folder's content after the sandbox is made."""
        super().setUp()
        self.work_before = snapshot(self.work)

    def test_keep_without_check_is_refused_and_writes_nothing(self) -> None:
        """Run --keep DIR without --check: a usage error, no file written anywhere."""
        self.assert_refused(self.run_command('--keep', os.path.join(self.work, 'elsewhere')))

    def test_keep_with_a_part_and_without_check_is_refused_and_writes_nothing(self) -> None:
        """Run PART --keep DIR without --check: a usage error, the part isn't written."""
        self.assert_refused(self.run_command(PART, '--keep',
                                             os.path.join(self.work, 'elsewhere')))

    def test_check_refuses_to_keep_the_files_in_meshes(self) -> None:
        """Run --check --keep meshes: a usage error, the marked file keeps its marker."""
        self.assert_refused(self.run_command('--check', '--keep', 'meshes'))

    def test_check_refuses_to_keep_the_files_in_a_new_folder_inside_meshes(self) -> None:
        """Run --check --keep meshes/regenerated: a usage error, no folder created."""
        self.assert_refused(self.run_command('--check', '--keep',
                                             os.path.join('meshes', 'regenerated')))

    def test_check_refuses_to_keep_the_files_below_a_link_to_meshes(self) -> None:
        """Run --check --keep LINK/regenerated with a link to meshes/: a usage error."""
        link = os.path.join(self.work, 'link')
        os.symlink(self.meshes, link)
        self.work_before = snapshot(self.work)

        self.assert_refused(self.run_command('--check', '--keep',
                                             os.path.join(link, 'regenerated')))

    def test_check_refuses_to_keep_the_files_in_an_existing_folder(self) -> None:
        """Run --check --keep with a folder that holds a link to a reference file: refused."""
        existing = os.path.join(self.work, 'existing')
        os.makedirs(os.path.join(existing, PART))
        os.symlink(os.path.join(self.meshes, MARKED_FILE), os.path.join(existing, MARKED_FILE))
        self.work_before = snapshot(self.work)

        self.assert_refused(self.run_command('--check', '--keep', existing))

    def test_check_refuses_to_keep_the_files_in_a_folder_without_a_name(self) -> None:
        """Run --check --keep with an empty folder name: a usage error."""
        self.assert_refused(self.run_command('--check', '--keep', ''))

    def test_check_refuses_part_names(self) -> None:
        """Run --check PART: a usage error, because the check always compares every part."""
        self.assert_refused(self.run_command('--check', PART))

    def test_check_refuses_a_temporary_folder_inside_meshes(self) -> None:
        """Run --check with TMPDIR set to meshes/: a usage error, no test file written there."""
        result = run_generators(self.repository, ['--check'],
                                dict(ENVIRONMENT, TMPDIR=self.meshes))

        self.assert_refused(result)

    def test_check_refuses_a_temporary_folder_inside_meshes_also_with_keep(self) -> None:
        """Run --check --keep NEW with TMPDIR set to meshes/: refused, nothing written."""
        result = run_generators(self.repository,
                                ['--check', '--keep', os.path.join(self.work, 'kept')],
                                dict(ENVIRONMENT, TMPDIR=self.meshes))

        self.assert_refused(result)

    def test_check_refuses_a_temporary_folder_inside_meshes_named_by_temp(self) -> None:
        """Run --check with TEMP set to a folder in meshes/: a usage error, nothing written."""
        result = run_generators(self.repository, ['--check'],
                                dict(ENVIRONMENT, TEMP=os.path.join(self.meshes, PART)))

        self.assert_refused(result)


class WritePartsTest(GeneratorsCommandTest):
    """Without --check, the command writes the named parts into meshes/."""

    def test_a_part_named_twice_is_written_once(self) -> None:
        """Run PART PART: the part's line is printed once, and the marked file is rewritten."""
        result = self.run_command(PART, PART)

        self.assertEqual(result.returncode, PASSED, result.stderr)
        self.assertEqual(result.stdout.count(f'{PART}/{PART}: '), 1)
        with open(os.path.join(self.meshes, MARKED_FILE), encoding='utf-8') as written:
            self.assertIn(SPDX_LINE, written.read())


class CheckReadOnlyTest(GeneratorsCommandTest):
    """--check reports a changed reference file and leaves it as it is."""

    def test_check_fails_for_a_changed_reference_file_and_leaves_it_changed(self) -> None:
        """Run --check with the marked reference file: it fails and the marker stays."""
        result = self.run_command('--check')

        self.assertEqual(result.returncode, FAILED, result.stderr)
        self.assertIn(f'{MARKED_FILE}: regenerated file differs', result.stdout)
        self.assert_reference_unchanged()

    def test_check_keeps_the_regenerated_files_in_new_folders_and_only_reads_meshes(
            self) -> None:
        """Run --check --keep twice, as CI does, around a repair of the marked file.

        Phase 1: with the marked file, the check fails. The marker stays in meshes/, and the
        first folder holds the regenerated file. Phase 2: the test repairs the marked file
        from the first folder. The check into the second folder then passes, and both folders
        hold the same files as meshes/.
        """
        first = os.path.join('regenerated', 'first')
        second = os.path.join('regenerated', 'second')

        failed = self.run_command('--check', '--keep', first)

        self.assertEqual(failed.returncode, FAILED, failed.stderr)
        self.assertIn(f'{MARKED_FILE}: regenerated file differs', failed.stdout)
        self.assert_reference_unchanged()
        with open(os.path.join(self.repository, first, MARKED_FILE), encoding='utf-8') as kept:
            self.assertIn(SPDX_LINE, kept.read())

        shutil.copyfile(os.path.join(self.repository, first, MARKED_FILE),
                        os.path.join(self.meshes, MARKED_FILE))
        repaired = snapshot(self.meshes)
        passed = self.run_command('--check', '--keep', second)

        self.assertEqual(passed.returncode, PASSED, passed.stdout + passed.stderr)
        self.assertIn('all files byte-identical', passed.stdout)
        self.assertEqual(snapshot(self.meshes), repaired)
        mesh_files = {path: content for path, content in repaired.items()
                      if path != 'manifest.yaml'}
        self.assertEqual(snapshot(os.path.join(self.repository, first)), mesh_files)
        self.assertEqual(snapshot(os.path.join(self.repository, second)), mesh_files)


class TemporaryFolderSpellingTest(GeneratorsCommandTest):
    """--check puts its temporary files where the system finds TMPDIR, links resolved.

    'via' in the repository is a symbolic link to a folder outside it. So 'via/../meshes' is
    the folder 'meshes' next to that folder, outside the repository. Cancelling '..' against
    'via' in the spelling would give the repository's own meshes/ instead. The part processes
    start with the platform's default method here, and with other methods in the subclasses.
    """

    start_method: Optional[str] = None

    def setUp(self) -> None:
        """Make the outside folders and the link, and spell TMPDIR through the link."""
        super().setUp()
        outside = os.path.join(self.work, 'elsewhere')
        os.makedirs(os.path.join(outside, 'child'))
        self.outside_meshes = os.path.join(outside, 'meshes')
        os.mkdir(self.outside_meshes)
        os.symlink(os.path.join(outside, 'child'), os.path.join(self.repository, 'via'))
        self.spelled = os.path.join(self.repository, 'via', os.pardir, 'meshes')
        self.outside_time = os.stat(self.outside_meshes).st_mtime_ns

    def assert_checked_in_the_outside_folder(
            self, result: 'subprocess.CompletedProcess[str]') -> None:
        """Fail unless the check ran, used the outside folder and left meshes/ alone.

        The marked reference file is the check's one difference: no regenerated file shows up
        in meshes/ as a file that wasn't generated.
        """
        self.assertEqual(result.returncode, FAILED, result.stderr)
        self.assertIn(f'{MARKED_FILE}: regenerated file differs', result.stdout)
        self.assertNotIn('not generated', result.stdout)
        self.assert_reference_unchanged()

    def run_check(self, *arguments: str) -> 'subprocess.CompletedProcess[str]':
        """Run --check with the arguments and the spelled TMPDIR, by the class's start method."""
        return run_generators(self.repository, ['--check', *arguments],
                              dict(ENVIRONMENT, TMPDIR=self.spelled), self.start_method)

    def test_check_resolves_a_temporary_folder_spelled_through_a_link(self) -> None:
        """Run --check with the spelled TMPDIR: its temporary folder is made outside."""
        result = self.run_check()

        self.assert_checked_in_the_outside_folder(result)
        self.assertNotEqual(os.stat(self.outside_meshes).st_mtime_ns, self.outside_time)
        self.assertEqual(os.listdir(self.outside_meshes), [])

    def test_check_with_keep_resolves_a_temporary_folder_spelled_through_a_link(self) -> None:
        """Run --check --keep NEW with the spelled TMPDIR: meshes/ is only read."""
        result = self.run_check('--keep', os.path.join(self.work, 'kept'))

        self.assert_checked_in_the_outside_folder(result)


class SpawnedPartProcessesTest(TemporaryFolderSpellingTest):
    """The same checks, with part processes that the 'spawn' method starts.

    A spawned process starts fresh: it has none of the parent's state, such as tempfile's
    folder, and it imports each function that it runs by its module's name. It doesn't run the
    package's __main__.py, so a function defined there can't be found.
    """

    start_method = 'spawn'


class ForkserverPartProcessesTest(TemporaryFolderSpellingTest):
    """The same checks, with part processes that the 'forkserver' method starts.

    A fork server is a fresh process too, and Python 3.14 makes it the default on Linux.
    """

    start_method = 'forkserver'


class MillimetreOptionTest(GeneratorsCommandTest):
    """--millimetre-collada DIR writes the COLLADA files in millimetres into the new folder."""

    def test_millimetre_option_with_spawned_part_processes_writes_the_file(self) -> None:
        """Run --millimetre-collada NEW with the part processes started by 'spawn'.

        A spawned process imports the function that it runs by its module's name. The file
        is written in millimetres, and meshes/ stays as it is.
        """
        destination = os.path.join(self.work, 'millimetre')

        result = run_generators(self.repository, ['--millimetre-collada', destination],
                                ENVIRONMENT, 'spawn')

        self.assertEqual(result.returncode, PASSED, result.stderr)
        with open(os.path.join(destination, PART, PART + '.dae'), encoding='utf-8') as written:
            self.assertIn('<unit name="millimeter" meter="0.001"/>', written.read())
        self.assert_reference_unchanged()


if __name__ == '__main__':
    unittest.main()
