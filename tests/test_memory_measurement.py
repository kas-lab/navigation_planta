"""Checks for the sampled process-tree RSS metric (not exact peak accounting)."""

import subprocess
import sys
import time

import psutil
import pytest

from utils import run_subprocess_with_memory


def test_concurrent_children_are_summed_but_sequential_peaks_are_not():
    child_code = 'import time; data = bytearray(48 * 1024 * 1024); time.sleep(0.4)'
    base = f'import subprocess, sys; command = [sys.executable, "-c", {child_code!r}]; '
    sequential = base + 'subprocess.run(command, check=True); subprocess.run(command, check=True)'
    concurrent = base + 'children = [subprocess.Popen(command) for _ in range(2)]; [p.wait() for p in children]'

    sequential_peak = run_subprocess_with_memory([sys.executable, '-c', sequential])
    concurrent_peak = run_subprocess_with_memory([sys.executable, '-c', concurrent])

    # Broad bounds: allocator/interpreter overhead differs across environments.
    assert sequential_peak >= 48
    assert concurrent_peak >= 96
    assert concurrent_peak > sequential_peak + 32


def test_experiment_runner_memory_is_excluded():
    runner_data = bytearray(128 * 1024 * 1024)
    peak = run_subprocess_with_memory(
        [sys.executable, '-c', 'import time; time.sleep(0.2)'])
    assert len(runner_data) == 128 * 1024 * 1024
    assert 0 < peak < 64


def test_tracks_discovered_children_after_root_exit(monkeypatch):
    tick = 0

    class FakeProcess:
        def __init__(self, rss, end):
            self.rss = rss
            self.end = end
            self.descendants = []

        def is_running(self):
            return tick < self.end

        def children(self, recursive):
            assert recursive
            return self.descendants

        def status(self):
            return psutil.STATUS_RUNNING

        def memory_info(self):
            from types import SimpleNamespace
            return SimpleNamespace(rss=self.rss * 1024 * 1024)

    root = FakeProcess(10, 1)
    child = FakeProcess(20, 2)
    grandchild = FakeProcess(50, 2)
    root.descendants = [child]

    class FakePopen:
        pid = 123
        returncode = None

        def __enter__(self):
            return self

        def __exit__(self, *args):
            return False

        def poll(self):
            self.returncode = 0 if tick >= 1 else None
            return self.returncode

    def advance(interval):
        nonlocal tick
        assert interval == 0.05
        tick += 1
        # A known child spawns a descendant after the original root exits.
        child.descendants = [grandchild]

    monkeypatch.setattr(subprocess, 'Popen', lambda *a, **kw: FakePopen())
    monkeypatch.setattr(psutil, 'Process', lambda pid: root)
    monkeypatch.setattr(time, 'sleep', advance)

    assert run_subprocess_with_memory(['unused']) == 70
    assert tick == 2


@pytest.mark.parametrize('interval', [0, -1, float('nan'), float('inf')])
def test_rejects_invalid_sampling_interval(interval):
    with pytest.raises(ValueError, match='sample_interval'):
        run_subprocess_with_memory(['unused'], sample_interval=interval)


def test_short_lived_command_and_failure_reporting(tmp_path):
    assert run_subprocess_with_memory([sys.executable, '-c', 'pass']) >= 0
    log = tmp_path / 'command.log'
    with pytest.raises(subprocess.CalledProcessError) as error:
        run_subprocess_with_memory(
            [sys.executable, '-c',
             'import sys; print("diagnostic", file=sys.stderr); raise SystemExit(7)'],
            log_file=log)
    assert error.value.returncode == 7
    assert 'diagnostic' in error.value.stderr
    assert 'diagnostic' in log.read_text()


@pytest.mark.parametrize('code', [11, 12])
def test_expected_unsolved_exit_can_be_accepted(code):
    assert run_subprocess_with_memory(
        [sys.executable, '-c', f'raise SystemExit({code})'],
        accepted_returncodes=(0, 11, 12)) >= 0


def test_missing_command_raises():
    with pytest.raises(FileNotFoundError):
        run_subprocess_with_memory(['/nonexistent/planta-test-command'])
