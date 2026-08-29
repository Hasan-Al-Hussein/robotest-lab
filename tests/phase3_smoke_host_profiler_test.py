# Copyright 2026 Hasan Ahmed
# SPDX-License-Identifier: Apache-2.0
# ruff: noqa: I001

"""Deterministic non-live tests for the Phase 3 smoke host profiler."""

from __future__ import annotations

import argparse
import copy
import csv
import errno
import importlib.util
import io
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest

TEST_DIR = Path(__file__).parent


def _load():
    spec = importlib.util.spec_from_file_location(
        'phase3_smoke_host_profiler', TEST_DIR / 'phase3_smoke_host_profiler.py'
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


profiler = _load()

GIT_SHA = '1' * 40
SOURCE_SHA = '2' * 64


def _canonical(path: Path, document: dict) -> str:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = profiler.canonical_json_bytes(document)
    path.write_bytes(payload)
    digest = __import__('hashlib').sha256(payload).hexdigest()
    Path(f'{path}.sha256').write_text(f'{digest}  {path.name}\n', encoding='ascii')
    return digest


def _canonical_unsigned(path: Path, document: dict) -> str:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = profiler.canonical_json_bytes(document)
    path.write_bytes(payload)
    return __import__('hashlib').sha256(payload).hexdigest()


def _fixture_config(tmp_path: Path, candidate_id: str = 'phase3-test-001'):
    workspace = tmp_path / 'workspace'
    metrics_package = workspace / 'src/robotest_metrics/robotest_metrics'
    metrics_schema = workspace / 'src/robotest_metrics/schema'
    metrics_package.mkdir(parents=True)
    metrics_schema.mkdir(parents=True)
    shutil.copyfile(
        TEST_DIR.parent / 'src/robotest_metrics/robotest_metrics/constants.py',
        metrics_package / 'constants.py',
    )
    shutil.copyfile(
        TEST_DIR.parent / 'src/robotest_metrics/schema/run-result.schema.json',
        metrics_schema / 'run-result.schema.json',
    )
    candidate = workspace / 'artifacts/evidence/phase3-benchmarks' / candidate_id
    candidate.mkdir(parents=True)
    plugin = workspace / 'build/robotest_sim/librobotest_contact_aggregator_system.so'
    plugin.parent.mkdir(parents=True)
    plugin.write_bytes(b'bound-contact-plugin')
    plugin_hash = __import__('hashlib').sha256(plugin.read_bytes()).hexdigest()
    binding = {
        'git': {'dirty': False, 'sha': GIT_SHA, 'status_porcelain': ''},
        'contact_aggregator_binary': {
            'build_embedded_source_inventory_match': True,
            'build_install_build_id_match': True,
            'build_install_embedded_source_inventory_match': True,
            'build_install_sha256_match': True,
            'installed_embedded_source_inventory_match': True,
            'installed_regular_file': True,
            'installed_path': plugin.relative_to(workspace).as_posix(),
            'installed_sha256': plugin_hash,
            'installed_elf_build_id': 'abc123',
            'installed_declared_path': (
                'install/robotest_sim/lib/robotest_sim/librobotest_contact_aggregator_system.so'
            ),
            'source_inventory_sha256': SOURCE_SHA,
        },
    }
    plan = {
        'candidate_id': candidate_id,
        'smoke': {
            'ros_domain_id': 116,
            'gz_partition': f'robotest_p3_{candidate_id}-smoke_00',
            'run_id': f'{candidate_id}-smoke-s1-r0',
        },
        'trials': [
            {
                'candidate_id': candidate_id,
                'gz_partition': f'robotest_p3_{candidate_id}_00',
                'repetition_index': 0,
                'ros_domain_id': 100,
                'run_id': f'{candidate_id}-s1-r0-i00',
                'scenario_id': 1,
                'scenario_name': 'baseline_navigation',
                'scenario_path': 'config/scenarios/phase3/01-baseline-navigation.yaml',
                'scenario_sha256': '3' * 64,
                'suite_index': 0,
            }
        ],
    }
    binding_path = candidate / 'build-binding.json'
    plan_path = candidate / 'suite-plan.json'
    binding_hash = _canonical(binding_path, binding)
    plan_hash = _canonical(plan_path, plan)
    prepared = {
        'build_binding_sha256': binding_hash,
        'candidate_id': candidate_id,
        'git_sha': GIT_SHA,
        'producer': profiler.BENCHMARK_PRODUCER,
        'suite_plan_sha256': plan_hash,
    }
    _canonical(candidate / 'prepared.json', prepared)
    producer = workspace / 'tests/phase3_smoke_host_profiler.py'
    producer.parent.mkdir(parents=True)
    producer.write_bytes(b'profiler-producer-fixture')
    renderer = workspace / 'home/.gz/rendering/ogre2.log'
    renderer.parent.mkdir(parents=True)
    output = (
        workspace
        / 'artifacts/evidence/phase3/performance-profiles'
        / f'{candidate_id}-smoke-profile.json'
    )
    config = profiler.ProfileConfig(
        workspace=workspace,
        candidate_root=candidate,
        candidate_id=candidate_id,
        build_binding=binding_path,
        ros_domain_id=116,
        gz_partition=f'robotest_p3_{candidate_id}-smoke_00',
        startup_timeout_s=1.0,
        sample_period_s=0.5,
        max_duration_s=5.0,
        renderer_log=renderer,
        output_path=output,
    )
    return config, plugin, producer


def _static_identity(config, producer):
    return profiler.collect_static_identity(
        config,
        git_reader=lambda _workspace: {'sha': GIT_SHA, 'status_porcelain': ''},
        environment={'ROBOTEST_CONTACT_PROFILE': '1'},
        producer_path=producer,
    )


def _config_arguments(config) -> argparse.Namespace:
    return argparse.Namespace(
        workspace=config.workspace,
        candidate_root=config.candidate_root,
        candidate_id=config.candidate_id,
        build_binding=config.build_binding,
        ros_domain_id=config.ros_domain_id,
        gz_partition=config.gz_partition,
        startup_timeout_s=profiler.CANONICAL_STARTUP_TIMEOUT_S,
        sample_period_s=profiler.CANONICAL_SAMPLE_PERIOD_S,
        max_duration_s=profiler.CANONICAL_MAX_DURATION_S,
    )


def _stat_text(
    pid: int,
    *,
    comm: str = 'gz sim server',
    state: str = 'S',
    start: int = 100,
    utime: int = 10,
    stime: int = 5,
    ppid: int = 50,
    group: int = 50,
    session: int = 50,
    processor: int = 2,
) -> str:
    fields = ['0'] * 50
    fields[0] = state
    fields[1] = str(ppid)
    fields[2] = str(group)
    fields[3] = str(session)
    fields[11] = str(utime)
    fields[12] = str(stime)
    fields[17] = '1'
    fields[19] = str(start)
    fields[36] = str(processor)
    return f'{pid} ({comm}) {" ".join(fields)}\n'


def _host_proc(proc_root: Path) -> None:
    (proc_root / 'pressure').mkdir(parents=True, exist_ok=True)
    (proc_root / 'uptime').write_text('0.50 0.25\n', encoding='ascii')
    (proc_root / 'loadavg').write_text('1.00 0.50 0.25 2/100 999\n', encoding='ascii')
    (proc_root / 'stat').write_text(
        'ctxt 100\nprocesses 20\nprocs_running 2\nprocs_blocked 0\n', encoding='ascii'
    )
    (proc_root / 'vmstat').write_text(
        '\n'.join(f'{key} {index}' for index, key in enumerate(profiler.VMSTAT_KEYS, 1)) + '\n',
        encoding='ascii',
    )
    some = 'some avg10=0.10 avg60=0.20 avg300=0.30 total=10\n'
    full = 'full avg10=0.00 avg60=0.00 avg300=0.00 total=0\n'
    (proc_root / 'pressure/cpu').write_text(some, encoding='ascii')
    (proc_root / 'pressure/io').write_text(some + full, encoding='ascii')
    (proc_root / 'pressure/memory').write_text(some + full, encoding='ascii')
    owner = proc_root / '40'
    owner.mkdir()
    (owner / 'stat').write_text(
        _stat_text(40, comm='python3', start=40, ppid=1, group=30, session=30),
        encoding='ascii',
    )
    workspace = proc_root.parent / 'workspace'
    runner = workspace / 'tests/phase3_benchmark_runner.py'
    binding = workspace / 'artifacts/evidence/phase3-benchmarks/phase3-test-001/build-binding.json'
    (owner / 'cmdline').write_bytes(
        b'/usr/bin/python3\0'
        + str(runner).encode()
        + b'\0--workspace\0'
        + str(workspace).encode()
        + b'\0--mode\0smoke\0--candidate-id\0phase3-test-001\0'
        + b'--domain-base\0'
        + b'100\0--output-root\0artifacts/evidence/phase3-benchmarks\0'
        + b'--build-binding\0'
        + str(binding).encode()
        + b'\0'
    )
    owner_executable = proc_root.parent / 'smoke-runner-python'
    owner_executable.write_bytes(b'python')
    (owner / 'exe').symlink_to(owner_executable)
    session_leader = proc_root / '50'
    session_leader.mkdir()
    (session_leader / 'stat').write_text(
        _stat_text(50, start=50, ppid=40, group=50, session=50), encoding='ascii'
    )


def _set_smoke_runner_option(proc_root: Path, name: str, value: Path | str) -> None:
    path = proc_root / '40/cmdline'
    command = [item for item in path.read_bytes().split(b'\0') if item]
    encoded_name = name.encode()
    index = command.index(encoded_name)
    command[index + 1] = str(value).encode()
    path.write_bytes(b'\0'.join(command) + b'\0')


def _make_process(
    proc_root: Path,
    pid: int,
    plugin: Path,
    *,
    domain: int = 116,
    partition: str = 'robotest_p3_phase3-test-001-smoke_00',
    profile_value: str | None = '1',
    start: int = 100,
    maps_plugin: bool = True,
    utime: int = 10,
    ppid: int = 50,
    group: int = 50,
    session: int = 50,
) -> Path:
    root = proc_root / str(pid)
    (root / 'task' / str(pid)).mkdir(parents=True)
    environment = f'ROS_DOMAIN_ID={domain}\0GZ_PARTITION={partition}\0'
    if profile_value is not None:
        environment += f'ROBOTEST_CONTACT_PROFILE={profile_value}\0'
    (root / 'environ').write_bytes(environment.encode())
    (root / 'cmdline').write_bytes(b'/usr/bin/ruby3.2\0gz\0sim\0')
    process_stat = _stat_text(
        pid,
        start=start,
        utime=utime,
        ppid=ppid,
        group=group,
        session=session,
    )
    (root / 'stat').write_text(process_stat, encoding='ascii')
    (root / 'task' / str(pid) / 'stat').write_text(process_stat, encoding='ascii')
    executable = proc_root.parent / f'executable-{pid}'
    executable.write_bytes(f'executable-{pid}'.encode())
    (root / 'exe').symlink_to(executable)
    if maps_plugin:
        status = plugin.stat()
        device = f'{os.major(status.st_dev):x}:{os.minor(status.st_dev):x}'
        inode = status.st_ino
        mapped_path = str(plugin)
    else:
        device = '0:0'
        inode = 0
        mapped_path = '[heap]'
    (root / 'maps').write_text(
        f'1000-2000 r--p 00000000 {device} {inode} {mapped_path}\n'
        f'2000-3000 r-xp 00001000 {device} {inode} {mapped_path}\n',
        encoding='utf-8',
    )
    return root


def _update_cpu(root: Path, pid: int, ticks: int, start: int = 100, group: int = 50) -> None:
    value = _stat_text(pid, start=start, utime=ticks, group=group)
    (root / 'stat').write_text(value, encoding='ascii')
    (root / 'task' / str(pid) / 'stat').write_text(value, encoding='ascii')


def _replace_process_stat(
    root: Path,
    pid: int,
    *,
    start: int,
    ppid: int,
    group: int,
    session: int,
    state: str = 'S',
) -> None:
    value = _stat_text(
        pid,
        state=state,
        start=start,
        ppid=ppid,
        group=group,
        session=session,
    )
    (root / 'stat').write_text(value, encoding='ascii')
    (root / 'task' / str(pid) / 'stat').write_text(value, encoding='ascii')


def _contact_record(step: int = 0, linux_tids: tuple[int, ...] = (101,)) -> dict:
    contributions = []
    for offset, linux_tid in enumerate(linux_tids):
        buckets = {
            'cached_event_state_check_ns': 20 + step + offset,
            'contact_policy_protobuf_ns': 40 + step + offset,
            'exhaustive_event_rescan_ns': 30 + step + offset,
            'locked_binding_validation_ns': 10 + step + offset,
            'publish_ns': 50 + step + offset,
        }
        contributions.append(
            {
                **buckets,
                'linux_tid': linux_tid,
                'measured_total_ns': sum(buckets.values()),
                'observation_count': 2 + step + offset,
                'publish_count': 1 + step + offset,
                'rescan_count': 1 + step + offset,
            }
        )
    totals = {
        field: sum(contribution[field] for contribution in contributions)
        for field in profiler.CONTACT_CONTRIBUTION_MONOTONIC_KEYS
    }
    return {
        **{field: totals[field] for field in profiler.CONTACT_BUCKET_KEYS},
        'clock_id': 'CLOCK_THREAD_CPUTIME_ID',
        'measured_total_ns': totals['measured_total_ns'],
        'observation_count': totals['observation_count'],
        'profile_epoch_start_sim_stamp_ns': 1000,
        'publish_count': totals['publish_count'],
        'rescan_count': totals['rescan_count'],
        'saturated': False,
        'schema_version': 2,
        'sim_stamp_ns': 5_000_001_000 + step * 5_000_000_000,
        'status': 'PASS',
        'thread_contributions': contributions,
    }


def _full_stack_files(config, records=()) -> None:
    process_dir = config.candidate_root / 'smoke/processes'
    process_dir.mkdir(parents=True, exist_ok=True)
    lines = []
    for record in records:
        payload = profiler.canonical_json_bytes(record).rstrip(b'\n')
        lines.append(b'[gazebo-1] ' + profiler.CONTACT_PROFILE_PREFIX + payload + b'\n')
    stdout = b''.join(lines)
    stderr = b''
    (process_dir / 'full_stack.stdout.log').write_bytes(stdout)
    (process_dir / 'full_stack.stderr.log').write_bytes(stderr)

    def stream(size):
        return {
            'error': None,
            'maximum_bytes': profiler.LOG_MAX_BYTES,
            'observed_bytes': size,
            'overflow': False,
            'retained_bytes': size,
        }

    metadata = {
        'command': ['ros2', 'launch', 'robotest_bringup', 'robotest.launch.py'],
        'cwd': str(config.workspace),
        'finished_steady_ns': 1_750_000_000,
        'group_confirmed_empty': True,
        'pgid': 50,
        'pid': 50,
        'returncode': -15,
        'role': 'full_stack',
        'started_steady_ns': 1,
        'stderr': stream(len(stderr)),
        'stdout': stream(len(stdout)),
        'timed_out': False,
        'wall_timeout_s': 360.0,
        'wrapped_command': [
            'timeout',
            '--signal=TERM',
            '--kill-after=10s',
            '360.000s',
            'ros2',
            'launch',
            'robotest_bringup',
            'robotest.launch.py',
        ],
    }
    (process_dir / 'full_stack.process.json').write_bytes(profiler.canonical_json_bytes(metadata))


def test_utc_and_proc_stat_support_comm_with_spaces_and_parenthesis() -> None:
    parsed = profiler.parse_proc_stat(_stat_text(44, comm='gz (server) worker'))

    assert parsed.pid == 44
    assert parsed.comm == 'gz (server) worker'
    assert parsed.cpu_ticks == 15
    assert parsed.start_ticks == 100
    assert profiler._utc(0) == '1970-01-01T00:00:00.000000Z'


def test_exact_environment_rejects_duplicate_or_near_match() -> None:
    exact = b'ROS_DOMAIN_ID=116\0GZ_PARTITION=robotest\0'
    duplicate = exact + b'ROS_DOMAIN_ID=116\0'

    assert profiler.has_exact_environment(exact, 116, 'robotest')
    assert not profiler.has_exact_environment(exact, 117, 'robotest')
    assert not profiler.has_exact_environment(duplicate, 116, 'robotest')


def test_regular_file_growth_is_read_through_a_hard_byte_cap(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    path = tmp_path / 'growing.log'
    path.write_bytes(b'a')
    real_open = Path.open
    requested_sizes = []

    class GrowingReader:
        def __init__(self):
            self.handle = real_open(path, 'rb')

        def __enter__(self):
            return self

        def __exit__(self, *_arguments):
            self.handle.close()

        def fileno(self):
            return self.handle.fileno()

        def read(self, size):
            requested_sizes.append(size)
            with real_open(path, 'ab') as target:
                target.write(b'b' * 32)
            return self.handle.read(size)

    def controlled_open(self, mode='r', *arguments, **keywords):
        if self == path and mode == 'rb':
            return GrowingReader()
        return real_open(self, mode, *arguments, **keywords)

    monkeypatch.setattr(Path, 'open', controlled_open)

    with pytest.raises(profiler.ProfileError, match='grew beyond'):
        profiler._bounded_regular_read(path, 8, 'growing fixture')

    assert requested_sizes == [9]


def test_host_parsers_capture_run_queue_vmstat_and_psi(tmp_path: Path) -> None:
    proc_root = tmp_path / 'proc'
    _host_proc(proc_root)

    sample = profiler.read_host_sample(proc_root)

    assert sample['loadavg']['runnable_entities'] == 2
    assert sample['proc_stat']['procs_running'] == 2
    assert sample['vmstat']['pgmajfault'] > 0
    assert sample['pressure']['cpu']['some']['total'] == 10


def test_host_parser_accepts_modern_cpu_full_pressure_row(tmp_path: Path) -> None:
    proc_root = tmp_path / 'proc'
    _host_proc(proc_root)
    full = 'full avg10=0.00 avg60=0.00 avg300=0.00 total=0\n'
    with (proc_root / 'pressure/cpu').open('a', encoding='ascii') as stream:
        stream.write(full)

    sample = profiler.read_host_sample(proc_root)

    assert set(sample['pressure']['cpu']) == {'full', 'some'}
    assert sample['pressure']['cpu']['full']['total'] == 0


@pytest.mark.parametrize('rows', ('full_only', 'unknown'))
def test_host_sample_validator_rejects_invalid_cpu_pressure_rows(
    tmp_path: Path,
    rows: str,
) -> None:
    proc_root = tmp_path / 'proc'
    _host_proc(proc_root)
    sample = profiler.read_host_sample(proc_root)
    row = copy.deepcopy(sample['pressure']['cpu']['some'])
    if rows == 'full_only':
        sample['pressure']['cpu'] = {'full': row}
    else:
        sample['pressure']['cpu']['unexpected'] = row

    with pytest.raises(profiler.ProfileError, match='cpu PSI rows differ'):
        profiler._validate_host_sample(sample, 'sample 0 host')


def test_command_identity_preserves_trailing_nul_padding(tmp_path: Path) -> None:
    proc_root = tmp_path / 'proc'
    process_root = proc_root / '101'
    process_root.mkdir(parents=True)
    payload = b'/usr/bin/ruby3.2\0gz sim server\0' + b'\0' * 3
    (process_root / 'cmdline').write_bytes(payload)
    executable = tmp_path / 'ruby3.2'
    executable.write_bytes(b'ruby')
    (process_root / 'exe').symlink_to(executable)

    identity = profiler._command_identity(proc_root, 101)

    reconstructed = b'\0'.join(item.encode() for item in identity['cmdline']) + b'\0'
    assert reconstructed == payload
    assert identity['cmdline'][-1] == ''
    assert identity['cmdline_size_bytes'] == len(payload)
    assert identity['cmdline_sha256'] == __import__('hashlib').sha256(payload).hexdigest()


def test_command_identity_rejects_nonterminated_or_non_utf8_payload(
    tmp_path: Path,
) -> None:
    proc_root = tmp_path / 'proc'
    process_root = proc_root / '101'
    process_root.mkdir(parents=True)
    cmdline = process_root / 'cmdline'
    cmdline.write_bytes(b'/usr/bin/ruby3.2')
    with pytest.raises(profiler.ProfileError, match='not NUL-terminated'):
        profiler._command_identity(proc_root, 101)

    cmdline.write_bytes(b'/usr/bin/ruby3.2\0\xff\0')
    with pytest.raises(profiler.ProfileError, match='not UTF-8'):
        profiler._command_identity(proc_root, 101)


def test_static_identity_binds_clean_candidate_and_producer(tmp_path: Path) -> None:
    config, plugin, producer = _fixture_config(tmp_path)

    identity = _static_identity(config, producer)

    assert identity['git']['sha'] == GIT_SHA
    assert (
        identity['plugin']['sha256']
        == __import__('hashlib').sha256(plugin.read_bytes()).hexdigest()
    )
    assert identity['profiler_producer']['path'] == str(producer)
    assert identity['smoke']['contact_profile_enabled'] is True

    with pytest.raises(profiler.ProfileError, match='ROBOTEST_CONTACT_PROFILE'):
        profiler.collect_static_identity(
            config,
            git_reader=lambda _workspace: {'sha': GIT_SHA, 'status_porcelain': ''},
            environment={},
            producer_path=producer,
        )
    with pytest.raises(profiler.ProfileError, match='current Git state'):
        profiler.collect_static_identity(
            config,
            git_reader=lambda _workspace: {'sha': GIT_SHA, 'status_porcelain': ' M x\n'},
            environment={'ROBOTEST_CONTACT_PROFILE': '1'},
            producer_path=producer,
        )

    plan_path = config.candidate_root / 'suite-plan.json'
    plan = __import__('json').loads(plan_path.read_text(encoding='utf-8'))
    plan['smoke']['run_id'] = 'wrong-smoke-run'
    plan_hash = _canonical(plan_path, plan)
    prepared_path = config.candidate_root / 'prepared.json'
    prepared = __import__('json').loads(prepared_path.read_text(encoding='utf-8'))
    prepared['suite_plan_sha256'] = plan_hash
    _canonical(prepared_path, prepared)
    with pytest.raises(profiler.ProfileError, match='run ID is not frozen'):
        profiler.collect_static_identity(
            config,
            git_reader=lambda _workspace: {'sha': GIT_SHA, 'status_porcelain': ''},
            environment={'ROBOTEST_CONTACT_PROFILE': '1'},
            producer_path=producer,
        )


def test_discovery_requires_exact_environment_and_one_plugin_host(tmp_path: Path) -> None:
    config, plugin, producer = _fixture_config(tmp_path)
    identity = _static_identity(config, producer)
    proc_root = tmp_path / 'proc'
    _host_proc(proc_root)
    _make_process(proc_root, 101, plugin)
    _make_process(proc_root, 102, plugin, domain=115)

    anchor, matching_count = profiler.discover_anchor(
        proc_root, config.ros_domain_id, config.gz_partition, identity['plugin']
    )

    assert matching_count == 1
    assert anchor['pid'] == 101
    assert anchor['executable']['sha256']
    assert anchor['mapping_count'] == 2

    _make_process(proc_root, 103, plugin)
    with pytest.raises(profiler.ProfileError, match='multiple'):
        profiler.discover_anchor(
            proc_root, config.ros_domain_id, config.gz_partition, identity['plugin']
        )


@pytest.mark.parametrize('profile_value', (None, '0', '1\0ROBOTEST_CONTACT_PROFILE=1'))
def test_anchor_requires_exact_contact_profile_environment(
    tmp_path: Path, profile_value: str | None
) -> None:
    config, plugin, producer = _fixture_config(tmp_path)
    identity = _static_identity(config, producer)
    proc_root = tmp_path / 'proc'
    _host_proc(proc_root)
    _make_process(proc_root, 101, plugin, profile_value=profile_value)

    with pytest.raises(
        profiler.ProfileError, match='anchor ROBOTEST_CONTACT_PROFILE must be exactly 1'
    ):
        profiler.discover_anchor(
            proc_root, config.ros_domain_id, config.gz_partition, identity['plugin']
        )


def test_smoke_runner_owner_rejects_wrong_output_root(tmp_path: Path) -> None:
    config, plugin, producer = _fixture_config(tmp_path)
    identity = _static_identity(config, producer)
    proc_root = tmp_path / 'proc'
    _host_proc(proc_root)
    _make_process(proc_root, 101, plugin)
    _set_smoke_runner_option(proc_root, '--output-root', config.candidate_root)
    anchor, _count = profiler.discover_anchor(
        proc_root, config.ros_domain_id, config.gz_partition, identity['plugin']
    )

    with pytest.raises(profiler.ProfileError, match='not the exact smoke runner'):
        profiler.capture_sample(proc_root, config, anchor, profiler.ProfileState())


def test_smoke_runner_owner_rejects_wrong_build_binding(tmp_path: Path) -> None:
    config, plugin, producer = _fixture_config(tmp_path)
    identity = _static_identity(config, producer)
    proc_root = tmp_path / 'proc'
    _host_proc(proc_root)
    _make_process(proc_root, 101, plugin)
    _set_smoke_runner_option(
        proc_root, '--build-binding', config.candidate_root / 'suite-plan.json'
    )
    anchor, _count = profiler.discover_anchor(
        proc_root, config.ros_domain_id, config.gz_partition, identity['plugin']
    )

    with pytest.raises(profiler.ProfileError, match='build binding differs from candidate'):
        profiler.capture_sample(proc_root, config, anchor, profiler.ProfileState())


def test_discovery_brackets_environment_read_against_pid_reuse(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    config, plugin, _producer = _fixture_config(tmp_path)
    proc_root = tmp_path / 'proc'
    _host_proc(proc_root)
    root = _make_process(proc_root, 101, plugin)
    original_read = profiler._bounded_proc_read
    replaced = {'done': False}

    def reuse_after_environment_read(path, maximum, label):
        payload = original_read(path, maximum, label)
        if path == root / 'environ' and not replaced['done']:
            replaced['done'] = True
            (root / 'stat').write_text(_stat_text(101, start=999), encoding='ascii')
        return payload

    monkeypatch.setattr(profiler, '_bounded_proc_read', reuse_after_environment_read)

    with pytest.raises(profiler.ProfileError, match='changed during discovery'):
        profiler.discover_matching(proc_root, config.ros_domain_id, config.gz_partition)


def test_renderer_requires_identity_in_post_start_segment(tmp_path: Path) -> None:
    path = tmp_path / 'ogre2.log'
    path.write_bytes(b'old renderer history\n')
    baseline = profiler._renderer_baseline(path)
    started = 1_800_000_000_000_000_000
    with path.open('ab') as stream:
        stream.write(b'GL_RENDERER = llvmpipe test device\n')
    os.utime(path, ns=(started + 1, started + 1))

    evidence = profiler.collect_renderer(path, baseline, started)

    assert evidence['post_start_segment_mode'] == 'appended'
    assert evidence['post_start_segment_offset_bytes'] == len(b'old renderer history\n')
    assert evidence['identity_lines'] == ['GL_RENDERER = llvmpipe test device']
    assert evidence['pre_smoke_identity']['sha256'] == baseline['sha256']

    stale = tmp_path / 'stale.log'
    stale.write_bytes(b'GL_RENDERER = stale\n')
    stale_baseline = profiler._renderer_baseline(stale)
    with pytest.raises(profiler.ProfileError, match='post-start byte segment'):
        profiler.collect_renderer(stale, stale_baseline, started)


def test_contact_parser_selects_last_fully_valid_cumulative_record(tmp_path: Path) -> None:
    config, _plugin, _producer = _fixture_config(tmp_path)
    first = _contact_record()
    last = _contact_record(1)
    _full_stack_files(config, (first, last))

    evidence = profiler.parse_contact_profile_logs(config.candidate_root / 'smoke')

    assert evidence['present'] is True
    assert evidence['selected_source_valid_record_count'] == 2
    assert evidence['last_valid_cumulative_record'] == last


def test_contact_profile_accepts_sorted_migration_and_reconciles_every_thread() -> None:
    first = _contact_record()
    migrated = _contact_record(1, (101, 202))

    profiler.validate_contact_profile_records((first, migrated))

    assert [contribution['linux_tid'] for contribution in migrated['thread_contributions']] == [
        101,
        202,
    ]


@pytest.mark.parametrize('mutation', ('unsorted', 'duplicate', 'disappeared', 'bad_total'))
def test_contact_profile_rejects_hostile_thread_contribution_sequences(mutation: str) -> None:
    first = _contact_record(0, (101, 202))
    second = _contact_record(1, (101, 202))
    if mutation == 'unsorted':
        second['thread_contributions'].reverse()
    elif mutation == 'duplicate':
        second['thread_contributions'][1]['linux_tid'] = 101
    elif mutation == 'disappeared':
        second = _contact_record(1, (101,))
    elif mutation == 'bad_total':
        second['cached_event_state_check_ns'] += 1
    else:  # pragma: no cover - exhaustive table guard
        raise AssertionError(mutation)

    with pytest.raises(profiler.ProfileError):
        profiler.validate_contact_profile_records((first, second))


def test_contact_parser_surfaces_explicit_producer_failure(tmp_path: Path) -> None:
    config, _plugin, _producer = _fixture_config(tmp_path)
    failure = {
        'failure_kind': 'thread_contribution_overflow',
        'schema_version': 2,
        'sim_stamp_ns': 1,
        'status': 'FAIL',
    }
    _full_stack_files(config, (failure,))
    process_dir = config.candidate_root / 'smoke/processes'
    stdout = process_dir / 'full_stack.stdout.log'
    stderr = process_dir / 'full_stack.stderr.log'
    stderr.write_bytes(stdout.read_bytes())
    stdout.write_bytes(b'')

    with pytest.raises(profiler.ProfileError, match='thread_contribution_overflow'):
        profiler.parse_contact_profile_logs(config.candidate_root / 'smoke')


def test_contact_parser_caps_cumulative_record_count(tmp_path: Path) -> None:
    config, _plugin, _producer = _fixture_config(tmp_path)
    records = tuple(_contact_record(index) for index in range(profiler.MAX_CONTACT_RECORDS + 1))
    _full_stack_files(config, records)

    with pytest.raises(profiler.ProfileError, match='exceeds 128 records'):
        profiler.parse_contact_profile_logs(config.candidate_root / 'smoke')


@pytest.mark.parametrize(
    'mutate',
    (
        lambda record: record.update(extra=1),
        lambda record: record.update(schema_version=True),
        lambda record: record.update(clock_id='CLOCK_MONOTONIC'),
        lambda record: record['thread_contributions'][0].update(linux_tid=0),
        lambda record: record.update(saturated=True),
        lambda record: record.update(measured_total_ns=999),
        lambda record: record.update(sim_stamp_ns=4_000_000_000),
    ),
)
def test_contact_schema_rejects_hostile_record_mutations(mutate) -> None:
    record = _contact_record()
    mutate(record)

    with pytest.raises(profiler.ProfileError):
        profiler.validate_contact_profile_records((record,))


@pytest.mark.parametrize(
    ('field', 'value'),
    (
        ('linux_tid', 9999),
        ('profile_epoch_start_sim_stamp_ns', 2),
        ('sim_stamp_ns', 5_000_000_999),
        ('publish_count', 0),
        ('publish_ns', 1),
    ),
)
def test_contact_sequence_rejects_identity_cadence_or_counter_regression(
    field: str, value: int
) -> None:
    first = _contact_record()
    second = _contact_record(1)
    if field == 'linux_tid':
        second['thread_contributions'][0]['linux_tid'] = value
    else:
        second[field] = value
        if field in profiler.CONTACT_CONTRIBUTION_MONOTONIC_KEYS:
            second['thread_contributions'][0][field] = value
    if field == 'publish_ns':
        second['measured_total_ns'] = sum(second[key] for key in profiler.CONTACT_BUCKET_KEYS)
        second['thread_contributions'][0]['measured_total_ns'] = second['measured_total_ns']

    with pytest.raises(profiler.ProfileError):
        profiler.validate_contact_profile_records((first, second))


@pytest.mark.parametrize('suffix', (b' ', b'\t', b' trailing'))
def test_contact_parser_rejects_noncanonical_suffix(tmp_path: Path, suffix: bytes) -> None:
    config, _plugin, _producer = _fixture_config(tmp_path)
    process_dir = config.candidate_root / 'smoke/processes'
    process_dir.mkdir(parents=True)
    record = profiler.canonical_json_bytes(_contact_record()).rstrip(b'\n')
    (process_dir / 'full_stack.stdout.log').write_bytes(
        profiler.CONTACT_PROFILE_PREFIX + record + suffix + b'\n'
    )

    with pytest.raises(profiler.ProfileError, match='invalid contact profile line'):
        profiler.parse_contact_profile_logs(config.candidate_root / 'smoke')


def test_capture_fails_closed_on_pid_reuse(tmp_path: Path) -> None:
    config, plugin, producer = _fixture_config(tmp_path)
    identity = _static_identity(config, producer)
    proc_root = tmp_path / 'proc'
    _host_proc(proc_root)
    root = _make_process(proc_root, 101, plugin)
    anchor, _count = profiler.discover_anchor(
        proc_root, config.ros_domain_id, config.gz_partition, identity['plugin']
    )
    state = profiler.ProfileState()
    assert profiler.capture_sample(proc_root, config, anchor, state)
    before = copy.deepcopy(state)
    _update_cpu(root, 101, 20, start=999)

    with pytest.raises(profiler.ProfileError, match='reused'):
        profiler.capture_sample(proc_root, config, anchor, state)
    assert state == before


def test_capture_fails_closed_on_thread_tid_reuse(tmp_path: Path) -> None:
    config, plugin, producer = _fixture_config(tmp_path)
    identity = _static_identity(config, producer)
    proc_root = tmp_path / 'proc'
    _host_proc(proc_root)
    root = _make_process(proc_root, 101, plugin)
    anchor, _count = profiler.discover_anchor(
        proc_root, config.ros_domain_id, config.gz_partition, identity['plugin']
    )
    state = profiler.ProfileState()
    assert profiler.capture_sample(proc_root, config, anchor, state)
    before = copy.deepcopy(state)
    (root / 'task/101/stat').write_text(_stat_text(101, start=999), encoding='ascii')

    with pytest.raises(profiler.ProfileError, match='thread 101/101 was reused'):
        profiler.capture_sample(proc_root, config, anchor, state)
    assert state == before


@pytest.mark.parametrize('disappearance_errno', (errno.ENOENT, errno.ESRCH))
def test_sample_threads_marks_process_ended_when_initial_stat_disappears(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    disappearance_errno: int,
) -> None:
    config, plugin, producer = _fixture_config(tmp_path)
    identity = _static_identity(config, producer)
    proc_root = tmp_path / 'proc'
    _host_proc(proc_root)
    root = _make_process(proc_root, 101, plugin)
    anchor, _count = profiler.discover_anchor(
        proc_root, config.ros_domain_id, config.gz_partition, identity['plugin']
    )
    state = profiler.ProfileState()
    assert profiler.capture_sample(proc_root, config, anchor, state) is True
    before = copy.deepcopy(state)
    process_stat = root / 'stat'
    real_open = Path.open

    def disappear_on_initial_process_stat(self, mode='r', *arguments, **keywords):
        if self == process_stat and mode == 'rb':
            raise OSError(
                disappearance_errno,
                os.strerror(disappearance_errno),
                self,
            )
        return real_open(self, mode, *arguments, **keywords)

    monkeypatch.setattr(Path, 'open', disappear_on_initial_process_stat)

    assert profiler._sample_threads(proc_root, state.processes[101], 1, state) is None

    assert state.processes[101].ended_sample == 1
    assert state.thread_records == before.thread_records
    assert state.last_thread_ticks == before.last_thread_ticks
    assert state.thread_start_by_tid == before.thread_start_by_tid
    assert state.thread_names == before.thread_names
    assert state.thread_delta_ticks == before.thread_delta_ticks
    assert state.process_delta_ticks == before.process_delta_ticks


@pytest.mark.parametrize('disappearance_errno', (errno.ENOENT, errno.ESRCH))
def test_capture_counts_thread_stat_disappearance_without_partial_thread_update(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    disappearance_errno: int,
) -> None:
    config, plugin, producer = _fixture_config(tmp_path)
    identity = _static_identity(config, producer)
    proc_root = tmp_path / 'proc'
    _host_proc(proc_root)
    root = _make_process(proc_root, 101, plugin)
    vanished_stat = root / 'task/202/stat'
    vanished_stat.parent.mkdir()
    vanished_stat.write_text(_stat_text(202, start=200), encoding='ascii')
    anchor, _count = profiler.discover_anchor(
        proc_root, config.ros_domain_id, config.gz_partition, identity['plugin']
    )
    real_open = Path.open

    def disappear_on_thread_stat(self, mode='r', *arguments, **keywords):
        if self == vanished_stat and mode == 'rb':
            raise OSError(
                disappearance_errno,
                os.strerror(disappearance_errno),
                self,
            )
        return real_open(self, mode, *arguments, **keywords)

    monkeypatch.setattr(Path, 'open', disappear_on_thread_stat)
    state = profiler.ProfileState()

    assert profiler.capture_sample(proc_root, config, anchor, state) is True

    sample = state.samples[-1]
    process_sample = sample['processes'][0]
    assert sample['anchor_alive'] is True
    assert sample['processes_vanished_during_sample'] == 0
    assert process_sample['thread_count'] == 1
    assert process_sample['threads_vanished_during_sample'] == 1
    assert [thread['tid'] for thread in process_sample['threads']] == [101]
    assert state.thread_records == 1
    assert all(identity[2] != 202 for identity in state.last_thread_ticks)
    assert all(identity[2] != 202 for identity in state.thread_names)
    assert all(identity[2] != 202 for identity in state.thread_delta_ticks)
    assert all(identity[2] != 202 for identity in state.thread_start_by_tid)


@pytest.mark.parametrize('disappearance_errno', (errno.ENOENT, errno.ESRCH))
def test_capture_marks_process_ended_when_task_enumeration_disappears(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    disappearance_errno: int,
) -> None:
    config, plugin, producer = _fixture_config(tmp_path)
    identity = _static_identity(config, producer)
    proc_root = tmp_path / 'proc'
    _host_proc(proc_root)
    root = _make_process(proc_root, 101, plugin)
    anchor, _count = profiler.discover_anchor(
        proc_root, config.ros_domain_id, config.gz_partition, identity['plugin']
    )
    task_root = root / 'task'
    real_iterdir = Path.iterdir

    def disappear_on_task_enumeration(self):
        if self == task_root:
            raise OSError(
                disappearance_errno,
                os.strerror(disappearance_errno),
                self,
            )
        return real_iterdir(self)

    monkeypatch.setattr(Path, 'iterdir', disappear_on_task_enumeration)
    state = profiler.ProfileState()

    assert profiler.capture_sample(proc_root, config, anchor, state) is False

    sample = state.samples[-1]
    assert state.processes[101].ended_sample == 0
    assert sample['anchor_alive'] is False
    assert sample['target_process_set_alive'] is False
    assert sample['process_count'] == 0
    assert sample['processes_vanished_during_sample'] == 1
    assert state.thread_records == 0
    assert state.last_thread_ticks == {}
    assert state.thread_start_by_tid == {}


@pytest.mark.parametrize('disappearance_errno', (errno.ENOENT, errno.ESRCH))
def test_sample_threads_discards_staged_updates_when_final_stat_disappears(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    disappearance_errno: int,
) -> None:
    config, plugin, producer = _fixture_config(tmp_path)
    identity = _static_identity(config, producer)
    proc_root = tmp_path / 'proc'
    _host_proc(proc_root)
    root = _make_process(proc_root, 101, plugin)
    anchor, _count = profiler.discover_anchor(
        proc_root, config.ros_domain_id, config.gz_partition, identity['plugin']
    )
    state = profiler.ProfileState()
    assert profiler.capture_sample(proc_root, config, anchor, state) is True
    _update_cpu(root, 101, 20)
    before = copy.deepcopy(state)
    process_stat = root / 'stat'
    real_open = Path.open
    process_stat_reads = 0

    def disappear_on_final_process_stat(self, mode='r', *arguments, **keywords):
        nonlocal process_stat_reads
        if self == process_stat and mode == 'rb':
            process_stat_reads += 1
            if process_stat_reads == 2:
                raise OSError(
                    disappearance_errno,
                    os.strerror(disappearance_errno),
                    self,
                )
        return real_open(self, mode, *arguments, **keywords)

    monkeypatch.setattr(Path, 'open', disappear_on_final_process_stat)

    assert profiler._sample_threads(proc_root, state.processes[101], 1, state) is None

    assert process_stat_reads == 2
    assert state.processes[101].ended_sample == 1
    assert state.thread_records == before.thread_records
    assert state.last_thread_ticks == before.last_thread_ticks
    assert state.thread_start_by_tid == before.thread_start_by_tid
    assert state.thread_names == before.thread_names
    assert state.thread_delta_ticks == before.thread_delta_ticks
    assert state.process_delta_ticks == before.process_delta_ticks


@pytest.mark.parametrize(
    ('error_type', 'error_number'),
    (
        (PermissionError, errno.EPERM),
        (OSError, errno.EIO),
        (FileNotFoundError, errno.EIO),
    ),
)
def test_capture_keeps_non_disappearance_thread_stat_errors_fatal_and_atomic(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    error_type: type[OSError],
    error_number: int,
) -> None:
    config, plugin, producer = _fixture_config(tmp_path)
    identity = _static_identity(config, producer)
    proc_root = tmp_path / 'proc'
    _host_proc(proc_root)
    root = _make_process(proc_root, 101, plugin)
    denied_stat = root / 'task/202/stat'
    denied_stat.parent.mkdir()
    denied_stat.write_text(_stat_text(202, start=200), encoding='ascii')
    anchor, _count = profiler.discover_anchor(
        proc_root, config.ros_domain_id, config.gz_partition, identity['plugin']
    )
    real_open = Path.open

    def fail_thread_stat(self, mode='r', *arguments, **keywords):
        if self == denied_stat and mode == 'rb':
            raise error_type(error_number, os.strerror(error_number), self)
        return real_open(self, mode, *arguments, **keywords)

    monkeypatch.setattr(Path, 'open', fail_thread_stat)
    state = profiler.ProfileState()

    with pytest.raises(profiler.ProfileError, match='cannot read proc stat'):
        profiler.capture_sample(proc_root, config, anchor, state)

    assert state == profiler.ProfileState()


def test_capture_keeps_malformed_thread_stat_fatal_and_atomic(tmp_path: Path) -> None:
    config, plugin, producer = _fixture_config(tmp_path)
    identity = _static_identity(config, producer)
    proc_root = tmp_path / 'proc'
    _host_proc(proc_root)
    root = _make_process(proc_root, 101, plugin)
    malformed_stat = root / 'task/202/stat'
    malformed_stat.parent.mkdir()
    malformed_stat.write_text('not a proc stat record\n', encoding='ascii')
    anchor, _count = profiler.discover_anchor(
        proc_root, config.ros_domain_id, config.gz_partition, identity['plugin']
    )
    state = profiler.ProfileState()

    with pytest.raises(profiler.ProfileError, match='malformed proc stat'):
        profiler.capture_sample(proc_root, config, anchor, state)

    assert state == profiler.ProfileState()


@pytest.mark.parametrize(
    ('error_type', 'error_number'),
    (
        (PermissionError, errno.EACCES),
        (OSError, errno.EIO),
        (FileNotFoundError, errno.EIO),
    ),
)
def test_capture_keeps_non_disappearance_task_enumeration_errors_fatal_and_atomic(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    error_type: type[OSError],
    error_number: int,
) -> None:
    config, plugin, producer = _fixture_config(tmp_path)
    identity = _static_identity(config, producer)
    proc_root = tmp_path / 'proc'
    _host_proc(proc_root)
    root = _make_process(proc_root, 101, plugin)
    anchor, _count = profiler.discover_anchor(
        proc_root, config.ros_domain_id, config.gz_partition, identity['plugin']
    )
    task_root = root / 'task'
    real_iterdir = Path.iterdir

    def fail_task_enumeration(self):
        if self == task_root:
            raise error_type(error_number, os.strerror(error_number), self)
        return real_iterdir(self)

    monkeypatch.setattr(Path, 'iterdir', fail_task_enumeration)
    state = profiler.ProfileState()

    with pytest.raises(profiler.ProfileError, match='cannot enumerate PID 101 threads'):
        profiler.capture_sample(proc_root, config, anchor, state)

    assert state == profiler.ProfileState()


def test_contact_linux_tid_must_bind_to_a_sampled_anchor_thread(tmp_path: Path) -> None:
    config, plugin, producer = _fixture_config(tmp_path)
    identity = _static_identity(config, producer)
    proc_root = tmp_path / 'proc'
    _host_proc(proc_root)
    root = _make_process(proc_root, 101, plugin)
    (root / 'task/202').mkdir()
    (root / 'task/202/stat').write_text(_stat_text(202, start=200), encoding='ascii')
    anchor, _count = profiler.discover_anchor(
        proc_root, config.ros_domain_id, config.gz_partition, identity['plugin']
    )
    state = profiler.ProfileState()
    profiler.capture_sample(proc_root, config, anchor, state)
    record = _contact_record(linux_tids=(101, 202))
    contact = {'last_valid_cumulative_record': record}

    bindings = profiler.bind_contact_profile_threads(contact, anchor, state)

    assert bindings[0]['anchor_pid'] == 101
    assert bindings[0]['linux_tid'] == 101
    assert bindings[1]['linux_tid'] == 202
    record['thread_contributions'][1]['linux_tid'] = 999
    with pytest.raises(profiler.ProfileError, match='not sampled in the DSO host'):
        profiler.bind_contact_profile_threads(contact, anchor, state)


def test_capture_fails_closed_on_thread_record_overflow(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    config, plugin, producer = _fixture_config(tmp_path)
    identity = _static_identity(config, producer)
    proc_root = tmp_path / 'proc'
    _host_proc(proc_root)
    _make_process(proc_root, 101, plugin)
    anchor, _count = profiler.discover_anchor(
        proc_root, config.ros_domain_id, config.gz_partition, identity['plugin']
    )
    monkeypatch.setattr(profiler, 'MAX_THREAD_RECORDS', 0)
    state = profiler.ProfileState()

    with pytest.raises(profiler.ProfileError, match='thread sample records'):
        profiler.capture_sample(proc_root, config, anchor, state)

    assert state == profiler.ProfileState()


def test_capture_atomically_rejects_more_than_512_live_threads(tmp_path: Path) -> None:
    config, plugin, producer = _fixture_config(tmp_path)
    identity = _static_identity(config, producer)
    proc_root = tmp_path / 'proc'
    _host_proc(proc_root)
    root = _make_process(proc_root, 101, plugin)
    for tid in range(102, 102 + profiler.MAX_LIVE_THREADS_PER_SAMPLE):
        (root / 'task' / str(tid)).mkdir()
        thread_stat = root / 'task' / str(tid) / 'stat'
        thread_stat.write_text(_stat_text(tid, start=tid), encoding='ascii')
    anchor, _count = profiler.discover_anchor(
        proc_root, config.ros_domain_id, config.gz_partition, identity['plugin']
    )
    state = profiler.ProfileState()

    with pytest.raises(profiler.ProfileError, match='exceeds 512 live threads'):
        profiler.capture_sample(proc_root, config, anchor, state)

    assert state == profiler.ProfileState()


def test_production_shaped_sampling_crosses_old_record_cap_without_overflow(
    tmp_path: Path,
) -> None:
    config, plugin, producer = _fixture_config(tmp_path)
    identity = _static_identity(config, producer)
    proc_root = tmp_path / 'proc'
    _host_proc(proc_root)
    root = _make_process(proc_root, 101, plugin)
    for tid in range(102, 501):
        (root / 'task' / str(tid)).mkdir()
        thread_stat = root / 'task' / str(tid) / 'stat'
        thread_stat.write_text(_stat_text(tid, start=tid), encoding='ascii')
    anchor, _count = profiler.discover_anchor(
        proc_root, config.ros_domain_id, config.gz_partition, identity['plugin']
    )
    state = profiler.ProfileState()

    for index in range(164):
        assert profiler.capture_sample(
            proc_root,
            config,
            anchor,
            state,
            monotonic_ns=lambda index=index: index * 500_000_000,
            wall_time_ns=lambda index=index: 1_800_000_000_000_000_000 + index * 500_000_000,
        )

    assert state.thread_records == 65_600
    assert state.thread_records > 65_536
    assert state.thread_records < profiler.MAX_THREAD_RECORDS


def test_capture_caps_aggregate_retained_command_lines(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    config, plugin, producer = _fixture_config(tmp_path)
    identity = _static_identity(config, producer)
    proc_root = tmp_path / 'proc'
    _host_proc(proc_root)
    _make_process(proc_root, 101, plugin)
    anchor, _count = profiler.discover_anchor(
        proc_root, config.ros_domain_id, config.gz_partition, identity['plugin']
    )
    monkeypatch.setattr(profiler, 'MAX_CMDLINE_TOTAL_BYTES', 0)

    with pytest.raises(profiler.ProfileError, match='command lines exceed'):
        profiler.capture_sample(proc_root, config, anchor, profiler.ProfileState())


def test_capture_tracks_the_whole_runner_owned_process_set_until_empty(tmp_path: Path) -> None:
    config, plugin, producer = _fixture_config(tmp_path)
    identity = _static_identity(config, producer)
    proc_root = tmp_path / 'proc'
    _host_proc(proc_root)
    anchor_root = _make_process(proc_root, 101, plugin)
    sibling_root = _make_process(proc_root, 102, plugin, maps_plugin=False, start=200)
    anchor, _count = profiler.discover_anchor(
        proc_root, config.ros_domain_id, config.gz_partition, identity['plugin']
    )
    state = profiler.ProfileState()

    assert profiler.capture_sample(proc_root, config, anchor, state) is True
    shutil.rmtree(anchor_root)
    assert profiler.capture_sample(proc_root, config, anchor, state) is True
    assert state.samples[-1]['anchor_alive'] is False
    assert state.samples[-1]['target_process_set_alive'] is True
    assert state.processes[102].ended_sample is None
    shutil.rmtree(sibling_root)
    assert profiler.capture_sample(proc_root, config, anchor, state) is False
    assert all(process.ended_sample is not None for process in state.processes.values())


def test_capture_tracks_latched_anchor_after_reparenting_until_terminal(tmp_path: Path) -> None:
    config, plugin, producer = _fixture_config(tmp_path)
    identity = _static_identity(config, producer)
    proc_root = tmp_path / 'proc'
    _host_proc(proc_root)
    anchor_root = _make_process(proc_root, 101, plugin)
    anchor, _count = profiler.discover_anchor(
        proc_root, config.ros_domain_id, config.gz_partition, identity['plugin']
    )
    state = profiler.ProfileState()
    assert profiler.capture_sample(proc_root, config, anchor, state) is True

    _replace_process_stat(
        anchor_root,
        101,
        start=100,
        ppid=1,
        group=50,
        session=50,
    )
    assert profiler.capture_sample(proc_root, config, anchor, state) is True
    assert state.samples[-1]['anchor_alive'] is True
    assert state.processes[101].ended_sample is None

    _replace_process_stat(
        anchor_root,
        101,
        start=100,
        ppid=1,
        group=50,
        session=50,
        state='Z',
    )
    assert profiler.capture_sample(proc_root, config, anchor, state) is False
    assert state.samples[-1]['anchor_alive'] is False
    assert state.processes[101].ended_sample == 2


def test_capture_reclassifies_latched_anchor_after_ancestor_disappears(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    config, plugin, producer = _fixture_config(tmp_path)
    identity = _static_identity(config, producer)
    proc_root = tmp_path / 'proc'
    _host_proc(proc_root)
    anchor_root = _make_process(proc_root, 101, plugin)
    anchor, _count = profiler.discover_anchor(
        proc_root, config.ros_domain_id, config.gz_partition, identity['plugin']
    )
    state = profiler.ProfileState()
    assert profiler.capture_sample(proc_root, config, anchor, state) is True
    original_read = profiler._read_stable_process_stat
    raced = False

    def lose_session_leader_once(procfs: Path, pid: int, label: str):
        nonlocal raced
        if pid == 50 and label == 'matching process ancestor' and not raced:
            raced = True
            _replace_process_stat(
                anchor_root,
                101,
                start=100,
                ppid=1,
                group=50,
                session=50,
            )
            shutil.rmtree(proc_root / '50')
            raise profiler.ProcessDisappearedError(pid, label)
        return original_read(procfs, pid, label)

    monkeypatch.setattr(profiler, '_read_stable_process_stat', lose_session_leader_once)

    assert profiler.capture_sample(proc_root, config, anchor, state) is True
    assert raced is True
    assert state.samples[-1]['anchor_alive'] is True
    assert state.processes[101].ended_sample is None


def test_capture_reclassifies_latched_anchor_after_ppid_changes_between_reads(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    config, plugin, producer = _fixture_config(tmp_path)
    identity = _static_identity(config, producer)
    proc_root = tmp_path / 'proc'
    _host_proc(proc_root)
    anchor_root = _make_process(proc_root, 101, plugin)
    anchor, _count = profiler.discover_anchor(
        proc_root, config.ros_domain_id, config.gz_partition, identity['plugin']
    )
    state = profiler.ProfileState()
    assert profiler.capture_sample(proc_root, config, anchor, state) is True
    original_read = profiler._read_stat
    anchor_stat_reads = 0

    def reparent_between_stat_reads(path: Path):
        nonlocal anchor_stat_reads
        result = original_read(path)
        if path == anchor_root / 'stat':
            anchor_stat_reads += 1
            if anchor_stat_reads == 3:
                _replace_process_stat(
                    anchor_root,
                    101,
                    start=100,
                    ppid=40,
                    group=50,
                    session=50,
                )
        return result

    monkeypatch.setattr(profiler, '_read_stat', reparent_between_stat_reads)

    assert profiler.capture_sample(proc_root, config, anchor, state) is True
    assert anchor_stat_reads >= 4
    assert state.samples[-1]['anchor_alive'] is True
    assert state.processes[101].ended_sample is None


def test_capture_reclassifies_latched_anchor_after_intermediate_ancestor_ppid_change(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    config, plugin, producer = _fixture_config(tmp_path)
    identity = _static_identity(config, producer)
    proc_root = tmp_path / 'proc'
    _host_proc(proc_root)
    _make_process(proc_root, 101, plugin)
    anchor, _count = profiler.discover_anchor(
        proc_root, config.ros_domain_id, config.gz_partition, identity['plugin']
    )
    state = profiler.ProfileState()
    assert profiler.capture_sample(proc_root, config, anchor, state) is True
    original_read = profiler._read_stable_process_stat
    raced = False

    def reparent_intermediate_once(procfs: Path, pid: int, label: str):
        nonlocal raced
        if pid == 50 and label == 'matching process ancestor' and not raced:
            raced = True
            (proc_root / '50' / 'stat').write_text(
                _stat_text(50, start=50, ppid=1, group=50, session=50),
                encoding='utf-8',
            )
            raise profiler.ProcessParentChangedError(pid, label)
        return original_read(procfs, pid, label)

    monkeypatch.setattr(profiler, '_read_stable_process_stat', reparent_intermediate_once)

    assert profiler.capture_sample(proc_root, config, anchor, state) is True
    assert raced is True
    assert state.samples[-1]['anchor_alive'] is True
    assert state.processes[101].ended_sample is None


def test_capture_keeps_second_intermediate_ancestor_parent_race_fatal(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    config, plugin, producer = _fixture_config(tmp_path)
    identity = _static_identity(config, producer)
    proc_root = tmp_path / 'proc'
    _host_proc(proc_root)
    _make_process(proc_root, 101, plugin)
    anchor, _count = profiler.discover_anchor(
        proc_root, config.ros_domain_id, config.gz_partition, identity['plugin']
    )
    state = profiler.ProfileState()
    assert profiler.capture_sample(proc_root, config, anchor, state) is True
    before = copy.deepcopy(state)
    original_read = profiler._read_stable_process_stat
    races = 0

    def reparent_intermediate_twice(procfs: Path, pid: int, label: str):
        nonlocal races
        if pid == 50 and label == 'matching process ancestor' and races < 2:
            races += 1
            raise profiler.ProcessParentChangedError(pid, label)
        return original_read(procfs, pid, label)

    monkeypatch.setattr(profiler, '_read_stable_process_stat', reparent_intermediate_twice)

    with pytest.raises(profiler.ProfileError, match='matching process ancestor ancestry changed'):
        profiler.capture_sample(proc_root, config, anchor, state)
    assert races == 2
    assert state == before


def test_capture_counts_reparented_process_exiting_during_cmdline_read(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    config, plugin, producer = _fixture_config(tmp_path)
    identity = _static_identity(config, producer)
    proc_root = tmp_path / 'proc'
    _host_proc(proc_root)
    _make_process(proc_root, 101, plugin)
    child = _make_process(proc_root, 102, plugin, maps_plugin=False, start=200)
    anchor, _count = profiler.discover_anchor(
        proc_root, config.ros_domain_id, config.gz_partition, identity['plugin']
    )
    state = profiler.ProfileState()
    assert profiler.capture_sample(proc_root, config, anchor, state) is True
    _replace_process_stat(
        child,
        102,
        start=200,
        ppid=1,
        group=50,
        session=50,
    )
    original_read = profiler._bounded_proc_read

    def exit_on_cmdline(path: Path, maximum: int, label: str) -> bytes:
        if path == child / 'cmdline':
            shutil.rmtree(child)
            raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), path)
        return original_read(path, maximum, label)

    monkeypatch.setattr(profiler, '_bounded_proc_read', exit_on_cmdline)

    assert profiler.capture_sample(proc_root, config, anchor, state) is True
    assert state.processes[102].ended_sample == 1
    assert state.samples[-1]['processes_vanished_during_sample'] == 1


def test_capture_counts_reparented_process_becoming_terminal_during_executable_read(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    config, plugin, producer = _fixture_config(tmp_path)
    identity = _static_identity(config, producer)
    proc_root = tmp_path / 'proc'
    _host_proc(proc_root)
    _make_process(proc_root, 101, plugin)
    child = _make_process(proc_root, 102, plugin, maps_plugin=False, start=200)
    anchor, _count = profiler.discover_anchor(
        proc_root, config.ros_domain_id, config.gz_partition, identity['plugin']
    )
    state = profiler.ProfileState()
    assert profiler.capture_sample(proc_root, config, anchor, state) is True
    _replace_process_stat(
        child,
        102,
        start=200,
        ppid=1,
        group=50,
        session=50,
    )
    original_readlink = profiler.os.readlink

    def become_terminal_on_executable(path: Path) -> str:
        if path == child / 'exe':
            _replace_process_stat(
                child,
                102,
                start=200,
                ppid=1,
                group=50,
                session=50,
                state='Z',
            )
            (child / 'exe').unlink()
        return original_readlink(path)

    monkeypatch.setattr(profiler.os, 'readlink', become_terminal_on_executable)

    assert profiler.capture_sample(proc_root, config, anchor, state) is True
    assert state.processes[102].ended_sample == 1
    assert state.samples[-1]['processes_vanished_during_sample'] == 1


def test_capture_counts_reparented_process_exiting_during_executable_esrch(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    config, plugin, producer = _fixture_config(tmp_path)
    identity = _static_identity(config, producer)
    proc_root = tmp_path / 'proc'
    _host_proc(proc_root)
    _make_process(proc_root, 101, plugin)
    child = _make_process(proc_root, 102, plugin, maps_plugin=False, start=200)
    anchor, _count = profiler.discover_anchor(
        proc_root, config.ros_domain_id, config.gz_partition, identity['plugin']
    )
    state = profiler.ProfileState()
    assert profiler.capture_sample(proc_root, config, anchor, state) is True
    _replace_process_stat(
        child,
        102,
        start=200,
        ppid=1,
        group=50,
        session=50,
    )
    original_readlink = profiler.os.readlink

    def exit_on_executable_read(path: Path) -> str:
        if path == child / 'exe':
            shutil.rmtree(child)
            raise ProcessLookupError(errno.ESRCH, os.strerror(errno.ESRCH), path)
        return original_readlink(path)

    monkeypatch.setattr(profiler.os, 'readlink', exit_on_executable_read)

    assert profiler.capture_sample(proc_root, config, anchor, state) is True
    assert state.processes[102].ended_sample == 1
    assert state.samples[-1]['processes_vanished_during_sample'] == 1


def test_capture_keeps_live_reparented_executable_enoent_fatal(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    config, plugin, producer = _fixture_config(tmp_path)
    identity = _static_identity(config, producer)
    proc_root = tmp_path / 'proc'
    _host_proc(proc_root)
    _make_process(proc_root, 101, plugin)
    child = _make_process(proc_root, 102, plugin, maps_plugin=False, start=200)
    anchor, _count = profiler.discover_anchor(
        proc_root, config.ros_domain_id, config.gz_partition, identity['plugin']
    )
    state = profiler.ProfileState()
    assert profiler.capture_sample(proc_root, config, anchor, state) is True
    before = copy.deepcopy(state)
    _replace_process_stat(
        child,
        102,
        start=200,
        ppid=1,
        group=50,
        session=50,
    )
    original_readlink = profiler.os.readlink

    def report_missing_executable(path: Path) -> str:
        if path == child / 'exe':
            raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), path)
        return original_readlink(path)

    monkeypatch.setattr(profiler.os, 'readlink', report_missing_executable)

    with pytest.raises(profiler.ProfileError, match='reparented matching process disappeared'):
        profiler.capture_sample(proc_root, config, anchor, state)
    assert state == before


@pytest.mark.parametrize(
    ('error_type', 'error_number'),
    (
        (PermissionError, errno.EACCES),
        (OSError, errno.EIO),
        (FileNotFoundError, errno.EIO),
    ),
)
def test_capture_keeps_live_reparented_executable_non_disappearance_error_fatal(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    error_type: type[OSError],
    error_number: int,
) -> None:
    config, plugin, producer = _fixture_config(tmp_path)
    identity = _static_identity(config, producer)
    proc_root = tmp_path / 'proc'
    _host_proc(proc_root)
    _make_process(proc_root, 101, plugin)
    child = _make_process(proc_root, 102, plugin, maps_plugin=False, start=200)
    anchor, _count = profiler.discover_anchor(
        proc_root, config.ros_domain_id, config.gz_partition, identity['plugin']
    )
    state = profiler.ProfileState()
    assert profiler.capture_sample(proc_root, config, anchor, state) is True
    before = copy.deepcopy(state)
    _replace_process_stat(
        child,
        102,
        start=200,
        ppid=1,
        group=50,
        session=50,
    )
    original_readlink = profiler.os.readlink

    def deny_executable_read(path: Path) -> str:
        if path == child / 'exe':
            raise error_type(error_number, os.strerror(error_number), path)
        return original_readlink(path)

    monkeypatch.setattr(profiler.os, 'readlink', deny_executable_read)

    with pytest.raises(profiler.ProfileError, match='cannot read PID 102 executable'):
        profiler.capture_sample(proc_root, config, anchor, state)
    assert state == before


def test_capture_rejects_reparented_pid_reuse(tmp_path: Path) -> None:
    config, plugin, producer = _fixture_config(tmp_path)
    identity = _static_identity(config, producer)
    proc_root = tmp_path / 'proc'
    _host_proc(proc_root)
    anchor_root = _make_process(proc_root, 101, plugin)
    anchor, _count = profiler.discover_anchor(
        proc_root, config.ros_domain_id, config.gz_partition, identity['plugin']
    )
    state = profiler.ProfileState()
    assert profiler.capture_sample(proc_root, config, anchor, state) is True
    before = copy.deepcopy(state)

    _replace_process_stat(
        anchor_root,
        101,
        start=999,
        ppid=1,
        group=50,
        session=50,
    )
    with pytest.raises(profiler.ProfileError, match='latched PID 101 was reused'):
        profiler.capture_sample(proc_root, config, anchor, state)
    assert state == before


@pytest.mark.parametrize(
    ('group', 'session'),
    (
        (51, 50),
        (50, 51),
    ),
)
def test_capture_rejects_reparented_group_or_session_drift(
    tmp_path: Path, group: int, session: int
) -> None:
    config, plugin, producer = _fixture_config(tmp_path)
    identity = _static_identity(config, producer)
    proc_root = tmp_path / 'proc'
    _host_proc(proc_root)
    anchor_root = _make_process(proc_root, 101, plugin)
    anchor, _count = profiler.discover_anchor(
        proc_root, config.ros_domain_id, config.gz_partition, identity['plugin']
    )
    state = profiler.ProfileState()
    assert profiler.capture_sample(proc_root, config, anchor, state) is True
    before = copy.deepcopy(state)

    _replace_process_stat(
        anchor_root,
        101,
        start=100,
        ppid=1,
        group=group,
        session=session,
    )
    with pytest.raises(profiler.ProfileError, match='process group or session changed'):
        profiler.capture_sample(proc_root, config, anchor, state)
    assert state == before


@pytest.mark.parametrize('drift', ('command', 'executable'))
def test_capture_rejects_reparented_command_or_executable_drift(
    tmp_path: Path,
    drift: str,
) -> None:
    config, plugin, producer = _fixture_config(tmp_path)
    identity = _static_identity(config, producer)
    proc_root = tmp_path / 'proc'
    _host_proc(proc_root)
    anchor_root = _make_process(proc_root, 101, plugin)
    anchor, _count = profiler.discover_anchor(
        proc_root, config.ros_domain_id, config.gz_partition, identity['plugin']
    )
    state = profiler.ProfileState()
    assert profiler.capture_sample(proc_root, config, anchor, state) is True
    before = copy.deepcopy(state)

    _replace_process_stat(
        anchor_root,
        101,
        start=100,
        ppid=1,
        group=50,
        session=50,
    )
    if drift == 'command':
        (anchor_root / 'cmdline').write_bytes(b'/usr/bin/ruby3.2\0gz\0sim\0--changed\0')
    else:
        replacement = tmp_path / 'replacement-executable'
        replacement.write_bytes(b'replacement')
        (anchor_root / 'exe').unlink()
        (anchor_root / 'exe').symlink_to(replacement)
    with pytest.raises(profiler.ProfileError, match='command identity changed'):
        profiler.capture_sample(proc_root, config, anchor, state)
    assert state == before


def test_capture_rejects_latched_process_isolation_drift(tmp_path: Path) -> None:
    config, plugin, producer = _fixture_config(tmp_path)
    identity = _static_identity(config, producer)
    proc_root = tmp_path / 'proc'
    _host_proc(proc_root)
    anchor_root = _make_process(proc_root, 101, plugin)
    anchor, _count = profiler.discover_anchor(
        proc_root, config.ros_domain_id, config.gz_partition, identity['plugin']
    )
    state = profiler.ProfileState()
    assert profiler.capture_sample(proc_root, config, anchor, state) is True
    before = copy.deepcopy(state)

    _replace_process_stat(
        anchor_root,
        101,
        start=100,
        ppid=1,
        group=50,
        session=50,
    )
    (anchor_root / 'environ').write_bytes(
        b'ROS_DOMAIN_ID=117\0GZ_PARTITION=robotest_p3_phase3-test-001-smoke_00\0'
    )
    with pytest.raises(profiler.ProfileError, match='changed isolation identity'):
        profiler.capture_sample(proc_root, config, anchor, state)
    assert state == before


def test_capture_counts_latched_process_vanishing_during_ancestry(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    config, plugin, producer = _fixture_config(tmp_path)
    identity = _static_identity(config, producer)
    proc_root = tmp_path / 'proc'
    _host_proc(proc_root)
    _make_process(proc_root, 101, plugin)
    child = _make_process(proc_root, 102, plugin, maps_plugin=False, start=200)
    anchor, _count = profiler.discover_anchor(
        proc_root, config.ros_domain_id, config.gz_partition, identity['plugin']
    )
    state = profiler.ProfileState()
    assert profiler.capture_sample(proc_root, config, anchor, state) is True
    original_discover = profiler.discover_matching

    def discover_then_remove(*args):
        matches = original_discover(*args)
        shutil.rmtree(child)
        return matches

    monkeypatch.setattr(profiler, 'discover_matching', discover_then_remove)

    assert profiler.capture_sample(proc_root, config, anchor, state) is True
    assert state.processes[102].ended_sample == 1
    assert state.samples[-1]['processes_vanished_during_sample'] == 1


@pytest.mark.parametrize(
    ('ppid', 'group', 'session'),
    (
        (50, 50, 50),
        (1, 303, 303),
    ),
)
def test_capture_rejects_unlatched_process_vanishing_before_ownership_proof(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    ppid: int,
    group: int,
    session: int,
) -> None:
    config, plugin, producer = _fixture_config(tmp_path)
    identity = _static_identity(config, producer)
    proc_root = tmp_path / 'proc'
    _host_proc(proc_root)
    _make_process(proc_root, 101, plugin)
    anchor, _count = profiler.discover_anchor(
        proc_root, config.ros_domain_id, config.gz_partition, identity['plugin']
    )
    state = profiler.ProfileState()
    assert profiler.capture_sample(proc_root, config, anchor, state) is True
    before = copy.deepcopy(state)
    new_process = _make_process(
        proc_root,
        303,
        plugin,
        maps_plugin=False,
        start=300,
        ppid=ppid,
        group=group,
        session=session,
    )
    original_discover = profiler.discover_matching

    def discover_then_remove(*args):
        matches = original_discover(*args)
        shutil.rmtree(new_process)
        return matches

    monkeypatch.setattr(profiler, 'discover_matching', discover_then_remove)

    with pytest.raises(profiler.ProfileError, match='matching process disappeared'):
        profiler.capture_sample(proc_root, config, anchor, state)
    assert state == before


def test_capture_counts_same_identity_becoming_terminal_during_ancestry(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    config, plugin, producer = _fixture_config(tmp_path)
    identity = _static_identity(config, producer)
    proc_root = tmp_path / 'proc'
    _host_proc(proc_root)
    _make_process(proc_root, 101, plugin)
    child = _make_process(proc_root, 102, plugin, maps_plugin=False, start=200)
    anchor, _count = profiler.discover_anchor(
        proc_root, config.ros_domain_id, config.gz_partition, identity['plugin']
    )
    state = profiler.ProfileState()
    assert profiler.capture_sample(proc_root, config, anchor, state) is True
    original_read = profiler._read_stable_process_stat
    raced = False

    def become_terminal(procfs: Path, pid: int, label: str):
        nonlocal raced
        if pid == 102 and label == 'matching process' and not raced:
            raced = True
            terminal = _stat_text(102, state='Z', start=200)
            (child / 'stat').write_text(terminal, encoding='ascii')
            (child / 'task/102/stat').write_text(terminal, encoding='ascii')
            raise profiler.ProcessDisappearedError(pid, label)
        return original_read(procfs, pid, label)

    monkeypatch.setattr(profiler, '_read_stable_process_stat', become_terminal)

    assert profiler.capture_sample(proc_root, config, anchor, state) is True
    assert state.processes[102].ended_sample == 1
    assert state.samples[-1]['processes_vanished_during_sample'] == 1


def test_capture_keeps_live_ancestry_disappearance_uncertainty_fatal(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    config, plugin, producer = _fixture_config(tmp_path)
    identity = _static_identity(config, producer)
    proc_root = tmp_path / 'proc'
    _host_proc(proc_root)
    _make_process(proc_root, 101, plugin)
    _make_process(proc_root, 102, plugin, maps_plugin=False, start=200)
    anchor, _count = profiler.discover_anchor(
        proc_root, config.ros_domain_id, config.gz_partition, identity['plugin']
    )
    state = profiler.ProfileState()
    assert profiler.capture_sample(proc_root, config, anchor, state) is True
    before = copy.deepcopy(state)
    original_read = profiler._read_stable_process_stat
    raced = False

    def report_disappearance(procfs: Path, pid: int, label: str):
        nonlocal raced
        if pid == 102 and label == 'matching process' and not raced:
            raced = True
            raise profiler.ProcessDisappearedError(pid, label)
        return original_read(procfs, pid, label)

    monkeypatch.setattr(profiler, '_read_stable_process_stat', report_disappearance)

    with pytest.raises(profiler.ProfileError, match='matching process disappeared'):
        profiler.capture_sample(proc_root, config, anchor, state)
    assert state == before


def test_capture_keeps_ancestor_disappearance_fatal(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    config, plugin, producer = _fixture_config(tmp_path)
    identity = _static_identity(config, producer)
    proc_root = tmp_path / 'proc'
    _host_proc(proc_root)
    _make_process(proc_root, 101, plugin)
    anchor, _count = profiler.discover_anchor(
        proc_root, config.ros_domain_id, config.gz_partition, identity['plugin']
    )
    state = profiler.ProfileState()
    assert profiler.capture_sample(proc_root, config, anchor, state) is True
    before = copy.deepcopy(state)
    original_read = profiler._read_stable_process_stat

    def lose_ancestor(procfs: Path, pid: int, label: str):
        if pid == 50 and label == 'matching process ancestor':
            raise profiler.ProcessDisappearedError(pid, label)
        return original_read(procfs, pid, label)

    monkeypatch.setattr(profiler, '_read_stable_process_stat', lose_ancestor)

    with pytest.raises(profiler.ProfileError, match='matching process ancestor disappeared'):
        profiler.capture_sample(proc_root, config, anchor, state)
    assert state == before


def test_capture_keeps_raced_pid_reuse_fatal(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    config, plugin, producer = _fixture_config(tmp_path)
    identity = _static_identity(config, producer)
    proc_root = tmp_path / 'proc'
    _host_proc(proc_root)
    _make_process(proc_root, 101, plugin)
    child = _make_process(proc_root, 102, plugin, maps_plugin=False, start=200)
    anchor, _count = profiler.discover_anchor(
        proc_root, config.ros_domain_id, config.gz_partition, identity['plugin']
    )
    state = profiler.ProfileState()
    assert profiler.capture_sample(proc_root, config, anchor, state) is True
    before = copy.deepcopy(state)
    original_read = profiler._read_stable_process_stat
    raced = False

    def reuse_pid(procfs: Path, pid: int, label: str):
        nonlocal raced
        if pid == 102 and label == 'matching process' and not raced:
            raced = True
            (child / 'stat').write_text(_stat_text(102, start=999), encoding='ascii')
            raise profiler.ProcessDisappearedError(pid, label)
        return original_read(procfs, pid, label)

    monkeypatch.setattr(profiler, '_read_stable_process_stat', reuse_pid)

    with pytest.raises(profiler.ProfileError, match='matching PID 102 was reused'):
        profiler.capture_sample(proc_root, config, anchor, state)
    assert state == before


@pytest.mark.parametrize('terminal_state', ('X', 'Z'))
def test_capture_stable_terminal_child_as_ended(tmp_path: Path, terminal_state: str) -> None:
    config, plugin, producer = _fixture_config(tmp_path)
    identity = _static_identity(config, producer)
    proc_root = tmp_path / 'proc'
    _host_proc(proc_root)
    _make_process(proc_root, 101, plugin)
    child = _make_process(proc_root, 102, plugin, maps_plugin=False, start=200)
    anchor, _count = profiler.discover_anchor(
        proc_root, config.ros_domain_id, config.gz_partition, identity['plugin']
    )
    state = profiler.ProfileState()
    assert profiler.capture_sample(proc_root, config, anchor, state) is True
    terminal_stat = _stat_text(102, state=terminal_state, start=200)
    (child / 'stat').write_text(terminal_stat, encoding='ascii')
    (child / 'task/102/stat').write_text(terminal_stat, encoding='ascii')
    (child / 'environ').write_bytes(b'')

    assert profiler.capture_sample(proc_root, config, anchor, state) is True
    assert state.processes[102].ended_sample == 1
    assert profiler.capture_sample(proc_root, config, anchor, state) is True
    assert state.processes[102].ended_sample == 1


def test_capture_accepts_exact_isolation_in_runner_owned_sibling_group(tmp_path: Path) -> None:
    config, plugin, producer = _fixture_config(tmp_path)
    identity = _static_identity(config, producer)
    proc_root = tmp_path / 'proc'
    _host_proc(proc_root)
    _make_process(proc_root, 101, plugin)
    _make_process(
        proc_root,
        202,
        plugin,
        maps_plugin=False,
        start=200,
        ppid=40,
        group=202,
        session=202,
    )
    anchor, _count = profiler.discover_anchor(
        proc_root, config.ros_domain_id, config.gz_partition, identity['plugin']
    )
    state = profiler.ProfileState()

    assert profiler.capture_sample(proc_root, config, anchor, state) is True
    assert sorted(state.processes) == [101, 202]


def test_capture_rejects_foreign_exact_isolation_process_tree(tmp_path: Path) -> None:
    config, plugin, producer = _fixture_config(tmp_path)
    identity = _static_identity(config, producer)
    proc_root = tmp_path / 'proc'
    _host_proc(proc_root)
    _make_process(proc_root, 101, plugin)
    _make_process(
        proc_root,
        303,
        plugin,
        maps_plugin=False,
        start=300,
        ppid=1,
        group=303,
        session=303,
    )
    anchor, _count = profiler.discover_anchor(
        proc_root, config.ros_domain_id, config.gz_partition, identity['plugin']
    )

    with pytest.raises(profiler.ProfileError, match='shared outside') as caught:
        profiler.capture_sample(proc_root, config, anchor, profiler.ProfileState())
    assert 'pid=303,start=300,ppid=1,pgid=303,sid=303' in str(caught.value)


def test_capture_rejects_smoke_runner_disappearance_or_pid_reuse(tmp_path: Path) -> None:
    config, plugin, producer = _fixture_config(tmp_path)
    identity = _static_identity(config, producer)
    proc_root = tmp_path / 'proc'
    _host_proc(proc_root)
    _make_process(proc_root, 101, plugin)
    anchor, _count = profiler.discover_anchor(
        proc_root, config.ros_domain_id, config.gz_partition, identity['plugin']
    )
    state = profiler.ProfileState()
    assert profiler.capture_sample(proc_root, config, anchor, state) is True
    owner_root = proc_root / '40'
    saved = {
        path.name: path.read_bytes()
        for path in owner_root.iterdir()
        if path.is_file() and not path.is_symlink()
    }
    executable_target = owner_root.joinpath('exe').resolve()
    shutil.rmtree(owner_root)

    with pytest.raises(profiler.ProfileError, match=r'smoke runner owner.*disappeared'):
        profiler.capture_sample(proc_root, config, anchor, state)

    owner_root.mkdir()
    for name, payload in saved.items():
        (owner_root / name).write_bytes(payload)
    (owner_root / 'exe').symlink_to(executable_target)
    (owner_root / 'stat').write_text(
        _stat_text(40, comm='python3', start=999, ppid=1, group=30, session=30),
        encoding='ascii',
    )
    with pytest.raises(profiler.ProfileError, match='smoke runner owner PID 40 was reused'):
        profiler.capture_sample(proc_root, config, anchor, state)


def test_full_stack_join_rejects_truncated_stream_or_unrelated_anchor(tmp_path: Path) -> None:
    config, _plugin, _producer = _fixture_config(tmp_path)
    _full_stack_files(config, (_contact_record(),))
    anchor = {'process_group': 60, 'session': 50}
    clock = _Clock()

    evidence = profiler.wait_for_full_stack_close(
        config.candidate_root / 'smoke',
        anchor,
        clock.monotonic,
        clock.sleep,
        profile_started_monotonic_ns=0,
    )

    assert evidence['streams']['stdout']['size_bytes'] > 0
    with pytest.raises(profiler.ProfileError, match='full-stack session'):
        profiler.wait_for_full_stack_close(
            config.candidate_root / 'smoke',
            {'process_group': 51, 'session': 51},
            clock.monotonic,
            clock.sleep,
            profile_started_monotonic_ns=0,
        )

    metadata_path = config.candidate_root / 'smoke/processes/full_stack.process.json'
    metadata = __import__('json').loads(metadata_path.read_text(encoding='utf-8'))
    metadata['stdout']['observed_bytes'] -= 1
    metadata_path.write_bytes(profiler.canonical_json_bytes(metadata))
    with pytest.raises(profiler.ProfileError, match='fully retained'):
        profiler.wait_for_full_stack_close(
            config.candidate_root / 'smoke',
            anchor,
            clock.monotonic,
            clock.sleep,
            profile_started_monotonic_ns=0,
        )


def test_full_stack_join_rejects_metadata_started_before_profiler(tmp_path: Path) -> None:
    config, _plugin, _producer = _fixture_config(tmp_path)
    _full_stack_files(config, (_contact_record(),))
    clock = _Clock()

    with pytest.raises(profiler.ProfileError, match='did not close cleanly'):
        profiler.wait_for_full_stack_close(
            config.candidate_root / 'smoke',
            {'process_group': 50, 'session': 50},
            clock.monotonic,
            clock.sleep,
            profile_started_monotonic_ns=2,
        )


def test_contact_parser_second_read_must_match_closed_stream_hashes(tmp_path: Path) -> None:
    config, _plugin, _producer = _fixture_config(tmp_path)
    _full_stack_files(config, (_contact_record(),))
    clock = _Clock()
    full_stack = profiler.wait_for_full_stack_close(
        config.candidate_root / 'smoke',
        {'process_group': 50, 'session': 50},
        clock.monotonic,
        clock.sleep,
        profile_started_monotonic_ns=0,
    )
    stdout = config.candidate_root / 'smoke/processes/full_stack.stdout.log'
    with stdout.open('ab') as stream:
        stream.write(b'unexpected post-close mutation\n')
    contact = profiler.parse_contact_profile_logs(config.candidate_root / 'smoke')

    with pytest.raises(profiler.ProfileError, match='changed after close'):
        profiler.reconcile_contact_log_sources(contact, full_stack)


class _Clock:
    def __init__(self, on_sleep=None) -> None:
        self.seconds = 0.0
        self.epoch_ns = 1_800_000_000_000_000_000
        self.on_sleep = on_sleep
        self.sleeps = []

    def monotonic(self) -> float:
        return self.seconds

    def monotonic_ns(self) -> int:
        return int(self.seconds * 1_000_000_000)

    def wall_time_ns(self) -> int:
        return self.epoch_ns + self.monotonic_ns()

    def sleep(self, duration: float) -> None:
        assert duration >= 0
        self.sleeps.append(duration)
        self.seconds += duration
        if self.on_sleep is not None:
            self.on_sleep(self)


def _write_complete_campaign_profile_fixture(tmp_path: Path):
    """Write one deterministic, production-shaped profiled-smoke prerequisite."""
    config, plugin, producer = _fixture_config(tmp_path)
    config = profiler.ProfileConfig(
        **{
            **vars(config),
            'startup_timeout_s': profiler.CANONICAL_STARTUP_TIMEOUT_S,
            'sample_period_s': profiler.CANONICAL_SAMPLE_PERIOD_S,
            'max_duration_s': profiler.CANONICAL_MAX_DURATION_S,
        }
    )
    identity = _static_identity(config, producer)
    proc_root = tmp_path / 'proc'
    _host_proc(proc_root)
    group_leader = _make_process(
        proc_root,
        50,
        plugin,
        start=50,
        maps_plugin=False,
        ppid=40,
    )
    root = _make_process(proc_root, 101, plugin, group=60)
    config.renderer_log.write_bytes(b'old renderer history\n')
    steps = {'count': 0}

    def on_sleep(clock: _Clock) -> None:
        steps['count'] += 1
        if steps['count'] < 4:
            _update_cpu(root, 101, 10 + steps['count'] * 5, group=60)
        elif root.exists():
            shutil.rmtree(root)
            shutil.rmtree(group_leader)
            _full_stack_files(config, (_contact_record(), _contact_record(1)))
            with config.renderer_log.open('ab') as stream:
                stream.write(b'Device Name: llvmpipe deterministic\n')
            stamp = clock.wall_time_ns() + 1
            os.utime(config.renderer_log, ns=(stamp, stamp))

    clock = _Clock(on_sleep)
    document = profiler.SmokeHostProfiler(
        config,
        identity,
        proc_root=proc_root,
        monotonic=clock.monotonic,
        monotonic_ns=clock.monotonic_ns,
        wall_time_ns=clock.wall_time_ns,
        sleep=clock.sleep,
    ).run()
    profile_hash = profiler.write_profile(config.output_path, document)
    plan = __import__('json').loads(
        (config.candidate_root / 'suite-plan.json').read_text(encoding='utf-8')
    )
    trial = plan['trials'][0]
    result = {
        'events': [],
        'identity': {
            'candidate_id': f'{config.candidate_id}-smoke',
            'cold_stack': True,
            'git_dirty': False,
            'git_sha': GIT_SHA,
            'gz_partition': plan['smoke']['gz_partition'],
            'repetition_index': trial['repetition_index'],
            'ros_domain_id': plan['smoke']['ros_domain_id'],
            'run_id': plan['smoke']['run_id'],
            'scenario_id': trial['scenario_id'],
            'scenario_index': trial['scenario_id'],
            'scenario_name': trial['scenario_name'],
            'scenario_sha256': trial['scenario_sha256'],
            'suite_index': trial['suite_index'],
        },
        'measurements': {
            'accepted_goal_stamp_ns': 1,
            'actual_path_length_m': 1.0,
            'collision_count': 0,
            'completed_waypoint_count': 2,
            'completion_time_sim_s': 1.0,
            'initial_planned_path_length_m': 1.0,
            'mission_action_status': 'SUCCEEDED',
            'path_efficiency': 1.0,
            'peak_rss_sum_bytes': 1,
            'replan_count': 0,
            'rtf_median': 1.0,
            'rtf_p5': 1.0,
            'terminal_action_stamp_ns': 2,
        },
        'quality': {
            'artifact_caps': {
                'canonical_csv_max_bytes': 1_048_576,
                'canonical_json_max_bytes': 33_554_432,
                'log_max_bytes': 8_388_608,
                'png_max_bytes': 4_194_304,
                'png_max_count': 8,
                'run_directory_max_bytes': 268_435_456,
            },
            'artifact_projection_preflight': 'PASS',
            'candidate_identity': {},
            'capture': {
                'clock': {},
                'failures': {},
                'overflowed': False,
                'status': 'PASS',
                'streams': {},
            },
            'collector_overflow': False,
            'component_failures': {},
            'components': {
                'fault_control': {},
                'mission': {},
                'mission_artifact_sha256': '4' * 64,
                'orchestrator': {},
                'scenario': {},
            },
            'contract_revision': 2,
            'infrastructure_failure': None,
            'metric_unavailable_reasons': {},
        },
        'targets': {},
        'verdict': {
            'automated_status': 'PASS',
            'capture_integrity': True,
            'components_complete': True,
            'exit_code': 0,
            'mission_success': True,
            'required_metric_checks': [],
            'scenario_metric_gate': True,
            'threshold_checks': [],
        },
    }
    result_path = config.candidate_root / 'smoke/result/run-result.json'
    result_hash = _canonical_unsigned(result_path, result)
    result_dir = result_path.parent
    (result_dir / 'run-result.csv').write_bytes(profiler._one_row_csv_bytes(result))
    (result_dir / 'report.md').write_text('# Fixture report\n', encoding='utf-8')
    (result_dir / 'report.html').write_text('<p>Fixture report</p>\n', encoding='utf-8')
    artifact_records = []
    for path in sorted(
        (item for item in result_dir.iterdir() if item.is_file()),
        key=lambda item: item.name,
    ):
        payload = path.read_bytes()
        artifact_records.append(
            {
                'bytes': len(payload),
                'path': path.name,
                'sha256': __import__('hashlib').sha256(payload).hexdigest(),
            }
        )
    _canonical(
        result_dir / 'run-artifacts.manifest.json',
        {
            'artifacts': artifact_records,
            'identity': {
                'run_id': result['identity']['run_id'],
                'run_result_sha256': result_hash,
            },
            'producer': 'robotest_metrics/metrics_analyze',
            'quality': {
                'artifact_bytes_excluding_manifest': sum(
                    record['bytes'] for record in artifact_records
                ),
                'artifact_count': len(artifact_records),
                'caps_within_limits': True,
                'hashes_verified': True,
                'path_set_complete': True,
            },
            'schema_version': 1,
        },
    )
    _canonical(
        config.candidate_root / 'smoke/PASS.json',
        {
            'producer': profiler.BENCHMARK_PRODUCER,
            'run_result_sha256': result_hash,
            'status': 'PASS',
        },
    )
    return config, document, profile_hash


def _rewrite_profile(config, document: dict) -> str:
    return _canonical(config.output_path, document)


def _rewrite_result_manifest(result_path: Path, result_hash: str) -> None:
    manifest_path = result_path.parent / 'run-artifacts.manifest.json'
    result = __import__('json').loads(result_path.read_text(encoding='utf-8'))
    records = []
    for path in sorted(
        (
            item
            for item in result_path.parent.iterdir()
            if item.name
            not in {'run-artifacts.manifest.json', 'run-artifacts.manifest.json.sha256'}
        ),
        key=lambda item: item.name,
    ):
        payload = path.read_bytes()
        records.append(
            {
                'bytes': len(payload),
                'path': path.name,
                'sha256': __import__('hashlib').sha256(payload).hexdigest(),
            }
        )
    _canonical(
        manifest_path,
        {
            'artifacts': records,
            'identity': {
                'run_id': result['identity']['run_id'],
                'run_result_sha256': result_hash,
            },
            'producer': 'robotest_metrics/metrics_analyze',
            'quality': {
                'artifact_bytes_excluding_manifest': sum(record['bytes'] for record in records),
                'artifact_count': len(records),
                'caps_within_limits': True,
                'hashes_verified': True,
                'path_set_complete': True,
            },
            'schema_version': 1,
        },
    )


def _rewrite_result_bundle(result_path: Path, result: dict) -> str:
    result_hash = _canonical_unsigned(result_path, result)
    (result_path.parent / 'run-result.csv').write_bytes(profiler._one_row_csv_bytes(result))
    _rewrite_result_manifest(result_path, result_hash)
    return result_hash


def test_profiler_completes_deterministically_without_live_ros(tmp_path: Path) -> None:
    config, plugin, producer = _fixture_config(tmp_path)
    identity = _static_identity(config, producer)
    proc_root = tmp_path / 'proc'
    _host_proc(proc_root)
    root = _make_process(proc_root, 101, plugin)
    config.renderer_log.write_bytes(b'old renderer history\n')
    steps = {'count': 0}

    def on_sleep(clock: _Clock) -> None:
        steps['count'] += 1
        if steps['count'] < 4:
            _update_cpu(root, 101, 10 + steps['count'] * 5)
        elif root.exists():
            shutil.rmtree(root)
            _full_stack_files(config, (_contact_record(), _contact_record(1)))
            with config.renderer_log.open('ab') as stream:
                stream.write(b'Device Name: llvmpipe deterministic\n')
            stamp = clock.wall_time_ns() + 1
            os.utime(config.renderer_log, ns=(stamp, stamp))

    clock = _Clock(on_sleep)
    monitor = profiler.SmokeHostProfiler(
        config,
        identity,
        proc_root=proc_root,
        monotonic=clock.monotonic,
        monotonic_ns=clock.monotonic_ns,
        wall_time_ns=clock.wall_time_ns,
        sleep=clock.sleep,
    )

    document = monitor.run()

    assert document['status'] == 'PASS'
    assert document['sampling']['sample_count'] == 5
    assert document['sampling']['anchor_alive_sample_count'] == 4
    assert document['sampling']['totals']['threads'][0]['cpu_ticks'] == 15
    assert document['renderer']['post_start_segment_mode'] == 'appended'
    assert document['contact_profile']['selected_source_valid_record_count'] == 2
    assert document['contact_profile']['host_thread_bindings'][0]['linux_tid'] == 101
    assert document['full_stack_process']['group_confirmed_empty'] is True


def test_no_target_fails_without_launching_any_process(tmp_path: Path) -> None:
    config, _plugin, producer = _fixture_config(tmp_path)
    identity = _static_identity(config, producer)
    proc_root = tmp_path / 'proc'
    _host_proc(proc_root)
    clock = _Clock()
    monitor = profiler.SmokeHostProfiler(
        config,
        identity,
        proc_root=proc_root,
        monotonic=clock.monotonic,
        monotonic_ns=clock.monotonic_ns,
        wall_time_ns=clock.wall_time_ns,
        sleep=clock.sleep,
    )

    with pytest.raises(profiler.ProfileError) as captured:
        monitor.run()

    assert captured.value.kind == 'no_target'
    assert clock.seconds == config.startup_timeout_s


def test_profiler_rejects_preexisting_anchor_and_full_stack_outputs(tmp_path: Path) -> None:
    config, plugin, producer = _fixture_config(tmp_path)
    identity = _static_identity(config, producer)
    proc_root = tmp_path / 'proc'
    _host_proc(proc_root)
    (proc_root / 'uptime').write_text('1000.00 10.00\n', encoding='ascii')
    _make_process(proc_root, 101, plugin, start=100)
    clock = _Clock()
    monitor = profiler.SmokeHostProfiler(
        config,
        identity,
        proc_root=proc_root,
        monotonic=clock.monotonic,
        monotonic_ns=clock.monotonic_ns,
        wall_time_ns=clock.wall_time_ns,
        sleep=clock.sleep,
    )

    with pytest.raises(profiler.ProfileError, match='anchor predates profiler start'):
        monitor.run()

    shutil.rmtree(proc_root / '101')
    _full_stack_files(config, (_contact_record(),))
    with pytest.raises(profiler.ProfileError, match='output predates profiler start'):
        monitor.run()


def test_startup_deadline_is_checked_after_anchor_discovery(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    config, _plugin, producer = _fixture_config(tmp_path)
    identity = _static_identity(config, producer)
    proc_root = tmp_path / 'proc'
    _host_proc(proc_root)
    clock = _Clock()
    anchor = {'pid': 50, 'process_group': 50, 'session': 50, 'start_ticks': 100}

    def slow_discovery(*_arguments):
        clock.seconds = config.startup_timeout_s
        return anchor, 1

    monkeypatch.setattr(profiler, 'discover_anchor', slow_discovery)
    monitor = profiler.SmokeHostProfiler(
        config,
        identity,
        proc_root=proc_root,
        monotonic=clock.monotonic,
        monotonic_ns=clock.monotonic_ns,
        wall_time_ns=clock.wall_time_ns,
        sleep=clock.sleep,
    )

    with pytest.raises(profiler.ProfileError, match='discovery exceeded startup timeout'):
        monitor.run()


def test_slow_capture_rebases_cadence_without_catch_up(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    config, _plugin, producer = _fixture_config(tmp_path)
    config = profiler.ProfileConfig(**{**vars(config), 'max_duration_s': 10.0})
    identity = _static_identity(config, producer)
    proc_root = tmp_path / 'proc'
    _host_proc(proc_root)
    clock = _Clock()
    anchor = {'pid': 50, 'process_group': 50, 'session': 50, 'start_ticks': 100}
    sample_starts = []
    calls = {'count': 0}
    monkeypatch.setattr(profiler, '_renderer_baseline', lambda _path: {'present': False})
    monkeypatch.setattr(profiler, 'discover_anchor', lambda *_arguments: (anchor, 1))
    monkeypatch.setattr(
        profiler,
        'wait_for_full_stack_close',
        lambda *_arguments, **_keywords: {},
    )
    monkeypatch.setattr(profiler, 'collect_renderer', lambda *_arguments: {})
    monkeypatch.setattr(
        profiler,
        'parse_contact_profile_logs',
        lambda _run_dir: {'present': False},
    )

    def slow_capture(_proc, _config, _anchor, state, *_clocks):
        calls['count'] += 1
        sample_starts.append(clock.monotonic())
        clock.seconds += 1.0
        alive = calls['count'] < 4
        state.samples.append({'anchor_alive': alive, 'index': calls['count'] - 1})
        return alive

    monkeypatch.setattr(profiler, 'capture_sample', slow_capture)
    monitor = profiler.SmokeHostProfiler(
        config,
        identity,
        proc_root=proc_root,
        monotonic=clock.monotonic,
        monotonic_ns=clock.monotonic_ns,
        wall_time_ns=clock.wall_time_ns,
        sleep=clock.sleep,
    )

    with pytest.raises(profiler.ProfileError, match='canonical contact profile record is absent'):
        monitor.run()

    assert sample_starts == [0.0, 1.5, 3.0, 4.5]


def test_terminating_capture_cannot_finish_on_duration_deadline(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    config, _plugin, producer = _fixture_config(tmp_path)
    config = profiler.ProfileConfig(**{**vars(config), 'sample_period_s': 1.0})
    identity = _static_identity(config, producer)
    proc_root = tmp_path / 'proc'
    _host_proc(proc_root)
    clock = _Clock()
    anchor = {'pid': 50, 'process_group': 50, 'session': 50, 'start_ticks': 100}
    calls = {'count': 0}
    monkeypatch.setattr(profiler, '_renderer_baseline', lambda _path: {'present': False})
    monkeypatch.setattr(profiler, 'discover_anchor', lambda *_arguments: (anchor, 1))

    def boundary_capture(_proc, _config, _anchor, state, *_clocks):
        calls['count'] += 1
        alive = calls['count'] < 4
        state.samples.append({'anchor_alive': alive, 'index': calls['count'] - 1})
        if not alive:
            clock.seconds = config.max_duration_s
        return alive

    monkeypatch.setattr(profiler, 'capture_sample', boundary_capture)
    monitor = profiler.SmokeHostProfiler(
        config,
        identity,
        proc_root=proc_root,
        monotonic=clock.monotonic,
        monotonic_ns=clock.monotonic_ns,
        wall_time_ns=clock.wall_time_ns,
        sleep=clock.sleep,
    )

    with pytest.raises(profiler.ProfileError, match='sampling exceeded timeout'):
        monitor.run()


def test_authoritative_run_rejects_missing_contact_record(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    config, _plugin, producer = _fixture_config(tmp_path)
    identity = _static_identity(config, producer)
    clock = _Clock()
    proc_root = tmp_path / 'proc'
    _host_proc(proc_root)
    anchor = {'pid': 50, 'process_group': 50, 'session': 50, 'start_ticks': 100}
    monkeypatch.setattr(profiler, '_renderer_baseline', lambda _path: {'present': False})
    monkeypatch.setattr(profiler, 'discover_anchor', lambda *_arguments: (anchor, 1))
    monkeypatch.setattr(
        profiler,
        'wait_for_full_stack_close',
        lambda *_arguments, **_keywords: {},
    )
    monkeypatch.setattr(profiler, 'collect_renderer', lambda *_arguments: {})
    monkeypatch.setattr(
        profiler,
        'parse_contact_profile_logs',
        lambda _run_dir: {'present': False},
    )
    calls = {'count': 0}

    def fake_capture(_proc, _config, _anchor, state, *_clocks):
        calls['count'] += 1
        state.samples.append(
            {
                'anchor_alive': calls['count'] < 4,
                'index': calls['count'] - 1,
            }
        )
        return calls['count'] < 4

    monkeypatch.setattr(profiler, 'capture_sample', fake_capture)
    monitor = profiler.SmokeHostProfiler(
        config,
        identity,
        proc_root=proc_root,
        monotonic=clock.monotonic,
        monotonic_ns=clock.monotonic_ns,
        wall_time_ns=clock.wall_time_ns,
        sleep=clock.sleep,
    )

    with pytest.raises(profiler.ProfileError, match='canonical contact profile record is absent'):
        monitor.run()


def test_duration_deadline_clamps_sleep_and_forbids_boundary_sample(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    config, _plugin, producer = _fixture_config(tmp_path)
    config = profiler.ProfileConfig(**{**vars(config), 'sample_period_s': 2.0})
    identity = _static_identity(config, producer)
    clock = _Clock()
    sample_times = []
    proc_root = tmp_path / 'proc'
    _host_proc(proc_root)
    fake_anchor = {'pid': 1, 'start_ticks': 100}
    monkeypatch.setattr(
        profiler,
        'discover_anchor',
        lambda *_arguments: (fake_anchor, 1),
    )

    def fake_capture(_proc, _config, _anchor, state, *_clocks):
        sample_times.append(clock.monotonic())
        state.samples.append({'anchor_alive': True, 'index': len(state.samples)})
        return True

    monkeypatch.setattr(profiler, 'capture_sample', fake_capture)
    monkeypatch.setattr(profiler, '_renderer_baseline', lambda _path: {'present': False})
    monitor = profiler.SmokeHostProfiler(
        config,
        identity,
        proc_root=proc_root,
        monotonic=clock.monotonic,
        monotonic_ns=clock.monotonic_ns,
        wall_time_ns=clock.wall_time_ns,
        sleep=clock.sleep,
    )

    with pytest.raises(profiler.ProfileError, match='past timeout'):
        monitor.run()

    assert sample_times == [0.0, 2.0, 4.0]
    assert clock.seconds == 5.0
    assert max(clock.sleeps) <= 2.0


def test_write_profile_is_canonical_immutable_and_has_gnu_sidecar(tmp_path: Path) -> None:
    path = tmp_path / 'profile.json'
    document = {'schema_version': 1, 'status': 'PASS'}

    digest = profiler.write_profile(path, document)

    assert path.read_bytes() == profiler.canonical_json_bytes(document)
    assert Path(f'{path}.sha256').read_text(encoding='ascii') == f'{digest}  profile.json\n'
    with pytest.raises(profiler.ProfileError, match='replace'):
        profiler.write_profile(path, document)


def test_campaign_profile_validator_returns_only_portable_binding(tmp_path: Path) -> None:
    config, _document, profile_hash = _write_complete_campaign_profile_fixture(tmp_path)

    binding = profiler.validate_campaign_smoke_profile(
        config.workspace,
        config.candidate_root,
        config.candidate_id,
    )

    assert binding['candidate_id'] == config.candidate_id
    assert binding['profile_sha256'] == profile_hash
    assert binding['profile_relative_path'] == (
        f'artifacts/evidence/phase3/performance-profiles/{config.candidate_id}-smoke-profile.json'
    )
    assert not any(key in binding for key in ('device', 'inode', 'mtime_ns', 'path'))
    assert not (config.candidate_root / 'smoke/result/run-result.json.sha256').exists()


def test_campaign_profile_validator_accepts_modern_cpu_pressure_rows(
    tmp_path: Path,
) -> None:
    config, original, _profile_hash = _write_complete_campaign_profile_fixture(tmp_path)
    document = copy.deepcopy(original)
    for sample in document['sampling']['samples']:
        sample['host']['pressure']['cpu']['full'] = {
            'avg10': 0.0,
            'avg60': 0.0,
            'avg300': 0.0,
            'total': 0,
        }
    _rewrite_profile(config, document)

    profiler.validate_campaign_smoke_profile(
        config.workspace,
        config.candidate_root,
        config.candidate_id,
    )


def test_campaign_profile_validator_accepts_lossless_anchor_cmdline_padding(
    tmp_path: Path,
) -> None:
    config, original, _profile_hash = _write_complete_campaign_profile_fixture(tmp_path)
    document = copy.deepcopy(original)
    anchor = document['anchor']
    lifecycle = next(
        item for item in document['process_lifecycles'] if item['pid'] == anchor['pid']
    )
    previous_size = lifecycle['cmdline_size_bytes']
    padded_cmdline = [*lifecycle['cmdline'], '', '']
    lifecycle['cmdline'] = padded_cmdline
    encoded = b'\0'.join(item.encode() for item in lifecycle['cmdline']) + b'\0'
    lifecycle['cmdline_size_bytes'] = len(encoded)
    lifecycle['cmdline_sha256'] = __import__('hashlib').sha256(encoded).hexdigest()
    anchor['cmdline'] = list(padded_cmdline)
    anchor['cmdline_size_bytes'] = lifecycle['cmdline_size_bytes']
    anchor['cmdline_sha256'] = lifecycle['cmdline_sha256']
    document['sampling']['retained_cmdline_bytes'] += len(encoded) - previous_size
    _rewrite_profile(config, document)

    profiler.validate_campaign_smoke_profile(
        config.workspace,
        config.candidate_root,
        config.candidate_id,
    )


def test_campaign_profile_validator_accepts_delayed_anchor_and_intact_clone(
    tmp_path: Path,
) -> None:
    config, original, _profile_hash = _write_complete_campaign_profile_fixture(tmp_path)
    delayed = copy.deepcopy(original)
    delay_ns = 30_000_000_000
    for sample in delayed['sampling']['samples']:
        sample['monotonic_ns'] += delay_ns
    metadata_path = config.candidate_root / 'smoke/processes/full_stack.process.json'
    metadata = __import__('json').loads(metadata_path.read_text(encoding='utf-8'))
    metadata['finished_steady_ns'] += delay_ns
    metadata_payload = profiler.canonical_json_bytes(metadata)
    metadata_path.write_bytes(metadata_payload)
    delayed['full_stack_process']['finished_steady_ns'] = metadata['finished_steady_ns']
    delayed['full_stack_process']['sha256'] = (
        __import__('hashlib').sha256(metadata_payload).hexdigest()
    )
    _rewrite_profile(config, delayed)

    profiler.validate_campaign_smoke_profile(
        config.workspace, config.candidate_root, config.candidate_id
    )

    clone_workspace = tmp_path / 'relocated-workspace'
    shutil.copytree(config.workspace, clone_workspace)
    clone_root = clone_workspace / 'artifacts/evidence/phase3-benchmarks'
    clone_candidate = clone_root / config.candidate_id
    cloned = profiler.validate_campaign_smoke_profile(
        clone_workspace, clone_candidate, config.candidate_id
    )
    assert (
        cloned['profile_sha256']
        == __import__('hashlib').sha256(config.output_path.read_bytes()).hexdigest()
    )


def test_campaign_profile_validator_rejects_self_consistent_unbounded_lifecycles(
    tmp_path: Path,
) -> None:
    config, original, _profile_hash = _write_complete_campaign_profile_fixture(tmp_path)
    document = copy.deepcopy(original)
    metadata_path = config.candidate_root / 'smoke/processes/full_stack.process.json'
    metadata = __import__('json').loads(metadata_path.read_text(encoding='utf-8'))
    metadata['finished_steady_ns'] = (
        int(
            (profiler.CANONICAL_MAX_DURATION_S + profiler.FULL_STACK_CLOSE_TIMEOUT_S)
            * 1_000_000_000
        )
        + 1
    )
    metadata_payload = profiler.canonical_json_bytes(metadata)
    metadata_path.write_bytes(metadata_payload)
    document['full_stack_process']['finished_steady_ns'] = metadata['finished_steady_ns']
    document['full_stack_process']['sha256'] = (
        __import__('hashlib').sha256(metadata_payload).hexdigest()
    )
    _rewrite_profile(config, document)

    with pytest.raises(profiler.ProfileError, match='lifecycle exceeds profile bounds'):
        profiler.validate_campaign_smoke_profile(
            config.workspace, config.candidate_root, config.candidate_id
        )


def test_campaign_profile_validator_requires_sampled_full_stack_group_leader(
    tmp_path: Path,
) -> None:
    config, original, _profile_hash = _write_complete_campaign_profile_fixture(tmp_path)
    document = copy.deepcopy(original)
    leader = next(item for item in document['process_lifecycles'] if item['pid'] == 50)
    document['process_lifecycles'].remove(leader)
    document['sampling']['retained_cmdline_bytes'] -= leader['cmdline_size_bytes']
    removed_threads = 0
    for sample in document['sampling']['samples']:
        removed = [item for item in sample['processes'] if item['pid'] == 50]
        removed_threads += sum(item['thread_count'] for item in removed)
        sample['processes'] = [item for item in sample['processes'] if item['pid'] != 50]
        sample['process_count'] = len(sample['processes'])
    document['sampling']['thread_record_count'] -= removed_threads
    document['sampling']['totals']['processes'] = [
        item for item in document['sampling']['totals']['processes'] if item['pid'] != 50
    ]
    document['sampling']['totals']['threads'] = [
        item for item in document['sampling']['totals']['threads'] if item['pid'] != 50
    ]
    _rewrite_profile(config, document)

    with pytest.raises(profiler.ProfileError, match='group leader lacks'):
        profiler.validate_campaign_smoke_profile(
            config.workspace, config.candidate_root, config.candidate_id
        )


@pytest.mark.parametrize('flag', ('anchor_alive', 'target_process_set_alive'))
def test_campaign_profile_validator_rejects_nonproducer_lifecycle_histories(
    tmp_path: Path,
    flag: str,
) -> None:
    config, original, _profile_hash = _write_complete_campaign_profile_fixture(tmp_path)
    document = copy.deepcopy(original)
    document['sampling']['samples'][0][flag] = False
    summary = {
        'anchor_alive': 'anchor_alive_sample_count',
        'target_process_set_alive': 'target_alive_sample_count',
    }[flag]
    document['sampling'][summary] -= 1
    _rewrite_profile(config, document)

    with pytest.raises(profiler.ProfileError):
        profiler.validate_campaign_smoke_profile(
            config.workspace, config.candidate_root, config.candidate_id
        )


@pytest.mark.parametrize(
    'mutation',
    (
        'fail_status',
        'bool_schema',
        'extra_top_key',
        'missing_top_key',
        'wrong_candidate',
        'wrong_build_hash',
        'wrong_smoke_partition',
        'wrong_profiler_hash',
        'bool_sample_count',
        'bool_processor',
        'bad_contact_total',
        'bad_lifecycle_end',
        'anchor_predates_profile',
        'unbounded_sample_clock',
        'finish_precedes_samples',
    ),
)
def test_campaign_profile_validator_rejects_hostile_document_mutations(
    tmp_path: Path,
    mutation: str,
) -> None:
    config, original, _profile_hash = _write_complete_campaign_profile_fixture(tmp_path)
    document = copy.deepcopy(original)
    if mutation == 'fail_status':
        document['status'] = 'FAIL'
    elif mutation == 'bool_schema':
        document['schema_version'] = True
    elif mutation == 'extra_top_key':
        document['unexpected'] = None
    elif mutation == 'missing_top_key':
        document.pop('renderer')
    elif mutation == 'wrong_candidate':
        document['candidate']['candidate_id'] = 'phase3-wrong-001'
    elif mutation == 'wrong_build_hash':
        document['candidate']['build_binding']['sha256'] = '0' * 64
    elif mutation == 'wrong_smoke_partition':
        document['candidate']['smoke']['gz_partition'] = 'wrong_partition'
    elif mutation == 'wrong_profiler_hash':
        document['candidate']['profiler_producer']['sha256'] = '0' * 64
    elif mutation == 'bool_sample_count':
        document['sampling']['sample_count'] = True
    elif mutation == 'bool_processor':
        document['sampling']['samples'][0]['processes'][0]['threads'][0]['processor'] = True
    elif mutation == 'bad_contact_total':
        document['contact_profile']['last_valid_cumulative_record']['measured_total_ns'] += 1
    elif mutation == 'bad_lifecycle_end':
        document['process_lifecycles'][0]['ended_sample_index'] -= 1
    elif mutation == 'anchor_predates_profile':
        document['anchor']['start_ticks'] = 0
    elif mutation == 'unbounded_sample_clock':
        document['sampling']['samples'][-1]['monotonic_ns'] = (
            document['profile_started_monotonic_ns']
            + int(profiler.CANONICAL_MAX_DURATION_S * 1_000_000_000)
            + 1
        )
    elif mutation == 'finish_precedes_samples':
        document['finished_utc'] = document['profile_started_utc']
    else:  # pragma: no cover - exhaustive test-table guard
        raise AssertionError(mutation)
    _rewrite_profile(config, document)

    with pytest.raises(profiler.ProfileError):
        profiler.validate_campaign_smoke_profile(
            config.workspace,
            config.candidate_root,
            config.candidate_id,
        )


def test_campaign_profile_validator_requires_evidence_and_exact_sidecar(
    tmp_path: Path,
) -> None:
    config, _document, _profile_hash = _write_complete_campaign_profile_fixture(tmp_path)
    config.output_path.unlink()
    Path(f'{config.output_path}.sha256').unlink()
    with pytest.raises(profiler.ProfileError):
        profiler.validate_campaign_smoke_profile(
            config.workspace, config.candidate_root, config.candidate_id
        )

    config, _document, _profile_hash = _write_complete_campaign_profile_fixture(
        tmp_path / 'mismatch'
    )
    Path(f'{config.output_path}.sha256').write_text(
        f'{"0" * 64}  {config.output_path.name}\n', encoding='ascii'
    )
    with pytest.raises(profiler.ProfileError, match='sidecar mismatch'):
        profiler.validate_campaign_smoke_profile(
            config.workspace, config.candidate_root, config.candidate_id
        )


def test_campaign_profile_validator_rejects_noncanonical_or_symlinked_paths(
    tmp_path: Path,
) -> None:
    config, document, _profile_hash = _write_complete_campaign_profile_fixture(tmp_path)
    payload = __import__('json').dumps(document, indent=2).encode() + b'\n'
    config.output_path.write_bytes(payload)
    digest = __import__('hashlib').sha256(payload).hexdigest()
    Path(f'{config.output_path}.sha256').write_text(
        f'{digest}  {config.output_path.name}\n', encoding='ascii'
    )
    with pytest.raises(profiler.ProfileError, match='not canonical'):
        profiler.validate_campaign_smoke_profile(
            config.workspace, config.candidate_root, config.candidate_id
        )

    config, _document, _profile_hash = _write_complete_campaign_profile_fixture(
        tmp_path / 'symlink'
    )
    real_smoke = config.candidate_root / 'real-smoke'
    (config.candidate_root / 'smoke').rename(real_smoke)
    (config.candidate_root / 'smoke').symlink_to(real_smoke, target_is_directory=True)
    with pytest.raises(profiler.ProfileError, match='real directory'):
        profiler.validate_campaign_smoke_profile(
            config.workspace, config.candidate_root, config.candidate_id
        )


@pytest.mark.parametrize('source', ('plugin', 'profiler'))
def test_campaign_profile_validator_rehashes_clone_local_sources(
    tmp_path: Path,
    source: str,
) -> None:
    config, _document, _profile_hash = _write_complete_campaign_profile_fixture(tmp_path)
    if source == 'plugin':
        path = config.workspace / 'build/robotest_sim/librobotest_contact_aggregator_system.so'
    else:
        path = config.workspace / 'tests/phase3_smoke_host_profiler.py'
    path.write_bytes(path.read_bytes() + b'changed')

    with pytest.raises(profiler.ProfileError, match='bytes differ'):
        profiler.validate_campaign_smoke_profile(
            config.workspace, config.candidate_root, config.candidate_id
        )


def test_campaign_profile_validator_requires_exact_successful_smoke(tmp_path: Path) -> None:
    config, _document, _profile_hash = _write_complete_campaign_profile_fixture(tmp_path)
    result_path = config.candidate_root / 'smoke/result/run-result.json'
    result = __import__('json').loads(result_path.read_text(encoding='utf-8'))
    result['verdict']['automated_status'] = 'FAIL'
    result['verdict']['exit_code'] = 30
    result_hash = _rewrite_result_bundle(result_path, result)
    _canonical(
        config.candidate_root / 'smoke/PASS.json',
        {
            'producer': profiler.BENCHMARK_PRODUCER,
            'run_result_sha256': result_hash,
            'status': 'PASS',
        },
    )

    with pytest.raises(profiler.ProfileError, match='not a complete PASS'):
        profiler.validate_campaign_smoke_profile(
            config.workspace, config.candidate_root, config.candidate_id
        )


def test_campaign_profile_validator_replays_result_schema_and_csv(tmp_path: Path) -> None:
    config, _document, _profile_hash = _write_complete_campaign_profile_fixture(tmp_path)
    result_path = config.candidate_root / 'smoke/result/run-result.json'
    result = __import__('json').loads(result_path.read_text(encoding='utf-8'))
    result['unexpected'] = None
    result_hash = _rewrite_result_bundle(result_path, result)
    _canonical(
        config.candidate_root / 'smoke/PASS.json',
        {
            'producer': profiler.BENCHMARK_PRODUCER,
            'run_result_sha256': result_hash,
            'status': 'PASS',
        },
    )
    with pytest.raises(profiler.ProfileError, match='run-result schema failed'):
        profiler.validate_campaign_smoke_profile(
            config.workspace, config.candidate_root, config.candidate_id
        )

    config, _document, _profile_hash = _write_complete_campaign_profile_fixture(tmp_path / 'csv')
    result_path = config.candidate_root / 'smoke/result/run-result.json'
    result_hash = __import__('hashlib').sha256(result_path.read_bytes()).hexdigest()
    (result_path.parent / 'run-result.csv').write_bytes(b'not,the,canonical,projection\n')
    _rewrite_result_manifest(result_path, result_hash)
    with pytest.raises(profiler.ProfileError, match='CSV projection differs'):
        profiler.validate_campaign_smoke_profile(
            config.workspace, config.candidate_root, config.candidate_id
        )


def test_phase3_run_csv_mirror_uses_bounded_scalar_summary_contract() -> None:
    document = {
        'events': [{'kind': 'one'}, {'kind': 'two'}],
        'measurements': {'collision_count': 0},
    }

    rows = list(csv.DictReader(io.StringIO(profiler._one_row_csv_bytes(document).decode())))

    assert len(rows) == 1
    row = rows[0]
    assert row[profiler.CSV_PROJECTION_CONTRACT_COLUMN] == profiler.CSV_PROJECTION_CONTRACT
    assert (
        row[profiler.CSV_PROJECTION_JSON_SHA256_COLUMN]
        == __import__('hashlib').sha256(profiler.canonical_json_bytes(document)).hexdigest()
    )
    assert json.loads(row['events']) == {
        'element_count': 2,
        'kind': 'sequence',
        'sha256': __import__('hashlib')
        .sha256(profiler.canonical_json_bytes(document['events']))
        .hexdigest(),
    }
    assert row['measurements.collision_count'] == '0'


def test_phase3_run_csv_mirror_matches_clone_local_production_bytes() -> None:
    document = {
        'events': [{'kind': 'one'}, {'kind': 'two'}],
        'measurements': {'collision_count': 0, 'optional': None},
    }
    metrics_source = TEST_DIR.parent / 'src/robotest_metrics'
    environment = dict(os.environ)
    environment['PYTHONPATH'] = str(metrics_source)
    completed = subprocess.run(
        [
            sys.executable,
            '-c',
            (
                'import json,sys; '
                'from robotest_metrics.artifacts import one_row_csv_bytes; '
                'sys.stdout.buffer.write(one_row_csv_bytes(json.load(sys.stdin)))'
            ),
        ],
        input=profiler.canonical_json_bytes(document),
        capture_output=True,
        check=True,
        env=environment,
    )

    assert profiler._one_row_csv_bytes(document) == completed.stdout


def test_phase3_run_csv_mirror_bounds_large_sequence_payloads() -> None:
    document = {
        'events': [{'payload': 'e' * 600, 'stamp_ns': index} for index in range(1_500)],
        'measurements': {
            'samples': [{'payload': 's' * 600, 'stamp_ns': index} for index in range(1_500)]
        },
    }

    assert len(profiler.canonical_json_bytes(document)) > 1_048_576
    assert len(profiler._one_row_csv_bytes(document)) < 1_048_576


@pytest.mark.parametrize(
    'reserved_name',
    [
        profiler.CSV_PROJECTION_CONTRACT_COLUMN,
        profiler.CSV_PROJECTION_JSON_SHA256_COLUMN,
    ],
)
def test_phase3_run_csv_mirror_rejects_reserved_root_columns(
    reserved_name: str,
) -> None:
    with pytest.raises(profiler.ProfileError, match='reserved column'):
        profiler._one_row_csv_bytes({reserved_name: 'forged'})


def test_phase3_run_csv_mirror_rejects_dotted_column_collision() -> None:
    with pytest.raises(profiler.ProfileError, match='column collision'):
        profiler._one_row_csv_bytes({'a': {'b': []}, 'a.b': []})


def test_metrics_result_caps_match_the_clone_local_production_contract(tmp_path: Path) -> None:
    config, _plugin, _producer = _fixture_config(tmp_path)

    assert profiler._metrics_result_caps(config.workspace) == {
        'LOG_MAX_BYTES': 8_388_608,
        'PER_RUN_CSV_MAX_BYTES': 1_048_576,
        'PER_RUN_DIRECTORY_MAX_BYTES': 268_435_456,
        'PER_RUN_JSON_MAX_BYTES': 33_554_432,
        'PNG_MAX_BYTES': 4_194_304,
        'PNG_MAX_COUNT': 8,
    }


@pytest.mark.parametrize(
    ('artifact_name', 'maximum_bytes'),
    (
        ('run-result.csv', 1_048_576),
        ('report.md', 8_388_608),
        ('trajectory.png', 4_194_304),
    ),
)
def test_campaign_profile_validator_rejects_result_artifact_overflow_at_production_cap(
    tmp_path: Path,
    artifact_name: str,
    maximum_bytes: int,
) -> None:
    config, _document, _profile_hash = _write_complete_campaign_profile_fixture(tmp_path)
    artifact = config.candidate_root / 'smoke/result' / artifact_name
    artifact.write_bytes(b'x' * (maximum_bytes + 1))

    with pytest.raises(profiler.ProfileError, match=f'exceeds {maximum_bytes} bytes'):
        profiler.validate_campaign_smoke_profile(
            config.workspace, config.candidate_root, config.candidate_id
        )


def test_validated_config_freezes_ignored_output_outside_candidate(tmp_path: Path) -> None:
    config, _plugin, _producer = _fixture_config(tmp_path)

    frozen = profiler.validated_config(_config_arguments(config))

    assert frozen.output_path == (
        config.workspace
        / 'artifacts/evidence/phase3/performance-profiles'
        / f'{config.candidate_id}-smoke-profile.json'
    )
    assert not frozen.output_path.is_relative_to(config.candidate_root)


@pytest.mark.parametrize(
    ('field', 'value'),
    (
        ('startup_timeout_s', 179.0),
        ('sample_period_s', 0.6),
        ('max_duration_s', 479.0),
    ),
)
def test_validated_config_rejects_noncanonical_timing(
    tmp_path: Path, field: str, value: float
) -> None:
    config, _plugin, _producer = _fixture_config(tmp_path)
    arguments = _config_arguments(config)
    setattr(arguments, field, value)

    with pytest.raises(profiler.ProfileError, match='canonical value'):
        profiler.validated_config(arguments)


def test_thread_and_output_caps_cover_canonical_512_thread_duration() -> None:
    maximum_captures = (
        __import__('math').ceil(
            profiler.CANONICAL_MAX_DURATION_S / profiler.CANONICAL_SAMPLE_PERIOD_S
        )
        + 1
    )

    assert maximum_captures * profiler.MAX_LIVE_THREADS_PER_SAMPLE <= (profiler.MAX_THREAD_RECORDS)
    assert profiler.MAX_THREAD_RECORDS * 512 <= profiler.OUTPUT_MAX_BYTES


def test_failure_document_records_effective_limits(tmp_path: Path) -> None:
    config, _plugin, _producer = _fixture_config(tmp_path)
    document = profiler._failure_document(
        config,
        profiler.ProfileError('overflow', 'fixture overflow'),
        None,
        None,
    )

    assert document['limits'] == profiler._profile_limits(config)
    assert document['limits']['maximum_live_threads_per_sample'] == 512
    assert document['limits']['maximum_thread_records'] == 524_288
    assert document['limits']['maximum_output_bytes'] == 256 * 1024 * 1024


@pytest.mark.parametrize(
    ('leaf_name', 'leaf_kind'),
    (
        ('output', 'regular'),
        ('output', 'direct_symlink'),
        ('output', 'broken_symlink'),
        ('sidecar', 'regular'),
        ('sidecar', 'broken_symlink'),
    ),
)
def test_validated_config_rejects_existing_or_symlinked_output_leaves(
    tmp_path: Path, leaf_name: str, leaf_kind: str
) -> None:
    config, _plugin, _producer = _fixture_config(tmp_path)
    output = config.output_path
    output.parent.mkdir(parents=True)
    leaf = output if leaf_name == 'output' else Path(f'{output}.sha256')
    if leaf_kind == 'regular':
        leaf.write_bytes(b'existing evidence')
    else:
        target = output.parent / f'{leaf.name}.{leaf_kind}.target'
        if leaf_kind == 'direct_symlink':
            target.write_bytes(b'direct target')
        leaf.symlink_to(target)

    with pytest.raises(profiler.ProfileError) as captured:
        profiler.validated_config(_config_arguments(config))

    assert captured.value.kind == 'output_exists'


def test_validated_config_rejects_symlinked_output_parent(tmp_path: Path) -> None:
    config, _plugin, _producer = _fixture_config(tmp_path)
    phase3 = config.workspace / 'artifacts/evidence/phase3'
    phase3.mkdir(parents=True)
    alternate = config.workspace / 'alternate-performance-profiles'
    alternate.mkdir()
    (phase3 / 'performance-profiles').symlink_to(alternate, target_is_directory=True)

    with pytest.raises(profiler.ProfileError, match='output parent is a symlink'):
        profiler.validated_config(_config_arguments(config))
