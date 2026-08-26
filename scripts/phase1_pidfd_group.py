#!/usr/bin/env python3
# Copyright 2026 Hasan Ahmed
# SPDX-License-Identifier: Apache-2.0

"""Validate and signal one Phase 1 process group through a stable procfd."""

from __future__ import annotations

import argparse
import errno
import hashlib
import json
import os
import signal
import stat
import sys
import time
from collections.abc import Callable
from dataclasses import asdict, dataclass

PIDFD_SIGNAL_PROCESS_GROUP = 1 << 2
PIDFD_GROUP_EMPTY_EXIT_CODE = 20
GROUP_TOKEN_ENVIRONMENT = 'ROBOTEST_PHASE1_GROUP_TOKEN'
VALIDATION_ATTEMPTS = 100
VALIDATION_INTERVAL_SECONDS = 0.01


class PidfdGroupError(RuntimeError):
    """Raised when stable process-group identity or signaling is not proven."""


@dataclass(frozen=True)
class ProcessIdentity:
    """Identity fields read relative to the retained proc directory fd."""

    pid: int
    command: str
    state: str
    parent_pid: int
    process_group_id: int
    session_id: int
    start_ticks: int


def canonical_json(value: object) -> str:
    """Return one deterministic JSON record."""

    return json.dumps(value, separators=(',', ':'), sort_keys=True) + '\n'


def parse_proc_stat(text: str) -> ProcessIdentity:
    """Parse Linux /proc/PID/stat without assuming the command has no spaces."""

    open_parenthesis = text.find('(')
    close_parenthesis = text.rfind(') ')
    if open_parenthesis <= 0 or close_parenthesis <= open_parenthesis:
        raise PidfdGroupError('malformed proc stat record')
    try:
        pid = int(text[:open_parenthesis].strip())
    except ValueError as error:
        raise PidfdGroupError('invalid proc stat PID') from error
    command = text[open_parenthesis + 1 : close_parenthesis]
    fields = text[close_parenthesis + 2 :].split()
    if len(fields) < 20:
        raise PidfdGroupError('truncated proc stat record')
    try:
        return ProcessIdentity(
            pid=pid,
            command=command,
            state=fields[0],
            parent_pid=int(fields[1]),
            process_group_id=int(fields[2]),
            session_id=int(fields[3]),
            start_ticks=int(fields[19]),
        )
    except ValueError as error:
        raise PidfdGroupError('non-integer proc identity field') from error


def read_procfd_file(fd: int, name: str, maximum_bytes: int) -> bytes:
    """Read a bounded regular proc file relative to an already-open procfd."""

    child_fd = os.open(name, os.O_RDONLY | os.O_CLOEXEC, dir_fd=fd)
    try:
        chunks: list[bytes] = []
        remaining = maximum_bytes + 1
        while remaining > 0:
            chunk = os.read(child_fd, min(remaining, 65536))
            if not chunk:
                break
            chunks.append(chunk)
            remaining -= len(chunk)
        content = b''.join(chunks)
    finally:
        os.close(child_fd)
    if len(content) > maximum_bytes:
        raise PidfdGroupError(f'proc {name} exceeds the bounded read limit')
    return content


def read_procfd_identity(fd: int) -> ProcessIdentity:
    """Read the identity pinned by the retained proc directory descriptor."""

    try:
        content = read_procfd_file(fd, 'stat', 16384)
        return parse_proc_stat(content.decode('utf-8', errors='strict'))
    except (OSError, UnicodeError) as error:
        raise PidfdGroupError(f'cannot read stable proc identity: {error}') from error


def read_procfd_environment(fd: int) -> frozenset[bytes]:
    """Read exact NUL-delimited environment entries from the pinned process."""

    try:
        content = read_procfd_file(fd, 'environ', 1024 * 1024)
    except OSError as error:
        raise PidfdGroupError(f'cannot read stable proc environment: {error}') from error
    return frozenset(item for item in content.split(b'\0') if item)


def identity_matches(
    identity: ProcessIdentity,
    *,
    expected_pid: int,
    expected_parent_pid: int,
) -> bool:
    """Return whether a process is the expected session and group leader."""

    return (
        identity.pid == expected_pid
        and identity.parent_pid == expected_parent_pid
        and identity.process_group_id == expected_pid
        and identity.session_id == expected_pid
        and identity.start_ticks > 0
        and identity.state != 'Z'
    )


def validate_identity_pair(
    initial: ProcessIdentity,
    closing: ProcessIdentity,
    *,
    expected_pid: int,
    expected_parent_pid: int,
) -> None:
    """Reject acquisition if identity changed across the stable-fd proof."""

    if not identity_matches(
        initial,
        expected_pid=expected_pid,
        expected_parent_pid=expected_parent_pid,
    ):
        raise PidfdGroupError('procfd is not the expected live session/group leader')
    if closing != initial:
        raise PidfdGroupError('procfd identity changed during acquisition validation')


def group_signal_status(
    fd: int,
    signal_number: int,
    *,
    sender: Callable[[int, int, None, int], None] = signal.pidfd_send_signal,
) -> str:
    """Signal/probe the fd-pinned group and classify only ESRCH as empty."""

    try:
        sender(fd, signal_number, None, PIDFD_SIGNAL_PROCESS_GROUP)
    except ProcessLookupError:
        return 'empty'
    except OSError as error:
        if error.errno == errno.ESRCH:
            return 'empty'
        raise PidfdGroupError(
            f'pidfd group signal {signal_number} failed: errno={error.errno}'
        ) from error
    return 'present' if signal_number == 0 else 'sent'


def validate_procfd(
    fd: int,
    *,
    expected_pid: int,
    expected_parent_pid: int,
    expected_token: str,
    attempts: int = VALIDATION_ATTEMPTS,
    interval_seconds: float = VALIDATION_INTERVAL_SECONDS,
) -> ProcessIdentity:
    """Prove stable leader identity, nonce, and group-signal capability."""

    if fd < 3 or expected_pid < 1 or expected_parent_pid < 1 or not expected_token:
        raise PidfdGroupError('invalid procfd validation input')
    try:
        descriptor_stat = os.fstat(fd)
    except OSError as error:
        raise PidfdGroupError(f'cannot stat procfd: {error}') from error
    if not stat.S_ISDIR(descriptor_stat.st_mode):
        raise PidfdGroupError('stable handle is not a proc directory descriptor')

    expected_environment = f'{GROUP_TOKEN_ENVIRONMENT}={expected_token}'.encode(
        'utf-8', errors='strict'
    )
    last_error = 'stable identity was not ready'
    for attempt in range(attempts):
        try:
            initial = read_procfd_identity(fd)
            environment = read_procfd_environment(fd)
            closing = read_procfd_identity(fd)
            validate_identity_pair(
                initial,
                closing,
                expected_pid=expected_pid,
                expected_parent_pid=expected_parent_pid,
            )
            if expected_environment not in environment:
                raise PidfdGroupError('launch group token is absent from stable procfd')
            if group_signal_status(fd, 0) != 'present':
                raise PidfdGroupError('stable process group disappeared during validation')
            return closing
        except PidfdGroupError as error:
            last_error = str(error)
        if attempt + 1 < attempts:
            time.sleep(interval_seconds)
    raise PidfdGroupError(last_error)


def validation_result(identity: ProcessIdentity, fd: int, token: str) -> dict[str, object]:
    """Build non-secret acquisition evidence."""

    return {
        'fd': fd,
        'group_signal_flag': PIDFD_SIGNAL_PROCESS_GROUP,
        'identity': asdict(identity),
        'status': 'validated',
        'token_sha256': hashlib.sha256(token.encode('utf-8')).hexdigest(),
    }


def error_result(command: str, error: BaseException) -> dict[str, object]:
    """Build deterministic fail-closed CLI evidence."""

    error_number = error.errno if isinstance(error, OSError) else None
    return {
        'command': command,
        'errno': error_number,
        'error': str(error),
        'group_signal_flag': PIDFD_SIGNAL_PROCESS_GROUP,
        'status': 'error',
    }


def emit_result(result: dict[str, object], exit_code: int) -> int:
    """Emit best-effort evidence without changing the action exit protocol."""

    try:
        sys.stdout.write(canonical_json(result))
        sys.stdout.flush()
    except OSError:
        pass
    return exit_code


def build_parser() -> argparse.ArgumentParser:
    """Build the explicit command interface."""

    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest='command', required=True)

    validate = subparsers.add_parser('validate')
    validate.add_argument('--fd', required=True, type=int)
    validate.add_argument('--parent-pid', required=True, type=int)
    validate.add_argument('--pid', required=True, type=int)
    validate.add_argument('--token', required=True)

    probe = subparsers.add_parser('probe')
    probe.add_argument('--fd', required=True, type=int)

    send = subparsers.add_parser('send')
    send.add_argument('--fd', required=True, type=int)
    send.add_argument('--signal', choices=('KILL', 'TERM'), required=True)
    return parser


def main(argv: list[str] | None = None) -> int:
    """Run one fail-closed validation, probe, or atomic group signal."""

    arguments = build_parser().parse_args(argv)
    try:
        if arguments.command == 'validate':
            identity = validate_procfd(
                arguments.fd,
                expected_pid=arguments.pid,
                expected_parent_pid=arguments.parent_pid,
                expected_token=arguments.token,
            )
            result = validation_result(identity, arguments.fd, arguments.token)
        elif arguments.command == 'probe':
            result = {
                'group_signal_flag': PIDFD_SIGNAL_PROCESS_GROUP,
                'status': group_signal_status(arguments.fd, 0),
            }
        else:
            signal_number = getattr(signal, f'SIG{arguments.signal}')
            result = {
                'group_signal_flag': PIDFD_SIGNAL_PROCESS_GROUP,
                'signal': arguments.signal,
                'status': group_signal_status(arguments.fd, signal_number),
            }
    except (OSError, PidfdGroupError) as error:
        return emit_result(error_result(arguments.command, error), 3)
    action_exit_code = (
        PIDFD_GROUP_EMPTY_EXIT_CODE
        if arguments.command in {'probe', 'send'} and result['status'] == 'empty'
        else 0
    )
    return emit_result(result, action_exit_code)


if __name__ == '__main__':
    raise SystemExit(main())
