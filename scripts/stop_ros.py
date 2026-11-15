#!/usr/bin/env python3
"""Stop the current user's ROS/Gazebo stack in this Linux PID namespace.

This stops matching active processes too: Linux cannot identify stale ROS nodes
reliably. Run inside the dev container, from a separate terminal. No ROS setup or
third-party Python packages are needed. Files, databases and shared memory are
left intact.
"""

import argparse
from dataclasses import dataclass
import os
from pathlib import Path
import re
import shlex
import signal
import sys
import time


WORKSPACE = Path(__file__).resolve().parents[1]
PROC = Path('/proc')


@dataclass(frozen=True)
class Process:
    pid: int
    parent: int
    started: str
    argv: tuple
    cwd: str
    state: str


def read_process(pid):
    try:
        directory = PROC / str(pid)
        if directory.stat().st_uid != os.getuid():
            return None
        # comm is parenthesized and may itself contain spaces or parentheses.
        fields = (directory / 'stat').read_text().rsplit(')', 1)[1].split()
        argv = tuple(os.fsdecode(arg) for arg in (directory / 'cmdline').read_bytes()
                     .split(b'\0') if arg)
        try:
            cwd = os.readlink(directory / 'cwd')
        except OSError:
            cwd = ''
        return Process(pid, int(fields[1]), fields[19], argv, cwd, fields[0])
    except (OSError, ValueError, IndexError):
        return None


def processes():
    result = {}
    for directory in PROC.iterdir():
        if directory.name.isdecimal():
            process = read_process(int(directory.name))
            if process is not None:
                result[process.pid] = process
    return result


def protected_pids(snapshot):
    # Never signal this script, its terminal/shell, or container init.
    protected = {1}
    pid = os.getpid()
    while pid and pid not in protected:
        protected.add(pid)
        process = snapshot.get(pid)
        pid = process.parent if process else 0
    return protected


def executable(process):
    if not process.argv:
        return ''
    program = process.argv[0]
    name = Path(program).name
    if re.fullmatch(r'(python[\d.]*|ruby[\d.]*|bash|sh)', name):
        # Inspect an interpreter's script, never arbitrary shell command text.
        for arg in process.argv[1:]:
            if arg in ('-c', '-m', '--command'):
                return ''
            if not arg.startswith('-'):
                return arg
        return ''
    return program


def matches(process, include_zenoh=False):
    program = executable(process)
    name = Path(program).name
    if name in ('rmw_zenohd', 'zenohd'):
        return include_zenoh
    if name == 'ros2':
        arguments = process.argv[process.argv.index(program) + 1:]
        if any(arg in ('rmw_zenohd', 'zenohd') for arg in arguments):
            return include_zenoh
        return bool(arguments) and arguments[0] in ('launch', 'run', 'daemon')
    if name in ('_ros2_daemon', 'rviz2', 'rtabmap', 'rtabmapviz',
                'gzserver', 'gzclient', 'gz-sim-server', 'gz-sim-gui'):
        return True
    if name in ('gz', 'ign'):
        return any(arg in ('sim', 'gazebo') for arg in process.argv[1:])
    path = Path(program)
    if program and not path.is_absolute() and process.cwd:
        path = Path(process.cwd) / path
    path = Path(os.path.abspath(path))
    path_text = str(path)
    # Installed ROS executables, including Python nodes and component containers.
    if re.match(r'^/opt/ros/[^/]+/lib/[^/]+/[^/]+$', path_text):
        return True
    if path_text.startswith(str(WORKSPACE / 'install') + '/') and '/lib/' in path_text:
        return True
    if path in (WORKSPACE / 'deploy.sh', WORKSPACE / 'deployWithoutBuild.sh',
                WORKSPACE / 'gazebo_deploy.sh'):
        return True
    # ros2cli may spawn its daemon through Python -c instead of a script.
    return (Path(process.argv[0]).name.startswith('python') and
            '-c' in process.argv and any(
                arg.startswith('from ros2cli.daemon.daemonize import main')
                for arg in process.argv)) if process.argv else False


def targets(snapshot, protected, include_zenoh):
    selected = {pid for pid, process in snapshot.items()
                if pid not in protected and process.state != 'Z'
                and matches(process, include_zenoh)}
    # Include child processes with generic executable names (e.g. Gazebo ruby).
    while True:
        children = {pid for pid, process in snapshot.items()
                    if process.parent in selected and pid not in protected
                    and process.state != 'Z'}
        if children <= selected:
            break
        selected.update(children)
    return {pid: snapshot[pid] for pid in selected}


def alive(process):
    current = read_process(process.pid)
    return current is not None and current.started == process.started and current.state != 'Z'


def send(process, sig):
    descriptor = None
    try:
        # A pidfd prevents signaling an unrelated process if a PID gets reused.
        if hasattr(os, 'pidfd_open') and hasattr(signal, 'pidfd_send_signal'):
            descriptor = os.pidfd_open(process.pid)
            if alive(process):
                signal.pidfd_send_signal(descriptor, sig)
        elif alive(process):
            os.kill(process.pid, sig)
    except ProcessLookupError:
        pass
    except PermissionError as error:
        print(f'Cannot signal PID {process.pid}: {error}', file=sys.stderr)
    finally:
        if descriptor is not None:
            os.close(descriptor)


def stop_stage(selected, protected, include_zenoh, sig, timeout):
    print(f'Sending {sig.name}; waiting up to {timeout:g}s...', flush=True)
    signaled = set()
    deadline = time.monotonic() + timeout
    while True:
        # Re-scan to catch respawned nodes while launchers are shutting down.
        selected.update(targets(processes(), protected, include_zenoh))
        remaining = {pid: process for pid, process in selected.items() if alive(process)}
        for process in remaining.values():
            identity = (process.pid, process.started)
            if identity not in signaled:
                send(process, sig)
                signaled.add(identity)
        if not remaining or time.monotonic() >= deadline:
            return remaining
        time.sleep(0.1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dry-run', action='store_true', help='List targets without stopping them')
    parser.add_argument('--include-zenoh', action='store_true', help='Also stop Zenoh routers')
    parser.add_argument('--force', action='store_true', help='SIGKILL survivors after SIGINT/SIGTERM')
    parser.add_argument('--timeout', type=float, default=5,
                        help='Seconds to wait at each graceful shutdown stage (default: 5)')
    args = parser.parse_args()
    if not 0 < args.timeout <= 60:
        parser.error('--timeout must be greater than 0 and at most 60')
    snapshot = processes()
    protected = protected_pids(snapshot)
    selected = targets(snapshot, protected, args.include_zenoh)
    if not selected:
        print('No matching ROS/Gazebo processes owned by this user.')
        return 0
    print('Targets (active and stale processes owned by this user):')
    for process in sorted(selected.values(), key=lambda item: item.pid):
        print(f'  {process.pid}: {shlex.join(process.argv)}')
    if args.dry_run:
        print('Dry run: nothing stopped.')
        return 0
    for sig in (signal.SIGINT, signal.SIGTERM):
        selected = stop_stage(selected, protected, args.include_zenoh, sig, args.timeout)
        if not selected:
            break
    if selected and args.force:
        selected = stop_stage(selected, protected, args.include_zenoh, signal.SIGKILL, 1)
    if selected:
        print('Still running:', ', '.join(map(str, sorted(selected))), file=sys.stderr)
        if not args.force:
            print('Use --force to SIGKILL survivors.', file=sys.stderr)
        return 1
    print('ROS/Gazebo processes stopped. Discovery daemon will restart on the next ROS CLI command.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
