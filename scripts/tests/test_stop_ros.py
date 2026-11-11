"""Run with: python3 -m unittest discover -s scripts/tests -v"""

import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import sys
import unittest
from unittest.mock import patch


spec = importlib.util.spec_from_file_location('stop_ros', Path(__file__).parents[1] / 'stop_ros.py')
stop_ros = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = stop_ros
spec.loader.exec_module(stop_ros)


def process(pid, *argv, parent=1, state='S'):
    return stop_ros.Process(pid, parent, '123', argv, str(stop_ros.WORKSPACE), state)


class SelectionTests(unittest.TestCase):
    def test_ros_nodes_launchers_and_gazebo(self):
        for argv in [
            ('/usr/bin/python3', '/opt/ros/jazzy/lib/rtabmap_slam/rtabmap'),
            ('python3', '-u', '/opt/ros/jazzy/bin/ros2', 'launch', 'startup', 'launch.py'),
            ('ruby', '/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz', 'sim', '-s'),
            ('/home/other/install/robot_perception/lib/robot_perception/AprilTagGate',),
            (str(stop_ros.WORKSPACE / 'install/robot_perception/lib/robot_perception/AprilTagGate'),),
            ('bash', './gazebo_deploy.sh'),
        ]:
            with self.subTest(argv=argv):
                expected = not argv[0].startswith('/home/other/')
                self.assertEqual(stop_ros.matches(process(100, *argv)), expected)

    def test_unrelated_processes_and_command_text_are_not_selected(self):
        for argv in [
            ('bash',), ('code', '/opt/ros/jazzy/lib/foo/bar'),
            ('python3', '-c', "print('gz sim')"),
            ('bash', '-c', 'ros2 launch startup launch.py'),
            ('python3', 'some_script.py'),
            ('gz', 'topic', '-l'), ('ros2', 'topic', 'echo', '/detections'),
        ]:
            with self.subTest(argv=argv):
                self.assertFalse(stop_ros.matches(process(100, *argv)))

    def test_zenoh_is_opt_in(self):
        for argv in [('rmw_zenohd',),
                     ('/opt/ros/jazzy/lib/rmw_zenoh_cpp/rmw_zenohd',),
                     ('ros2', 'run', 'rmw_zenoh_cpp', 'rmw_zenohd')]:
            self.assertFalse(stop_ros.matches(process(100, *argv)))
            self.assertTrue(stop_ros.matches(process(100, *argv), include_zenoh=True))

    def test_descendants_protected_ancestors_and_zombies(self):
        snapshot = {
            1: process(1, 'init'),
            2: process(2, 'rviz2', parent=1),
            3: process(3, 'generic-renderer', parent=2),
            4: process(4, 'generic-worker', parent=3),
            5: process(5, 'rviz2', state='Z'),
            6: process(6, 'rviz2', parent=1),
        }
        self.assertEqual(set(stop_ros.targets(snapshot, {1, 6}, False)), {2, 3, 4})

    def test_dry_run_never_signals(self):
        with patch.object(stop_ros, 'processes', return_value={100: process(100, 'rviz2')}), \
                patch.object(sys, 'argv', ['stop_ros.py', '--dry-run']), \
                patch.object(stop_ros, 'send') as send:
            self.assertEqual(stop_ros.main(), 0)
            send.assert_not_called()


class SignalTests(unittest.TestCase):
    def test_graceful_shutdown_and_force_for_unresponsive_process(self):
        for ignore, sig in [(False, signal.SIGTERM), (True, signal.SIGKILL)]:
            code = ('import signal,time; '
                    + ('signal.signal(signal.SIGTERM, signal.SIG_IGN); ' if ignore else '')
                    + "print('ready', flush=True); time.sleep(30)")
            child = subprocess.Popen([sys.executable, '-c', code], stdout=subprocess.PIPE, text=True)
            try:
                self.assertEqual(child.stdout.readline().strip(), 'ready')
                target = stop_ros.read_process(child.pid)
                self.assertIsNotNone(target)
                stop_ros.send(target, sig)
                self.assertEqual(child.wait(timeout=3), -sig)
            finally:
                if child.poll() is None:
                    child.kill()
                    child.wait()
                child.stdout.close()

    def test_reused_pid_is_not_signaled(self):
        current = stop_ros.read_process(os.getpid())
        old = stop_ros.Process(current.pid, current.parent, 'not-the-current-start-time',
                               current.argv, current.cwd, current.state)
        self.assertFalse(stop_ros.alive(old))
        with patch.object(signal, 'pidfd_send_signal') as send:
            stop_ros.send(old, signal.SIGTERM)
            send.assert_not_called()


if __name__ == '__main__':
    unittest.main()
