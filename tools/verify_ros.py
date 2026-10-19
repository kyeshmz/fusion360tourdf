"""Verify installed packages using real ROS launches, controllers, feedback, and TF."""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tests.fixtures import TARGET, VARIANTS, oracle
from tools.verify_loaders import check_urdfdom, expand, mesh_path


def transform_matrix(transform):
    t = transform.translation
    q = transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    assert abs(x * x + y * y + z * z + w * w - 1) < 1e-4, 'Invalid TF quaternion'
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), t.x],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), t.y],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y), t.z],
        [0, 0, 0, 1],
    ]


def matrices_match(actual, expected):
    return all(math.isfinite(a) and abs(a - b) < 5e-4
               for row_a, row_b in zip(actual, expected) for a, b in zip(row_a, row_b))


def stop_launch(process):
    for sig, timeout in ((signal.SIGINT, 10), (signal.SIGTERM, 5), (signal.SIGKILL, 5)):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            process.wait(timeout=1)
            return
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            process.poll()
            try:
                os.killpg(process.pid, 0)
            except ProcessLookupError:
                process.wait(timeout=1)
                return
            time.sleep(.05)
    raise RuntimeError('Could not stop ROS launch process group ' + str(process.pid))


def check_installed(package, robot_name, output):
    from ros2launch.api import get_share_file_path_from_package

    for relative in ('package.xml', 'viewers/display.launch.py',
                     'viewers/display.rviz', 'config/robot_data.yaml'):
        assert (package / relative).is_file(), 'Missing installed asset ' + relative
    for relative in ('launch/display.launch.py', 'config/display.rviz'):
        assert not (package / relative).exists(), 'Legacy installed viewer asset ' + relative
    for name in ('display.launch.py', 'display.rviz'):
        assert list(package.rglob(name)) == [package / 'viewers' / name], 'Duplicate viewer asset ' + name
    display = get_share_file_path_from_package(package_name=package.name, file_name='display.launch.py')
    assert Path(display).resolve() == (package / 'viewers/display.launch.py').resolve(), display
    urdf = expand(package, output / (robot_name + '.urdf'))
    check_urdfdom(urdf, output / (robot_name + '-urdfdom.log'))
    root = ET.parse(urdf).getroot()
    if any(joint.get('type') != 'fixed' for joint in root.findall('joint')):
        for relative in ('launch/control.launch.py', 'config/controllers.yaml'):
            assert (package / relative).is_file(), 'Missing installed control asset ' + relative
        assert list(package.rglob('control.launch.py')) == [package / 'launch/control.launch.py'], \
            'Duplicate control launch'
        control = get_share_file_path_from_package(package_name=package.name, file_name='control.launch.py')
        assert Path(control).resolve() == (package / 'launch/control.launch.py').resolve(), control
    else:
        assert not list(package.rglob('control.launch.py')), 'Fixed package has a control launch'
        assert not list(package.rglob('controllers.yaml')), 'Fixed package has controller configuration'
    assert not root.findall('collision'), 'Root-level collision'
    meshes = set()
    for link in root.findall('link'):
        assert link.findall('visual') and link.findall('collision'), 'Missing installed geometry'
        for mesh in link.findall('.//mesh'):
            path = Path(mesh_path(mesh.get('filename'), package, urdf))
            data = path.read_bytes()
            assert len(data) >= 84, 'Invalid installed STL'
            count = struct.unpack_from('<I', data, 80)[0]
            assert count > 0 and len(data) == 84 + count * 50, 'Invalid installed STL length'
            for offset in range(84, len(data), 50):
                assert all(math.isfinite(v) for v in struct.unpack_from('<12f', data, offset))
            assert mesh.get('scale').split() == ['0.001'] * 3, 'Millimeter mesh scale missing'
            meshes.add(str(path))
    assert len(meshes) == len(root.findall('link'))
    return sorted(meshes)


def verify_launch(package_name, mode, expected, output, timeout):
    try:
        import rclpy
        from action_msgs.msg import GoalStatus
        from control_msgs.action import FollowJointTrajectory
        from controller_manager_msgs.srv import ListControllers
        from rclpy.action import ActionClient
        from rclpy.time import Time
        from sensor_msgs.msg import JointState
        from tf2_ros import Buffer, TransformException, TransformListener
        from trajectory_msgs.msg import JointTrajectoryPoint
    except ImportError as exc:
        raise RuntimeError('Selected ROS suite requires sourced ROS runtime and control packages') from exc

    log = output / (package_name + '-' + mode + '.log')
    node = rclpy.create_node('verify_' + mode)
    buffer = Buffer()
    listener = TransformListener(buffer, node, spin_thread=False)
    latest = {}
    received = [0.0]

    def feedback(message):
        latest.clear()
        latest.update(zip(message.name, message.position))
        received[0] = time.monotonic()

    subscription = node.create_subscription(JointState, '/joint_states', feedback, 10)
    command = ['ros2', 'launch', package_name, mode + '.launch.py']
    if mode == 'display':
        command += ['gui:=false', 'rviz:=false']
    action = None
    process = None
    report = {}
    with log.open('w') as stream:
        try:
            process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                                       start_new_session=True)

            def wait_for(predicate, label):
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    if process.poll() is not None:
                        raise AssertionError(f'{mode} launch exited with {process.returncode}. See {log}')
                    rclpy.spin_once(node, timeout_sec=.1)
                    result = predicate()
                    if result:
                        return result
                raise TimeoutError(label + ' timed out. See ' + str(log))

            def observed_pose(values, after=0):
                targets = values['positions']
                if targets and (received[0] <= after or any(
                        name not in latest or not math.isfinite(latest[name]) or
                        abs(latest[name] - target) > 5e-4 for name, target in targets.items())):
                    return False
                transforms = {}
                for link, pose in values['link_poses'].items():
                    if link == 'base_link':
                        transforms[link] = pose
                        continue
                    try:
                        actual = transform_matrix(buffer.lookup_transform('base_link', link, Time()).transform)
                    except TransformException:
                        return False
                    if not matrices_match(actual, pose):
                        return False
                    transforms[link] = actual
                return {'joint_states': dict(latest), 'transforms': transforms}

            if mode == 'control':
                client = node.create_client(ListControllers, '/controller_manager/list_controllers')
                wait_for(client.service_is_ready, 'controller_manager service')
                deadline = time.monotonic() + timeout
                while True:
                    future = client.call_async(ListControllers.Request())
                    wait_for(future.done, 'list_controllers response')
                    states = {controller.name: controller.state for controller in future.result().controller}
                    if all(states.get(name) == 'active' for name in (
                            'joint_state_broadcaster', 'joint_trajectory_controller')):
                        report['controllers'] = states
                        break
                    if time.monotonic() >= deadline:
                        raise TimeoutError('Controllers did not become active: ' + str(states))
                node.destroy_client(client)
            report['zero'] = wait_for(lambda: observed_pose(expected['zero']), 'zero joint_states and TF')
            if mode == 'control':
                action = ActionClient(node, FollowJointTrajectory,
                                      '/joint_trajectory_controller/follow_joint_trajectory')
                wait_for(action.server_is_ready, 'trajectory action server')
                goal = FollowJointTrajectory.Goal()
                goal.trajectory.joint_names = list(TARGET)
                point = JointTrajectoryPoint()
                point.positions = list(TARGET.values())
                point.time_from_start.sec = 3
                goal.trajectory.points = [point]
                issued = time.monotonic()
                accepted = action.send_goal_async(goal)
                wait_for(accepted.done, 'trajectory acceptance')
                handle = accepted.result()
                assert handle.accepted, 'Trajectory goal was rejected'
                finished = handle.get_result_async()
                wait_for(finished.done, 'trajectory completion')
                result = finished.result()
                assert result.status == GoalStatus.STATUS_SUCCEEDED, 'Trajectory action did not succeed'
                assert result.result.error_code == 0, result.result.error_string
                report['action_error_code'] = result.result.error_code
                report['moved'] = wait_for(lambda: observed_pose(expected['moved'], issued),
                                          'commanded joint_states and numeric TF')
        finally:
            if action is not None:
                action.destroy()
            node.destroy_subscription(subscription)
            listener.unregister()
            node.destroy_node()
            if process is not None:
                stop_launch(process)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--install', type=Path, required=True)
    parser.add_argument('--timeout', type=float, default=45)
    args = parser.parse_args()
    assert args.timeout > 0
    from ament_index_python.packages import get_package_share_directory
    import rclpy
    from rclpy.utilities import get_rmw_implementation_identifier

    args.output.mkdir(parents=True, exist_ok=True)
    report = {'ros_distro': os.environ.get('ROS_DISTRO'),
              'rmw_implementation': get_rmw_implementation_identifier(),
              'python_version': sys.version.split()[0], 'packages': {}}
    rclpy.init()
    try:
        for variant in VARIANTS:
            robot_name = 'fixture_' + variant
            package_name = robot_name + '_description'
            package = Path(get_package_share_directory(package_name)).resolve()
            assert package.is_relative_to(args.install.resolve()), 'Launch resolved the source package'
            report['packages'][variant] = {'install_share': str(package),
                                          'meshes': check_installed(package, robot_name, args.output)}
            expected = oracle(variant == 'fixed')
            report['packages'][variant]['display'] = verify_launch(
                package_name, 'display', expected, args.output, args.timeout)
            (args.output / 'ros.json').write_text(json.dumps(report, indent=2) + '\n')
            if variant == 'articulated':
                report['packages'][variant]['control'] = verify_launch(
                    package_name, 'control', expected, args.output, args.timeout)
            (args.output / 'ros.json').write_text(json.dumps(report, indent=2) + '\n')
    finally:
        rclpy.shutdown()
    print('PASS installed ROS packages, headless display, active controllers, trajectory feedback, and TF')
    print('Report at ' + str(args.output / 'ros.json'))


if __name__ == '__main__':
    main()
