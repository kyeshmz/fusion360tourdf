# Original exporter by syuntoku14. See the repository MIT license.
import json
import os
from pathlib import Path
import re
import tempfile
import xml.etree.ElementTree as ET

from ..utils import config as config_mod
from ..utils import transforms as tf
from ..utils import validate
from ..utils.stl import read_binary_stl


def _link_frames(joints):
    frames = {'base_link': {'t': [0, 0, 0], 'R': tf.IDENTITY_3X3}}
    pending = list(joints.values())
    while pending:
        remaining = []
        for joint in pending:
            if joint['parent'] not in frames:
                remaining.append(joint)
                continue
            frames[joint['child']] = {
                't': joint['xyz'],
                'R': tf.mat3_mul(frames[joint['parent']]['R'],
                                 tf.mat3_from_rpy(joint.get('rpy', [0, 0, 0]))),
            }
        if len(remaining) == len(pending):
            raise ValueError('Joint tree is disconnected from base_link')
        pending = remaining
    return frames


def _local_point(point, frame):
    return tf.rotate_vector_by_mat3([a - b for a, b in zip(point, frame['t'])],
                                    tf.mat3_transpose(frame['R']))


def _origin(parent, xyz, rpy):
    return ET.SubElement(parent, 'origin', xyz=tf.format_vector(xyz), rpy=tf.format_vector(rpy))


def _robot_xml(robot_name, package_name, joints, inertial, frames, movable, config):
    robot = ET.Element('robot', name=robot_name)
    for name, entry in inertial.items():
        frame = frames[name]
        inverse_rotation = tf.mat3_transpose(frame['R'])
        mesh_xyz = tf.rotate_vector_by_mat3([-v for v in frame['t']], inverse_rotation)
        mesh_rpy = tf.rpy_from_mat3(inverse_rotation)
        link = ET.SubElement(robot, 'link', name=name)
        physics = ET.SubElement(link, 'inertial')
        _origin(physics, _local_point(entry['center_of_mass'], frame), [0, 0, 0])
        ET.SubElement(physics, 'mass', value=tf.format_float(entry['mass']))
        xx, yy, zz, xy, yz, xz = entry['inertia']
        tensor = (xx, xy, xz, xy, yy, yz, xz, yz, zz)
        tensor = tf.mat3_mul(tf.mat3_mul(inverse_rotation, tensor), frame['R'])
        ET.SubElement(physics, 'inertia', **{key: tf.format_float(tensor[index]) for key, index in
                      (('ixx', 0), ('iyy', 4), ('izz', 8), ('ixy', 1), ('iyz', 5), ('ixz', 2))})
        visual = ET.SubElement(link, 'visual')
        _origin(visual, mesh_xyz, mesh_rpy)
        uri = 'package://%s/meshes/%s.stl' % (package_name, name)
        ET.SubElement(ET.SubElement(visual, 'geometry'), 'mesh', filename=uri, scale='0.001 0.001 0.001')
        material = ET.SubElement(visual, 'material', name=name + '_material')
        rgba = entry.get('color_rgba')
        ET.SubElement(material, 'color', rgba=tf.format_vector(rgba if rgba is not None else [0.7, 0.7, 0.7, 1]))
        collision = ET.SubElement(link, 'collision')
        box_center, box_size = entry.get('bbox_center_m'), entry.get('bbox_size_m')
        if config['collision_method'] == 'primitive_box' and box_center is not None and box_size is not None:
            _origin(collision, _local_point(box_center, frame), mesh_rpy)
            ET.SubElement(ET.SubElement(collision, 'geometry'), 'box', size=tf.format_vector(box_size))
        else:
            _origin(collision, mesh_xyz, mesh_rpy)
            ET.SubElement(ET.SubElement(collision, 'geometry'), 'mesh', filename=uri, scale='0.001 0.001 0.001')
    for name, joint in joints.items():
        element = ET.SubElement(robot, 'joint', name=name, type=joint['type'])
        _origin(element, _local_point(joint['xyz'], frames[joint['parent']]), joint.get('rpy', [0, 0, 0]))
        ET.SubElement(element, 'parent', link=joint['parent'])
        ET.SubElement(element, 'child', link=joint['child'])
        if joint['type'] != 'fixed':
            ET.SubElement(element, 'axis', xyz=tf.format_vector(tf.normalize(joint['axis'])))
            limits = {key: tf.format_float(joint.get(key, config[key])) for key in ('effort', 'velocity')}
            if joint['type'] != 'continuous':
                limits.update(lower=tf.format_float(joint['lower_limit']), upper=tf.format_float(joint['upper_limit']))
            ET.SubElement(element, 'limit', **limits)
    if movable:
        control = ET.SubElement(robot, 'ros2_control', name=robot_name + '_system', type='system')
        ET.SubElement(ET.SubElement(control, 'hardware'), 'plugin').text = 'mock_components/GenericSystem'
        for name, joint in movable:
            element = ET.SubElement(control, 'joint', name=name)
            ET.SubElement(element, 'command_interface', name='position')
            position = ET.SubElement(element, 'state_interface', name='position')
            initial = 0 if joint['type'] == 'continuous' else min(max(0, joint['lower_limit']), joint['upper_limit'])
            ET.SubElement(position, 'param', name='initial_value').text = tf.format_float(initial)
            velocity = ET.SubElement(element, 'state_interface', name='velocity')
            ET.SubElement(velocity, 'param', name='initial_value').text = '0'
    return robot


def _write_xml(path, root):
    ET.indent(root, space='  ')
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)


def _launch(package_name, robot_name, control=False):
    preamble = f'''import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg = get_package_share_directory('{package_name}')
    description = xacro.process_file(os.path.join(pkg, 'urdf', '{robot_name}.xacro')).toxml()
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='{'false' if control else 'true'}'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{{'robot_description': description}}], output='screen'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(pkg, 'viewers', 'display.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
'''
    if control:
        return preamble + '''        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[os.path.join(pkg, 'config', 'controllers.yaml')],
             remappings=[('~/robot_description', '/robot_description'),
                         ('robot_description', '/robot_description')], output='screen'),
        Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager',
                        '--param-file', os.path.join(pkg, 'config', 'controllers.yaml')], output='screen'),
        Node(package='controller_manager', executable='spawner',
             arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager',
                        '--param-file', os.path.join(pkg, 'config', 'controllers.yaml')], output='screen'),
    ])
'''
    return preamble + '''        DeclareLaunchArgument('gui', default_value='true'),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             condition=IfCondition(LaunchConfiguration('gui'))),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             condition=UnlessCondition(LaunchConfiguration('gui'))),
    ])
'''


def _package_metadata(stage, package_name, movable):
    root = ET.Element('package', format='3')
    for tag, text in (('name', package_name), ('version', '0.0.0'),
                      ('description', 'Fusion robot description for ROS 2'),
                      ('license', 'MIT'), ('buildtool_depend', 'ament_cmake')):
        ET.SubElement(root, tag).text = text
    ET.SubElement(root, 'maintainer', email='maintainer@example.com').text = 'Robot maintainer'
    dependencies = ['ament_index_python', 'launch', 'launch_ros', 'xacro', 'robot_state_publisher',
                    'joint_state_publisher', 'joint_state_publisher_gui', 'rviz2']
    if movable:
        dependencies += ['controller_manager', 'hardware_interface', 'joint_state_broadcaster',
                         'joint_trajectory_controller']
    for dependency in dependencies:
        ET.SubElement(root, 'exec_depend').text = dependency
    ET.SubElement(ET.SubElement(root, 'export'), 'build_type').text = 'ament_cmake'
    _write_xml(stage / 'package.xml', root)
    (stage / 'CMakeLists.txt').write_text(f'''cmake_minimum_required(VERSION 3.8)
project({package_name})
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf meshes launch config viewers DESTINATION share/${{PROJECT_NAME}})
ament_package()
''', encoding='utf-8')


def write_ros2_package(destination, robot_name, joints_dict, inertial_dict, *, write_meshes, config=None):
    """Publish a new package only after its batch mesh callback succeeds.

    Joint xyz and link CoM/inertia are in the design frame, in SI units.
    Joint rpy is relative to its parent link and axis is in the joint frame.
    Meshes must be binary STL in design-frame millimeters.
    """
    destination = Path(destination).absolute()
    if os.path.lexists(destination):
        raise FileExistsError(destination)
    for label, name in (('robot', robot_name), ('package', destination.name)):
        if not isinstance(name, str) or not re.fullmatch(r'[a-z][a-z0-9_]*', name):
            raise ValueError('%s name must match [a-z][a-z0-9_]*' % label)
    config = config_mod.load_config(overrides=config)
    errors, warnings = validate.validate_export(joints_dict, inertial_dict, config=config)
    if errors:
        raise ValueError('\n'.join(errors))
    joints, closing, loop_warnings = validate.detect_closing_joints(joints_dict)
    frames = _link_frames(joints)
    movable = [(name, joint) for name, joint in joints.items() if joint['type'] != 'fixed']
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.' + destination.name + '-', dir=destination.parent) as temporary:
        stage = Path(temporary)
        for folder in ('urdf', 'meshes', 'launch', 'config', 'viewers'):
            (stage / folder).mkdir()
        _write_xml(stage / 'urdf' / (robot_name + '.xacro'),
                   _robot_xml(robot_name, destination.name, joints, inertial_dict, frames, movable, config))
        (stage / 'viewers/display.launch.py').write_text(_launch(destination.name, robot_name), encoding='utf-8')
        (stage / 'viewers/display.rviz').write_text('''Panels:
  - Class: rviz_common/Displays
Visualization Manager:
  Class: ""
  Global Options:
    Fixed Frame: base_link
  Displays:
    - Class: rviz_default_plugins/RobotModel
      Name: RobotModel
      Enabled: true
      Description Source: Topic
      Description Topic:
        Value: /robot_description
        Durability Policy: Transient Local
    - Class: rviz_default_plugins/TF
      Name: TF
      Enabled: true
''', encoding='utf-8')
        if movable:
            (stage / 'launch/control.launch.py').write_text(_launch(destination.name, robot_name, control=True), encoding='utf-8')
            controllers = {
                'controller_manager': {'ros__parameters': {
                    'update_rate': 100,
                    'joint_state_broadcaster': {'type': 'joint_state_broadcaster/JointStateBroadcaster'},
                    'joint_trajectory_controller': {'type': 'joint_trajectory_controller/JointTrajectoryController'},
                }},
                'joint_trajectory_controller': {'ros__parameters': {
                    'joints': [name for name, _ in movable], 'command_interfaces': ['position'],
                    'state_interfaces': ['position', 'velocity'],
                }},
            }
            (stage / 'config/controllers.yaml').write_text(json.dumps(controllers, indent=2) + '\n', encoding='utf-8')
        sidecar = {'robot_name': robot_name, 'package_name': destination.name,
                   'links': list(inertial_dict), 'joints': joints, 'closing_joints': closing,
                   'warnings': warnings + loop_warnings}
        if inertial_dict['base_link'].get('source_notes'):
            sidecar['source_notes'] = inertial_dict['base_link']['source_notes']
        (stage / 'config/robot_data.yaml').write_text(json.dumps(sidecar, indent=2) + '\n', encoding='utf-8')
        _package_metadata(stage, destination.name, movable)
        targets = {name: stage / 'meshes' / (name + '.stl') for name in inertial_dict}
        write_meshes(targets)
        for target in targets.values():
            read_binary_stl(target)
        if os.path.lexists(destination):
            raise FileExistsError(destination)
        stage.rename(destination)
    return destination
