# Original exporter by syuntoku14. See the repository MIT license.
import copy
import json
import math
import os
from pathlib import Path
import re
import tempfile
import xml.etree.ElementTree as ET

from ..utils import config as config_mod
from ..utils import transforms as tf
from ..utils import validate
from ..utils.export_policy import apply_export_policies, regularize_inertia, Y_UP_TO_Z_UP
from ..utils.stl import read_binary_stl
from ..utils.convex_collision import decompose_stl
from ..utils.convex_hull import hull_stl
from ..utils.mesh_geometry import CancelledError, ResourceLimitError
from .coupling import apply_couplings, plain_data, evaluate_affine
from ..utils.visual_assets import validate_asset_bundles, visual_descriptors


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


def _robot_xml(robot_name, package_name, joints, inertial, frames, movable, config, collision_meshes,
               visual_assets, coupling_plan, initial_positions, emission=None, output_reference=None):
    robot = ET.Element('robot', name=robot_name)
    if output_reference is not None:
        ET.SubElement(robot, 'link', name=output_reference['link'])
        element = ET.SubElement(robot, 'joint', name=output_reference['joint'], type='fixed')
        pose = emission['root_pose']
        _origin(element, pose['t'], tf.rpy_from_mat3(pose['R']))
        ET.SubElement(element, 'parent', link=output_reference['link'])
        ET.SubElement(element, 'child', link='base_link')
    for name, entry in inertial.items():
        link = ET.SubElement(robot, 'link', name=name)
        if entry.get('kind', 'physical') == 'frame':
            continue
        frame = frames[name]
        inverse_rotation = tf.mat3_transpose(frame['R'])
        if emission is None:
            mesh_xyz = tf.rotate_vector_by_mat3([-v for v in frame['t']], inverse_rotation)
            mesh_rpy = tf.rpy_from_mat3(inverse_rotation)
            com = _local_point(entry['center_of_mass'], frame)
            xx, yy, zz, xy, yz, xz = entry['inertia']
            tensor = (xx, xy, xz, xy, yy, yz, xz, yz, zz)
            tensor = tf.mat3_mul(tf.mat3_mul(inverse_rotation, tensor), frame['R'])
        else:
            mesh_pose = emission['mesh_origins'][name]
            mesh_xyz, mesh_rpy = mesh_pose['t'], tf.rpy_from_mat3(mesh_pose['R'])
            local = emission['inertials'][name]
            com = local['center_of_mass']
            xx, yy, zz, xy, yz, xz = local['inertia']
            tensor = (xx, xy, xz, xy, yy, yz, xz, yz, zz)
        physics = ET.SubElement(link, 'inertial')
        _origin(physics, com, [0, 0, 0])
        ET.SubElement(physics, 'mass', value=tf.format_float(entry['mass']))
        ET.SubElement(physics, 'inertia', **{key: tf.format_float(tensor[index]) for key, index in
                      (('ixx', 0), ('iyy', 4), ('izz', 8), ('ixy', 1), ('iyz', 5), ('ixz', 2))})
        uri = 'package://%s/meshes/%s.stl' % (package_name, name)
        for descriptor in visual_descriptors(visual_assets[name]) if name in visual_assets else (None,):
            visual = ET.SubElement(link, 'visual')
            _origin(visual, mesh_xyz, mesh_rpy)
            visual_uri = 'package://%s/%s' % (package_name, descriptor['mesh_path']) if descriptor else uri
            ET.SubElement(ET.SubElement(visual, 'geometry'), 'mesh', filename=visual_uri, scale='0.001 0.001 0.001')
            if descriptor is None:
                material = ET.SubElement(visual, 'material', name=name + '_material')
                rgba = entry.get('color_rgba')
                ET.SubElement(material, 'color', rgba=tf.format_vector(rgba if rgba is not None else [0.7, 0.7, 0.7, 1]))
        box_center, box_size = entry.get('bbox_center_m'), entry.get('bbox_size_m')
        if config['collision_method'] in ('convex_decomposition', 'convex_hull'):
            for index, relative in enumerate(collision_meshes[name]):
                collision = ET.SubElement(link, 'collision', name=name + '_convex_' + str(index))
                _origin(collision, mesh_xyz, mesh_rpy)
                ET.SubElement(ET.SubElement(collision, 'geometry'), 'mesh',
                              filename='package://%s/%s' % (package_name, relative), scale='0.001 0.001 0.001')
        elif config['collision_method'] == 'primitive_box' and box_center is not None and box_size is not None:
            collision = ET.SubElement(link, 'collision')
            if emission is None:
                _origin(collision, _local_point(box_center, frame), mesh_rpy)
            else:
                box = emission['boxes'][name]
                _origin(collision, box['t'], tf.rpy_from_mat3(box['R']))
            ET.SubElement(ET.SubElement(collision, 'geometry'), 'box', size=tf.format_vector(box_size))
        else:
            collision = ET.SubElement(link, 'collision')
            _origin(collision, mesh_xyz, mesh_rpy)
            ET.SubElement(ET.SubElement(collision, 'geometry'), 'mesh', filename=uri, scale='0.001 0.001 0.001')
    for name, joint in joints.items():
        element = ET.SubElement(robot, 'joint', name=name, type=joint['type'])
        if emission is None:
            _origin(element, _local_point(joint['xyz'], frames[joint['parent']]), joint.get('rpy', [0, 0, 0]))
        else:
            pose = emission['joint_origins'][name]
            _origin(element, pose['t'], tf.rpy_from_mat3(pose['R']))
        ET.SubElement(element, 'parent', link=joint['parent'])
        ET.SubElement(element, 'child', link=joint['child'])
        if joint['type'] != 'fixed':
            ET.SubElement(element, 'axis', xyz=tf.format_vector(tf.normalize(joint['axis'])))
            limits = {key: tf.format_float(joint[key]) for key in config_mod.JOINT_LIMIT_FIELDS}
            if joint['type'] != 'continuous':
                limits.update(lower=tf.format_float(joint['lower_limit']), upper=tf.format_float(joint['upper_limit']))
            ET.SubElement(element, 'limit', **limits)
            if 'mimic' in joint:
                mimic = joint['mimic']
                ET.SubElement(element, 'mimic', joint=mimic['joint'],
                              multiplier=tf.format_float(mimic['multiplier']), offset=tf.format_float(mimic['offset']))
    if movable:
        control = ET.SubElement(robot, 'ros2_control', name=robot_name + '_system', type='system')
        ET.SubElement(ET.SubElement(control, 'hardware'), 'plugin').text = 'mock_components/GenericSystem'
        controlled = list(movable)
        pending = [(name, joint) for name, joint in joints.items() if 'mimic' in joint]
        while pending:
            ready = {name for name, _ in controlled}
            for name, joint in pending[:]:
                if joint['mimic']['joint'] in ready:
                    controlled.append((name, joint))
                    pending.remove((name, joint))
        for name, joint in controlled:
            element = ET.SubElement(control, 'joint', name=name)
            if 'mimic' not in joint:
                ET.SubElement(element, 'command_interface', name='position')
            else:
                dependency = coupling_plan['dependencies'][name]
                ET.SubElement(element, 'param', name='mimic').text = dependency['independent_joint']
                ET.SubElement(element, 'param', name='multiplier').text = tf.format_float(dependency['multiplier'])
            position = ET.SubElement(element, 'state_interface', name='position')
            initial = initial_positions[name]
            ET.SubElement(position, 'param', name='initial_value').text = tf.format_float(initial)
            velocity = ET.SubElement(element, 'state_interface', name='velocity')
            ET.SubElement(velocity, 'param', name='initial_value').text = '0'
    return robot


def _write_xml(path, root):
    ET.indent(root, space='  ')
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)


def _launch(package_name, robot_name, control=False, humble_offset_blocked=False):
    runtime_guard = ("    if os.environ.get('ROS_DISTRO') == 'humble':\n"
                     "        raise RuntimeError('Humble GenericSystem cannot propagate nonzero mimic offsets. Use Jazzy or a purpose-built hardware implementation.')\n"
                     if humble_offset_blocked else '')
    preamble = f'''import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
{runtime_guard}    pkg = get_package_share_directory('{package_name}')
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


def resolve_joint_limits(joints_dict, config):
    """Bind normalized settings to an owned joint copy before topology reduction."""
    if not isinstance(joints_dict, dict):
        raise ValueError('Joint and inertial data must be dictionaries.')
    joints_dict = copy.deepcopy(joints_dict)
    for name, limits in config.get('joint_limits', {}).items():
        if name not in joints_dict:
            raise ValueError('joint_limits[%r] does not match an exported joint; use an exact exported '
                             'joint name. Available names: %s' %
                             (name, ', '.join(sorted(map(str, joints_dict)))))
        if not isinstance(joints_dict[name], dict):
            raise ValueError('Joint %s data must be a dictionary.' % name)
        if joints_dict[name].get('type') == 'fixed':
            raise ValueError('joint_limits[%r] targets a fixed joint; effort and velocity require '
                             'a moving joint' % name)
        joints_dict[name].update(limits)
    for joint in joints_dict.values():
        if isinstance(joint, dict) and joint.get('type') != 'fixed':
            for key in config_mod.JOINT_LIMIT_FIELDS:
                joint.setdefault(key, config[key])
    return joints_dict


def _output_reference(joints, inertial, root_pose):
    if root_pose['t'] == [0., 0., 0.] and root_pose['R'] == list(tf.IDENTITY_3X3):
        return None
    occupied = {name.casefold() for name in set(joints) | set(inertial)}

    def unique(stem):
        name, index = stem, 2
        while name.casefold() in occupied:
            name, index = stem + '_' + str(index), index + 1
        occupied.add(name.casefold())
        return name

    return {'link': unique('export_output_reference'), 'joint': unique('export_output_to_base_link')}


def _zero_floor_policy(joints, inertial):
    """Zero is identity even when finite tensor eigenvalues overflow."""
    receipts = {}
    for name, entry in inertial.items():
        if entry.get('kind', 'physical') == 'frame':
            continue
        six = entry['inertia']
        magnitude = max(map(abs, six)) or 1.
        normalized = regularize_inertia([v/magnitude for v in six])
        moments = [v*magnitude for v in normalized['principal_moments_before']]
        representable = all(math.isfinite(v) for v in moments)
        if not representable:
            moments = normalized['principal_moments_before']
        receipts[name] = dict(normalized, original=copy.deepcopy(six), final=copy.deepcopy(six),
                              enabled=True, changed=False, threshold_kg_m2=0., warning=None,
                              principal_moments_before=moments, principal_moments_after=list(moments),
                              principal_moment_scale_kg_m2=1. if representable else magnitude)
    identity = {'t': [0., 0., 0.], 'R': list(tf.IDENTITY_3X3)}
    return {'joints': joints, 'inertial': inertial, 'emission': None,
            'link_frames': _link_frames(joints), 'mesh_source_to_output': identity, 'warnings': [],
            'receipts': {'coordinate_rotation': {'enabled': False, 'source_to_output_rotation': identity['R']},
                         'frame_convention': {'selected': None, 'joints': {}}, 'inertia': receipts}}


def write_ros2_package(destination, robot_name, joints_dict, inertial_dict, *, write_meshes, config=None,
                       cancelled=None, visual_assets=None, motion_relations=None):
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
    identifiers = getattr(joints_dict, 'source_identifiers', None)
    removed = list(getattr(joints_dict, 'removed_joint_names', ()))
    source_closing = list(getattr(joints_dict, 'closing_joints', ()))
    source_loop_warnings = list(getattr(joints_dict, 'loop_warnings', ()))
    relations = list(getattr(joints_dict, 'motion_relations', ()))
    if motion_relations is not None:
        relations.extend(motion_relations)
    settings = {name: value for name, value in config.get('joint_limits', {}).items() if name not in removed}
    joints_dict = resolve_joint_limits(plain_data(dict(joints_dict), 'joints'), dict(config, joint_limits=settings))
    joints, closing, loop_warnings = validate.detect_closing_joints(joints_dict)
    removed.extend(entry['joint'] for entry in closing)
    for name, joint in joints_dict.items():
        if 'mimic' in joint:
            mimic = joint['mimic']
            if not isinstance(mimic, dict) or set(mimic) - {'joint', 'multiplier', 'offset'} or 'joint' not in mimic:
                raise ValueError('MIMIC_SCHEMA: expected joint, multiplier, offset')
            relations.append(dict(driver=mimic['joint'], follower=name, multiplier=mimic.get('multiplier', 1.),
                                  offset=mimic.get('offset', 0.), provenance={'source': 'explicit-mimic'}))
            if name in joints:
                joints[name] = dict(joints[name])
                del joints[name]['mimic']
    plan = apply_couplings(dict(joints), relations, removed_joint_names=removed) if relations else None
    if plan:
        if plan['representation_blockers']:
            raise ValueError('COUPLING_REPRESENTATION: ' + json.dumps(plan['representation_blockers']))
        joints = plan['joints']
    for name, joint in joints.items():
        if ('source_motion' in joint and joint['type'] in ('revolute', 'prismatic')
                and (joint.get('lower_limit') is None or joint.get('upper_limit') is None)):
            raise ValueError('URDF_FINITE_TRAVEL_REQUIRED: Joint %s needs finite lower_limit and upper_limit.' % name)
    errors, warnings = validate.validate_export(joints, inertial_dict, config=config)
    if errors:
        raise ValueError('\n'.join(errors))
    descriptors, asset_files = validate_asset_bundles(visual_assets, inertial_dict,
        config.get('visual_materials', 'legacy'), config.get('visual_meshes', 'combined'))
    closing = source_closing + closing
    loop_warnings = source_loop_warnings + loop_warnings
    policy = None
    output_reference = None
    root_pose = {'t': [0., 0., 0.], 'R': list(tf.IDENTITY_3X3)}
    if any(config.get(key) is not None for key in config_mod.POLICY_OPTIONS):
        if config.get('frame_convention') is not None:
            if closing:
                raise ValueError('dh_common_normal does not support closing joints')
            if (any('source_motion' in joint or 'generated_from' in joint for joint in joints.values())
                    or any('generated_from' in entry for entry in inertial_dict.values())):
                raise ValueError('dh_common_normal does not support native compound source_motion/generated_from provenance')
        rotation = Y_UP_TO_Z_UP if config.get('output_rotation_preset') is not None else config.get('output_rotation')
        original_closing = {entry['joint']: copy.deepcopy(joints_dict.get(entry['joint'], entry)) for entry in closing}
        if rotation is None and config.get('frame_convention') is None and config.get('minimum_principal_inertia') == 0:
            policy = _zero_floor_policy(joints, inertial_dict)
        else:
            policy = apply_export_policies(joints, inertial_dict, output_rotation=rotation,
                                          frame_convention=config.get('frame_convention'),
                                          minimum_principal_inertia=config.get('minimum_principal_inertia'))
            root_pose = policy['emission']['root_pose']
        joints, inertial_dict = policy['joints'], policy['inertial']
        frames = policy['link_frames']
        errors, derived_warnings = validate.validate_export(joints, inertial_dict, config=config)
        if errors:
            raise ValueError('\n'.join(errors))
        warnings += derived_warnings + policy['warnings']
        closing = copy.deepcopy(closing)
        if rotation is not None:
            for entry in closing:
                if 'xyz' in entry:
                    entry['xyz'] = tf.rotate_vector_by_mat3(entry['xyz'], rotation)
        output_reference = _output_reference(joints, inertial_dict, root_pose)
    else:
        frames = _link_frames(joints)
    independent = plan['independent_joints'] if plan else [n for n, j in joints.items() if j['type'] != 'fixed']
    movable = [(name, joints[name]) for name in independent]
    initial_positions = {name: (0. if joint['type'] == 'continuous' else
                                min(max(0., joint['lower_limit']), joint['upper_limit'])) for name, joint in movable}
    if plan:
        for name, dependency in plan['dependencies'].items():
            initial_positions[name] = evaluate_affine(dependency['multiplier'],
                initial_positions[dependency['independent_joint']], dependency['offset'],
                field='initial position for ' + name)

    def check_cancelled():
        if cancelled is not None and cancelled():
            raise CancelledError('Package export cancelled')

    check_cancelled()
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.' + destination.name + '-', dir=destination.parent) as temporary:
        stage = Path(temporary)
        for folder in ('urdf', 'meshes', 'launch', 'config', 'viewers'):
            (stage / folder).mkdir()
        targets = {name: stage / 'meshes' / (name + '.stl') for name, entry in inertial_dict.items()
                   if entry.get('kind', 'physical') == 'physical'}
        write_meshes(targets)
        check_cancelled()
        method = config['collision_method']
        simplified = method in ('convex_decomposition', 'convex_hull')
        for target in targets.values():
            if (simplified and target.stat().st_size > 84 + 50 * config[method]['max_triangles']):
                raise ResourceLimitError('Staged STL exceeds collision max_triangles byte allowance: ' + target.name)
            read_binary_stl(target)
        for relative, data in asset_files.items():
            check_cancelled()
            target = stage / relative
            if target.exists() or target.is_symlink() or any(parent.is_symlink() for parent in target.parents):
                raise ValueError('VISUAL_ASSET_DESTINATION_CONFLICT: ' + relative)
            target.parent.mkdir(parents=True, exist_ok=True)
            with target.open('xb') as stream:
                stream.write(data)
        collision_meshes, collision_metadata = {}, {}
        if simplified:
            controls = config[method]
            remaining_pieces, remaining_work = controls['max_pieces'], controls['max_work']
            remaining_bytes = controls.get('max_output_bytes')
            for name, target in targets.items():
                if remaining_pieces <= 0 or remaining_work <= 0 or remaining_bytes == 0:
                    raise ResourceLimitError('Package collision simplification exceeds aggregate piece/work/byte budget')
                options = dict(controls, max_pieces=remaining_pieces, max_work=remaining_work)
                if method == 'convex_hull':
                    options['max_output_bytes'] = remaining_bytes
                    result = hull_stl(target.read_bytes(), **options, cancelled=cancelled)
                else:
                    result = decompose_stl(target.read_bytes(), **options, cancelled=cancelled)
                folder = stage / 'meshes/collision' / name
                folder.mkdir(parents=True)
                relatives = []
                for index, data in enumerate(result['pieces']):
                    check_cancelled()
                    path = folder / ('piece_%06d.stl' % index)
                    path.write_bytes(data)
                    read_binary_stl(path)
                    relatives.append(path.relative_to(stage).as_posix())
                collision_meshes[name] = relatives
                collision_metadata[name] = dict(result['metadata'], assets=relatives)
                remaining_pieces -= len(result['pieces'])
                remaining_work -= result['metadata']['work_used']
                if remaining_bytes is not None:
                    remaining_bytes -= result['metadata']['output_bytes']
            if method == 'convex_hull':
                warnings.append('Single convex hull collision intentionally fills concavities, not decomposition. '
                                'The hull encloses the input STL vertices; visual and source physics are unchanged unless an explicit inertia floor is recorded.')
                if controls.get('source_policy') == 'triangle_vertices':
                    warnings.append('Hull source topology is not verified. The hull fills gaps between bodies '
                                    'and any cavities; source triangles are not a closed-CAD or density claim.')
            else:
                warnings.append('Convex collision uses conservative voxel cells, not exact CAD geometry. '
                                'See collision_decomposition bounds and single-shell limits; visual and CAD physics are unchanged.')
        robot = _robot_xml(robot_name, destination.name, joints, inertial_dict, frames, movable, config, collision_meshes,
                           descriptors, plan, initial_positions,
                           emission=policy['emission'] if policy else None, output_reference=output_reference)
        _write_xml(stage / 'urdf' / (robot_name + '.xacro'), robot)
        _write_xml(stage / 'urdf' / (robot_name + '.urdf'), robot)
        components = stage / 'urdf/components'
        components.mkdir()
        for link in robot.findall('link'):
            component = ET.Element('robot', name=robot_name + '_' + link.get('name'))
            component.append(copy.deepcopy(link))
            _write_xml(components / (link.get('name') + '.urdf'), component)
        (stage / 'viewers/display.launch.py').write_text(_launch(destination.name, robot_name), encoding='utf-8')
        fixed_frame = output_reference['link'] if output_reference else 'base_link'
        (stage / 'viewers/display.rviz').write_text(f'''Panels:
  - Class: rviz_common/Displays
Visualization Manager:
  Class: ""
  Global Options:
    Fixed Frame: {fixed_frame}
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
            offset_blocked = bool(plan and any(d['offset'] != 0 for d in plan['dependencies'].values()))
            (stage / 'launch/control.launch.py').write_text(_launch(destination.name, robot_name, control=True,
                                                                  humble_offset_blocked=offset_blocked), encoding='utf-8')
            if offset_blocked:
                warnings.append('Humble GenericSystem does not implement nonzero mimic offsets; control launch rejects on Humble. Jazzy runtime still requires verification.')
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
                    'urdf': 'urdf/' + robot_name + '.urdf',
                    'component_urdfs': {name: 'urdf/components/' + name + '.urdf' for name in inertial_dict},
                     'warnings': warnings + loop_warnings}
        if identifiers is not None:
            sidecar['source_identifiers'] = identifiers
        sidecar['independent_joints'] = independent
        sidecar['initial_positions'] = initial_positions
        if plan:
            sidecar['motion_coupling'] = {k: v for k, v in plan.items() if k != 'joints'}
        if descriptors:
            sidecar['visual_assets'] = descriptors
            sidecar['warnings'].extend(sorted({w for value in descriptors.values()
                for d in visual_descriptors(value) for w in d['warnings']}))
            if config.get('visual_meshes', 'combined') == 'per_body':
                sidecar['visual_meshes'] = 'per_body'
        sidecar['link_sources'] = {name: {'kind': entry.get('kind', 'physical'), 'path': entry.get('source_path')}
                                   for name, entry in inertial_dict.items()}
        for name, entry in inertial_dict.items():
            for key in ('generated_from', 'current_pose'):
                if key in entry:
                    sidecar['link_sources'][name][key] = plain_data(entry[key], name+'.'+key)
        mesh_physics = {name: entry['mesh_physics'] for name, entry in inertial_dict.items() if entry.get('mesh_physics')}
        if mesh_physics:
            sidecar['mesh_physics'] = mesh_physics
        if policy is not None:
            sidecar['root_frame'] = {'source_root_link': 'base_link', 'output_reference_link': fixed_frame,
                                    'source_root_pose_in_output': root_pose,
                                    'fixed_joint': output_reference['joint'] if output_reference else None}
            sidecar['export_policies'] = {
                'selected': {key: config.get(key) for key in config_mod.POLICY_OPTIONS},
                'receipts': policy['receipts'], 'link_frames': dict(frames),
                'mesh_source_to_output': policy['mesh_source_to_output'],
                'mesh_coordinates': 'unchanged-source-design-millimeters',
                'source_closing_joints': original_closing,
                'source_closing_records': copy.deepcopy(source_closing),
                'closure_coordinates': 'xyz in output design meters; rpy parent-relative; axis joint-local; '
                                       'opaque source metadata unchanged; original complete records retained'}
            if output_reference is not None:
                name = output_reference['link']
                sidecar['links'] = [name] + sidecar['links']
                sidecar['component_urdfs'][name] = 'urdf/components/' + name + '.urdf'
                sidecar['link_sources'][name] = {'kind': 'frame', 'path': None, 'generated': 'output_reference'}
                sidecar['export_policies']['link_frames'][name] = {'t': [0., 0., 0.], 'R': list(tf.IDENTITY_3X3)}
                sidecar['joints'] = dict(joints)
                sidecar['joints'][output_reference['joint']] = {
                    'parent': name, 'child': 'base_link', 'type': 'fixed', 'xyz': root_pose['t'],
                    'rpy': list(tf.rpy_from_mat3(root_pose['R'])), 'generated': 'output_reference'}
        if config['collision_method'] == 'convex_decomposition':
            sidecar['collision_decomposition'] = {'links': collision_metadata, 'controls': controls,
                'budget_scope': 'max_pieces/max_work aggregate per package; other limits per physical link',
                'physics_policy': 'supplied-CAD-physics-unchanged', 'visual_policy': 'original-STL-bytes-unchanged'}
        elif method == 'convex_hull':
            sidecar['convex_hull'] = {'links': collision_metadata, 'controls': controls,
                'budget_scope': 'max_pieces/max_work/max_output_bytes aggregate per package; other limits per physical link',
                'physics_policy': 'supplied-CAD-physics-unchanged', 'visual_policy': 'original-STL-bytes-unchanged'}
        if simplified:
            key = 'convex_hull' if method == 'convex_hull' else 'collision_decomposition'
            if mesh_physics:
                sidecar[key]['physics_policy'] = 'source-mesh-physics-unchanged'
            if descriptors:
                sidecar[key]['portable_visual_policy'] = 'manifested-OBJ-MTL-image-bytes-unchanged'
        if simplified and policy is not None and any(entry['changed'] for entry in policy['receipts']['inertia'].values()):
            key = 'convex_hull' if method == 'convex_hull' else 'collision_decomposition'
            sidecar[key]['physics_policy'] = 'mass-unchanged; output-inertia-explicit-spectral-floor'
            sidecar['warnings'] = [warning.replace('visual and CAD physics are unchanged',
                'visual and mass are unchanged; output inertia has an explicit spectral floor') for warning in sidecar['warnings']]
        if inertial_dict['base_link'].get('source_notes'):
            sidecar['source_notes'] = inertial_dict['base_link']['source_notes']
        (stage / 'config/robot_data.yaml').write_text(json.dumps(sidecar, indent=2, allow_nan=False) + '\n', encoding='utf-8')
        _package_metadata(stage, destination.name, movable)
        check_cancelled()
        if os.path.lexists(destination):
            raise FileExistsError(destination)
        stage.rename(destination)
    return destination
