# -*- coding: utf-8 -*-
"""
Created on Sun May 12 20:46:26 2019

@author: syuntoku
"""

import os
from xml.etree.ElementTree import Element, SubElement
from . import Link, Joint
from ..utils import utils
from ..utils import transforms as tf
from ..utils import materials as matlib


def _effort_velocity(config):
    try:
        return float((config or {}).get('effort', 100.0)), float((config or {}).get('velocity', 100.0))
    except Exception:
        return 100.0, 100.0


def _link_frames(joints_dict):
    """Accumulate link frames (root-frame pose) following the joint tree.

    Returns {link: {'t': [x,y,z] m, 'R': 3x3 row-major}}. base_link is
    identity. Children processed topologically so branches work regardless
    of Fusion joint order. Unresolvable joints keep legacy translation-only
    behavior downstream.
    """
    frames = {'base_link': {'t': [0.0, 0.0, 0.0], 'R': tf.IDENTITY_3X3}}
    pending = list(joints_dict.items())
    progressed = True
    while pending and progressed:
        progressed = False
        remaining = []
        for name, joint in pending:
            parent = joint.get('parent')
            child = joint.get('child')
            if parent in frames:
                try:
                    parent_frame = frames[parent]
                    t_joint = list(joint.get('xyz', [0.0, 0.0, 0.0]))
                    r_joint = tf.mat3_from_rpy(joint.get('rpy', [0, 0, 0]) or [0, 0, 0])
                    r_child = tf.mat3_mul(parent_frame['R'], r_joint)
                    frames[child] = {'t': t_joint, 'R': r_child}
                    progressed = True
                except Exception:
                    frames.setdefault(child, {'t': list(joint.get('xyz', [0.0, 0.0, 0.0])),
                                              'R': tf.IDENTITY_3X3})
                    progressed = True
            else:
                remaining.append((name, joint))
        pending = remaining
    for name, joint in pending:
        # Disconnected subtree: anchor at its joint position with identity rotation.
        frames.setdefault(joint.get('child'), {'t': list(joint.get('xyz', [0.0, 0.0, 0.0])),
                                               'R': tf.IDENTITY_3X3})
    return frames


def _joint_origin_pose(joint, parent_frame):
    """URDF joint origin (translation + rpy) relative to the parent link frame."""
    try:
        t_joint = list(joint.get('xyz', [0.0, 0.0, 0.0]))
        t_parent = parent_frame.get('t', [0.0, 0.0, 0.0])
        r_parent = parent_frame.get('R', tf.IDENTITY_3X3)
        diff = [t_joint[0] - t_parent[0], t_joint[1] - t_parent[1], t_joint[2] - t_parent[2]]
        rpt = tf.mat3_transpose(r_parent)
        t_rel = [rpt[0] * diff[0] + rpt[1] * diff[1] + rpt[2] * diff[2],
                 rpt[3] * diff[0] + rpt[4] * diff[1] + rpt[5] * diff[2],
                 rpt[6] * diff[0] + rpt[7] * diff[1] + rpt[8] * diff[2]]
        xyz = [round(v, 6) for v in t_rel]
    except Exception:
        xyz = [round(float(v), 6) for v in joint.get('xyz', [0.0, 0.0, 0.0])]
    rpy = list(joint.get('rpy', [0, 0, 0]) or [0, 0, 0])
    return xyz, rpy

def write_link_urdf(joints_dict, repo, links_xyz_dict, file_name, inertial_dict,
                    config=None, links_rpy_dict=None, link_frames=None):
    """
    Write links information into urdf "repo/file_name"


    Parameters
    ----------
    joints_dict: dict
        information of the each joint
    repo: str
        the name of the repository to save the xml file
    links_xyz_dict: vacant dict
        xyz information of the each link
    file_name: str
        urdf full path
    inertial_dict:
        information of the each inertial

    Note
    ----------
    In this function, links_xyz_dict is set for write_joint_tran_urdf.
    The origin of the coordinate of center_of_mass is the coordinate of the link
    """
    if links_rpy_dict is None:
        links_rpy_dict = {}
    frames = link_frames if link_frames is not None else _link_frames(joints_dict)
    with open(file_name, mode='a') as f:
        # for base_link
        center_of_mass = inertial_dict['base_link']['center_of_mass']
        link = Link.Link(name='base_link', xyz=[0,0,0],
            center_of_mass=center_of_mass, repo=repo,
            mass=inertial_dict['base_link']['mass'],
            inertia_tensor=inertial_dict['base_link']['inertia'],
            rpy=[0, 0, 0],
            color_rgba=inertial_dict['base_link'].get('color_rgba'))
        links_xyz_dict[link.name] = link.xyz
        links_rpy_dict[link.name] = list(link.rpy)
        link.make_link_xml(with_collision_mesh=False)
        f.write(link.link_xml)
        f.write('\n')
        _write_collision(f, 'base_link', frames.get('base_link', {'t': [0.0, 0.0, 0.0],
                                                                   'R': tf.IDENTITY_3X3}),
                         inertial_dict['base_link'], repo, config, [0, 0, 0], [0, 0, 0])

        # others
        for joint in joints_dict:
            name = joints_dict[joint]['child']
            center_of_mass = \
                [ i-j for i, j in zip(inertial_dict[name]['center_of_mass'], joints_dict[joint]['xyz'])]
            frame = frames.get(name, {'t': list(joints_dict[joint]['xyz']),
                                      'R': tf.IDENTITY_3X3})
            # Visual origin undoes the link frame pose (meshes are in root frame).
            vis_xyz, vis_rpy = tf.inverse_pose(frame['t'], list(tf.rpy_from_mat3(frame['R'])))
            vis_xyz = [round(v, 6) for v in vis_xyz]
            vis_rpy = [round(v, 6) for v in vis_rpy]
            link = Link.Link(name=name, xyz=joints_dict[joint]['xyz'],
                center_of_mass=center_of_mass,
                repo=repo, mass=inertial_dict[name]['mass'],
                inertia_tensor=inertial_dict[name]['inertia'],
                rpy=vis_rpy,
                color_rgba=inertial_dict[name].get('color_rgba'))
            # Override with the exact inverse-pose translation.
            link.xyz = vis_xyz
            link.rpy = vis_rpy
            links_xyz_dict[link.name] = link.xyz
            links_rpy_dict[link.name] = list(link.rpy)
            link.make_link_xml(with_collision_mesh=False)
            f.write(link.link_xml)
            f.write('\n')
            _write_collision(f, name, frame, inertial_dict[name], repo, config, vis_xyz, vis_rpy)


def _write_collision(f, link_name, frame, inertial_entry, repo, config,
                     vis_xyz=None, vis_rpy=None):
    """Append one <collision> block: primitive box when configured, else mesh."""
    if vis_xyz is None:
        vis_xyz = [round(-v, 6) for v in frame.get('t', [0.0, 0.0, 0.0])]
    if vis_rpy is None:
        try:
            vis_rpy = [round(v, 6) for v in tf.rpy_from_mat3(tf.mat3_transpose(frame.get('R', tf.IDENTITY_3X3)))]
        except Exception:
            vis_rpy = [0, 0, 0]
    method = (config or {}).get('collision_method', 'visual')
    bbox_center = (inertial_entry or {}).get('bbox_center_m')
    bbox_size = (inertial_entry or {}).get('bbox_size_m')
    if method == 'primitive_box' and bbox_center is not None and bbox_size is not None:
        try:
            if all(s > 1e-9 for s in bbox_size):
                # Box center in the link frame: R_link^T * (c_root - t_link).
                rpt = tf.mat3_transpose(frame.get('R', tf.IDENTITY_3X3))
                diff = [bbox_center[0] - frame['t'][0], bbox_center[1] - frame['t'][1],
                        bbox_center[2] - frame['t'][2]]
                c_rel = [rpt[0] * diff[0] + rpt[1] * diff[1] + rpt[2] * diff[2],
                         rpt[3] * diff[0] + rpt[4] * diff[1] + rpt[5] * diff[2],
                         rpt[6] * diff[0] + rpt[7] * diff[1] + rpt[8] * diff[2]]
                box_rpy = list(tf.rpy_from_mat3(rpt))
                collision = Element('collision')
                origin = SubElement(collision, 'origin')
                origin.attrib = {'xyz': tf.format_vector([round(v, 6) for v in c_rel]),
                                 'rpy': tf.format_vector([round(v, 6) for v in box_rpy])}
                geometry = SubElement(collision, 'geometry')
                box = SubElement(geometry, 'box')
                box.attrib = {'size': tf.format_vector([round(v, 6) for v in bbox_size])}
                f.write('\n'.join(utils.prettify(collision).split('\n')[1:]))
                f.write('\n')
                return
        except Exception:
            pass
    collision = Element('collision')
    origin = SubElement(collision, 'origin')
    origin.attrib = {'xyz': ' '.join(str(_) for _ in vis_xyz),
                     'rpy': tf.format_vector(vis_rpy)}
    geometry = SubElement(collision, 'geometry')
    mesh = SubElement(geometry, 'mesh')
    mesh.attrib = {'filename': 'package://' + repo + link_name + '.stl',
                   'scale': '0.001 0.001 0.001'}
    f.write('\n'.join(utils.prettify(collision).split('\n')[1:]))
    f.write('\n')


def write_joint_urdf(joints_dict, repo, links_xyz_dict, file_name, config=None,
                     link_frames=None):
    """
    Write joints and transmission information into urdf "repo/file_name"


    Parameters
    ----------
    joints_dict: dict
        information of the each joint
    repo: str
        the name of the repository to save the xml file
    links_xyz_dict: dict
        xyz information of the each link
    file_name: str
        urdf full path
    """

    effort, velocity = _effort_velocity(config)
    frames = link_frames if link_frames is not None else _link_frames(joints_dict)
    with open(file_name, mode='a') as f:
        for j in joints_dict:
            parent = joints_dict[j]['parent']
            child = joints_dict[j]['child']
            joint_type = joints_dict[j]['type']
            upper_limit = joints_dict[j]['upper_limit']
            lower_limit = joints_dict[j]['lower_limit']
            parent_frame = frames.get(parent, {'t': [0.0, 0.0, 0.0], 'R': tf.IDENTITY_3X3})
            xyz, rpy = _joint_origin_pose(joints_dict[j], parent_frame)

            joint = Joint.Joint(name=j, joint_type=joint_type, xyz=xyz,
            axis=joints_dict[j]['axis'], parent=parent, child=child,
            upper_limit=upper_limit, lower_limit=lower_limit, rpy=rpy,
            effort=joints_dict[j].get('effort', effort),
            velocity=joints_dict[j].get('velocity', velocity))
            joint.make_joint_xml()
            joint.make_transmission_xml()
            f.write(joint.joint_xml)
            f.write('\n')

def write_gazebo_endtag(file_name):
    """
    Write about gazebo_plugin and the </robot> tag at the end of the urdf
    
    
    Parameters
    ----------
    file_name: str
        urdf full path
    """
    with open(file_name, mode='a') as f:
        f.write('</robot>\n')
        

def write_urdf(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir,
               config=None, link_frames=None, links_rpy_dict=None):
    os.makedirs(os.path.join(save_dir, 'urdf'), exist_ok=True)

    file_name = os.path.join(save_dir, 'urdf', robot_name + '.xacro')  # the name of urdf file
    repo = package_name + '/meshes/'  # the repository of binary stl files
    with open(file_name, mode='w') as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write('<robot name="{}" xmlns:xacro="http://www.ros.org/wiki/xacro">\n'.format(robot_name))
        f.write('\n')
        f.write('<xacro:include filename="$(find {})/urdf/materials.xacro" />'.format(package_name))
        f.write('\n')
        f.write('<xacro:include filename="$(find {})/urdf/{}.trans" />'.format(package_name, robot_name))
        f.write('\n')
        f.write('<xacro:include filename="$(find {})/urdf/{}.gazebo" />'.format(package_name, robot_name))
        f.write('\n')

    frames = link_frames if link_frames is not None else _link_frames(joints_dict)
    if links_rpy_dict is None:
        links_rpy_dict = {}
    write_link_urdf(joints_dict, repo, links_xyz_dict, file_name, inertial_dict,
                     config=config, links_rpy_dict=links_rpy_dict, link_frames=frames)
    write_joint_urdf(joints_dict, repo, links_xyz_dict, file_name, config=config,
                     link_frames=frames)
    write_gazebo_endtag(file_name)
    return frames, links_rpy_dict

def write_materials_xacro(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir):
    os.makedirs(os.path.join(save_dir, 'urdf'), exist_ok=True)

    file_name = os.path.join(save_dir, 'urdf', 'materials.xacro')  # the name of urdf file
    with open(file_name, mode='w') as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write('<robot name="{}" xmlns:xacro="http://www.ros.org/wiki/xacro" >\n'.format(robot_name))
        f.write('\n')
        f.write('<material name="silver">\n')
        f.write('  <color rgba="0.700 0.700 0.700 1.000"/>\n')
        f.write('</material>\n')
        f.write('\n')
        # Per-link Fusion appearance colors (newtonjeri/Adriaeik direction).
        try:
            emitted = set()
            for link_name in ['base_link'] + [joints_dict[j]['child'] for j in joints_dict]:
                entry = (inertial_dict or {}).get(link_name, {})
                rgba = entry.get('color_rgba')
                if rgba is None or link_name in emitted:
                    continue
                emitted.add(link_name)
                f.write('<material name="%s">\n' % matlib.material_name_for(link_name))
                f.write('  <color rgba="%s"/>\n' % matlib.rgba_to_urdf_string(rgba))
                f.write('</material>\n')
                f.write('\n')
        except Exception:
            pass
        f.write('</robot>\n')

def write_transmissions_xacro(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir,
                              config=None, link_frames=None):
    """
    Write joints and transmission information into urdf "repo/file_name"


    Parameters
    ----------
    joints_dict: dict
        information of the each joint
    repo: str
        the name of the repository to save the xml file
    links_xyz_dict: dict
        xyz information of the each link
    file_name: str
        urdf full path
    """

    effort, velocity = _effort_velocity(config)
    frames = link_frames if link_frames is not None else _link_frames(joints_dict)
    file_name = os.path.join(save_dir, 'urdf', '{}.trans'.format(robot_name))  # the name of urdf file
    with open(file_name, mode='w') as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write('<robot name="{}" xmlns:xacro="http://www.ros.org/wiki/xacro" >\n'.format(robot_name))
        f.write('\n')

        for j in joints_dict:
            parent = joints_dict[j]['parent']
            child = joints_dict[j]['child']
            joint_type = joints_dict[j]['type']
            upper_limit = joints_dict[j]['upper_limit']
            lower_limit = joints_dict[j]['lower_limit']
            parent_frame = frames.get(parent, {'t': [0.0, 0.0, 0.0], 'R': tf.IDENTITY_3X3})
            xyz, rpy = _joint_origin_pose(joints_dict[j], parent_frame)

            joint = Joint.Joint(name=j, joint_type = joint_type, xyz=xyz, \
            axis=joints_dict[j]['axis'], parent=parent, child=child, \
            upper_limit=upper_limit, lower_limit=lower_limit, rpy=rpy,
            effort=joints_dict[j].get('effort', effort),
            velocity=joints_dict[j].get('velocity', velocity))
            if joint_type != 'fixed':
                joint.make_transmission_xml()
                f.write(joint.tran_xml)
                f.write('\n')

        f.write('</robot>\n')

def write_gazebo_xacro(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir):
    os.makedirs(os.path.join(save_dir, 'urdf'), exist_ok=True)

    file_name = os.path.join(save_dir, 'urdf', robot_name + '.gazebo')  # the name of urdf file
    repo = robot_name + '/meshes/'  # the repository of binary stl files
    #repo = package_name + '/' + robot_name + '/bin_stl/'  # the repository of binary stl files
    with open(file_name, mode='w') as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write('<robot name="{}" xmlns:xacro="http://www.ros.org/wiki/xacro" >\n'.format(robot_name))
        f.write('\n')
        f.write('<xacro:property name="body_color" value="Gazebo/Silver" />\n')
        f.write('\n')

        gazebo = Element('gazebo')
        plugin = SubElement(gazebo, 'plugin')
        plugin.attrib = {'name':'control', 'filename':'libgazebo_ros_control.so'}
        gazebo_xml = "\n".join(utils.prettify(gazebo).split("\n")[1:])
        f.write(gazebo_xml)

        # for base_link
        f.write('<gazebo reference="base_link">\n')
        f.write('  <material>${body_color}</material>\n')
        f.write('  <mu1>0.2</mu1>\n')
        f.write('  <mu2>0.2</mu2>\n')
        f.write('  <selfCollide>true</selfCollide>\n')
        f.write('  <gravity>true</gravity>\n')
        f.write('</gazebo>\n')
        f.write('\n')

        # others
        for joint in joints_dict:
            name = joints_dict[joint]['child']
            f.write('<gazebo reference="{}">\n'.format(name))
            f.write('  <material>${body_color}</material>\n')
            f.write('  <mu1>0.2</mu1>\n')
            f.write('  <mu2>0.2</mu2>\n')
            f.write('  <selfCollide>true</selfCollide>\n')
            f.write('</gazebo>\n')
            f.write('\n')

        f.write('</robot>\n')

def write_display_launch(package_name, robot_name, save_dir):
    """
    write display launch file "save_dir/launch/display.launch"


    Parameter
    ---------
    robot_name: str
    name of the robot
    save_dir: str
    path of the repository to save
    """   
    os.makedirs(os.path.join(save_dir, 'launch'), exist_ok=True)

    launch = Element('launch')     

    arg1 = SubElement(launch, 'arg')
    arg1.attrib = {'name':'model', 'default':'$(find {})/urdf/{}.xacro'.format(package_name, robot_name)}

    arg2 = SubElement(launch, 'arg')
    arg2.attrib = {'name':'gui', 'default':'true'}

    arg3 = SubElement(launch, 'arg')
    arg3.attrib = {'name':'rvizconfig', 'default':'$(find {})/launch/urdf.rviz'.format(package_name)}

    param1 = SubElement(launch, 'param')
    param1.attrib = {'name':'robot_description', 'command':'$(find xacro)/xacro $(arg model)'}

    param2 = SubElement(launch, 'param')
    param2.attrib = {'name':'use_gui', 'value':'$(arg gui)'}

    node1 = SubElement(launch, 'node')
    node1.attrib = {'name':'joint_state_publisher_gui', 'pkg':'joint_state_publisher_gui', 'type':'joint_state_publisher_gui'}

    node2 = SubElement(launch, 'node')
    node2.attrib = {'name':'robot_state_publisher', 'pkg':'robot_state_publisher', 'type':'robot_state_publisher'}

    node3 = SubElement(launch, 'node')
    node3.attrib = {'name':'rviz', 'pkg':'rviz', 'args':'-d $(arg rvizconfig)', 'type':'rviz', 'required':'true'}

    launch_xml = "\n".join(utils.prettify(launch).split("\n")[1:])        

    file_name = os.path.join(save_dir, 'launch', 'display.launch')    
    with open(file_name, mode='w') as f:
        f.write(launch_xml)

def write_gazebo_launch(package_name, robot_name, save_dir):
    """
    write gazebo launch file "save_dir/launch/gazebo.launch"
    
    
    Parameter
    ---------
    robot_name: str
        name of the robot
    save_dir: str
        path of the repository to save
    """
    
    os.makedirs(os.path.join(save_dir, 'launch'), exist_ok=True)
    
    launch = Element('launch')
    param = SubElement(launch, 'param')
    param.attrib = {'name':'robot_description', 'command':'$(find xacro)/xacro $(find {})/urdf/{}.xacro'.format(package_name, robot_name)}

    node = SubElement(launch, 'node')
    node.attrib = {'name':'spawn_urdf', 'pkg':'gazebo_ros', 'type':'spawn_model',\
                    'args':'-param robot_description -urdf -model {}'.format(robot_name)}

    include_ =  SubElement(launch, 'include')
    include_.attrib = {'file':'$(find gazebo_ros)/launch/empty_world.launch'}        
    
    number_of_args = 5
    args = [None for i in range(number_of_args)]
    args_name_value_pairs = [['paused', 'true'], ['use_sim_time', 'true'],
                             ['gui', 'true'], ['headless', 'false'], 
                             ['debug', 'false']]
                             
    for i, arg in enumerate(args):
        arg = SubElement(include_, 'arg')
        arg.attrib = {'name' : args_name_value_pairs[i][0] , 
        'value' : args_name_value_pairs[i][1]}


    
    launch_xml = "\n".join(utils.prettify(launch).split("\n")[1:])        
    
    file_name = os.path.join(save_dir, 'launch', 'gazebo.launch')    
    with open(file_name, mode='w') as f:
        f.write(launch_xml)


def write_control_launch(package_name, robot_name, save_dir, joints_dict):
    """
    write control launch file "save_dir/launch/controller.launch"
    
    
    Parameter
    ---------
    robot_name: str
        name of the robot
    save_dir: str
        path of the repository to save
    joints_dict: dict
        information of the joints
    """
    
    os.makedirs(os.path.join(save_dir, 'launch'), exist_ok=True)
    
    #launch = Element('launch')

    controller_name = robot_name + '_controller'
    #rosparam = SubElement(launch, 'rosparam')
    #rosparam.attrib = {'file':'$(find {})/launch/controller.yaml'.format(package_name),
    #                   'command':'load'}
                       
    controller_args_str = ""
    for j in joints_dict:
        joint_type = joints_dict[j]['type']
        if joint_type != 'fixed':
            controller_args_str += j + '_position_controller '
    controller_args_str += 'joint_state_controller '

    node_controller = Element('node')
    node_controller.attrib = {'name':'controller_spawner', 'pkg':'controller_manager', 'type':'spawner',\
                    'respawn':'false', 'output':'screen', 'ns':robot_name,\
                    'args':'{}'.format(controller_args_str)}
    
    node_publisher = Element('node')
    node_publisher.attrib = {'name':'robot_state_publisher', 'pkg':'robot_state_publisher',\
                    'type':'robot_state_publisher', 'respawn':'false', 'output':'screen'}
    remap = SubElement(node_publisher, 'remap')
    remap.attrib = {'from':'/joint_states',\
                    'to':'/' + robot_name + '/joint_states'}
    
    #launch_xml  = "\n".join(utils.prettify(launch).split("\n")[1:])   
    launch_xml  = "\n".join(utils.prettify(node_controller).split("\n")[1:])   
    launch_xml += "\n".join(utils.prettify(node_publisher).split("\n")[1:])   

    file_name = os.path.join(save_dir, 'launch', 'controller.launch')    
    with open(file_name, mode='w') as f:
        f.write('<launch>\n')
        f.write('\n')
        #for some reason ROS is very picky about the attribute ordering, so we'll bitbang this element
        f.write('<rosparam file="$(find {})/launch/controller.yaml" command="load"/>'.format(package_name))
        f.write('\n')
        f.write(launch_xml)
        f.write('\n')
        f.write('</launch>')
        

def write_yaml(package_name, robot_name, save_dir, joints_dict):
    """
    write yaml file "save_dir/launch/controller.yaml"


    Parameter
    ---------
    robot_name: str
        name of the robot
    save_dir: str
        path of the repository to save
    joints_dict: dict
        information of the joints
    """
    os.makedirs(os.path.join(save_dir, 'launch'), exist_ok=True)

    controller_name = robot_name + '_controller'
    file_name = os.path.join(save_dir, 'launch', 'controller.yaml')
    with open(file_name, 'w') as f:
        f.write(controller_name + ':\n')
        # joint_state_controller
        f.write('  # Publish all joint states -----------------------------------\n')
        f.write('  joint_state_controller:\n')
        f.write('    type: joint_state_controller/JointStateController\n')
        f.write('    publish_rate: 50\n\n')
        # position_controllers
        f.write('  # Position Controllers --------------------------------------\n')
        for joint in joints_dict:
            joint_type = joints_dict[joint]['type']
            if joint_type != 'fixed':
                f.write('  ' + joint + '_position_controller:\n')
                f.write('    type: effort_controllers/JointPositionController\n')
                f.write('    joint: '+ joint + '\n')
                f.write('    pid: {p: 100.0, i: 0.01, d: 10.0}\n')


# ---------------------------------------------------------------------------
# ROS2 output (dheena2k2 / newtonjeri / runtimerobotics direction, kept
# alongside ROS1 so one export serves both ecosystems).
# ---------------------------------------------------------------------------

def write_ros2_control_xacro(joints_dict, package_name, robot_name, save_dir):
    """Write ros2_control xacro: command interfaces for movable joints."""
    os.makedirs(os.path.join(save_dir, 'urdf'), exist_ok=True)
    file_name = os.path.join(save_dir, 'urdf', robot_name + '.ros2_control.xacro')
    movable = [(name, joints_dict[name]) for name in joints_dict
               if joints_dict[name].get('type') != 'fixed']
    with open(file_name, 'w') as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write('<robot name="{}" xmlns:xacro="http://www.ros.org/wiki/xacro">\n'.format(robot_name))
        f.write('  <ros2_control name="{}_controller" type="system">\n'.format(robot_name))
        f.write('    <hardware>\n')
        f.write('      <plugin>gazebo_ros2_control/GazeboSystem</plugin>\n')
        f.write('    </hardware>\n')
        for name, joint in movable:
            f.write('    <joint name="{}">\n'.format(name))
            f.write('      <command_interface name="position"/>\n')
            f.write('      <state_interface name="position"/>\n')
            f.write('      <state_interface name="velocity"/>\n')
            f.write('    </joint>\n')
        f.write('  </ros2_control>\n')
        f.write('</robot>\n')
    return file_name


def write_ros2_controllers_yaml(package_name, robot_name, save_dir, joints_dict):
    """Write ROS2 controllers config (joint_state_broadcaster + JTC)."""
    os.makedirs(os.path.join(save_dir, 'config'), exist_ok=True)
    movable = [name for name in joints_dict if joints_dict[name].get('type') != 'fixed']
    file_name = os.path.join(save_dir, 'config', 'controllers.yaml')
    with open(file_name, 'w') as f:
        f.write('controller_manager:\n')
        f.write('  ros__parameters:\n')
        f.write('    update_rate: 100\n')
        f.write('    joint_state_broadcaster:\n')
        f.write('      type: joint_state_broadcaster/JointStateBroadcaster\n')
        if movable:
            f.write('    joint_trajectory_controller:\n')
            f.write('      type: joint_trajectory_controller/JointTrajectoryController\n')
            f.write('      joints:\n')
            for name in movable:
                f.write('        - {}\n'.format(name))
            f.write('      command_interfaces:\n')
            f.write('        - position\n')
            f.write('      state_interfaces:\n')
            f.write('        - position\n')
            f.write('        - velocity\n')
    return file_name


def write_ros2_display_launch_py(package_name, robot_name, save_dir):
    """Write ROS2 RViz display launch file."""
    os.makedirs(os.path.join(save_dir, 'launch'), exist_ok=True)
    file_name = os.path.join(save_dir, 'launch', 'display.launch.py')
    with open(file_name, 'w') as f:
        f.write('from launch import LaunchDescription\n')
        f.write('from launch.actions import DeclareLaunchArgument\n')
        f.write('from launch.substitutions import LaunchConfiguration\n')
        f.write('from launch_ros.actions import Node\n')
        f.write('from launch_ros.parameter_descriptions import ParameterValue\n')
        f.write('from launch.substitutions import Command\n')
        f.write('import os\n')
        f.write('from ament_index_python.packages import get_package_share_directory\n')
        f.write('\n\n')
        f.write('def generate_launch_description():\n')
        f.write("    pkg = get_package_share_directory('%s')\n" % package_name)
        f.write("    xacro_file = os.path.join(pkg, 'urdf', '%s.xacro')\n" % robot_name)
        f.write('    robot_description = ParameterValue(Command(["xacro ", xacro_file]), value_type=str)\n')
        f.write('    return LaunchDescription([\n')
        f.write("        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui', name='joint_state_publisher_gui'),\n")
        f.write("        Node(package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher', parameters=[{'robot_description': robot_description}]),\n")
        f.write("        Node(package='rviz2', executable='rviz2', name='rviz2', arguments=['-d', os.path.join(pkg, 'launch', 'urdf.rviz')]),\n")
        f.write('    ])\n')
    return file_name


def write_ros2_gazebo_launch_py(package_name, robot_name, save_dir):
    """Write ROS2 Gazebo launch file."""
    os.makedirs(os.path.join(save_dir, 'launch'), exist_ok=True)
    file_name = os.path.join(save_dir, 'launch', 'gazebo.launch.py')
    with open(file_name, 'w') as f:
        f.write('from launch import LaunchDescription\n')
        f.write('from launch.actions import IncludeLaunchDescription, ExecuteProcess\n')
        f.write('from launch.launch_description_sources import PythonLaunchDescriptionSource\n')
        f.write('from launch_ros.actions import Node\n')
        f.write('from launch_ros.parameter_descriptions import ParameterValue\n')
        f.write('from launch.substitutions import Command\n')
        f.write('import os\n')
        f.write('from ament_index_python.packages import get_package_share_directory\n')
        f.write('\n\n')
        f.write('def generate_launch_description():\n')
        f.write("    pkg = get_package_share_directory('%s')\n" % package_name)
        f.write("    ros_gz_sim = get_package_share_directory('ros_gz_sim')\n")
        f.write("    xacro_file = os.path.join(pkg, 'urdf', '%s.xacro')\n" % robot_name)
        f.write('    robot_description = ParameterValue(Command(["xacro ", xacro_file]), value_type=str)\n')
        f.write('    return LaunchDescription([\n')
        f.write("        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py'))),\n")
        f.write("        Node(package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher', parameters=[{'robot_description': robot_description}]),\n")
        f.write("        Node(package='ros_gz_sim', executable='create', name='spawn_urdf', arguments=['-topic', 'robot_description', '-name', '%s']),\n" % robot_name)
        f.write('    ])\n')
    return file_name


def write_ros2_package_files(package_name, robot_name, save_dir):
    """Write minimal ament_cmake package.xml + CMakeLists.txt for ROS2."""
    os.makedirs(save_dir, exist_ok=True)
    package_xml = os.path.join(save_dir, 'package.xml')
    with open(package_xml, 'w') as f:
        f.write('<?xml version="1.0"?>\n')
        f.write('<package format="3">\n')
        f.write('  <name>%s</name>\n' % package_name)
        f.write('  <version>0.0.0</version>\n')
        f.write('  <description>The %s package</description>\n' % package_name)
        f.write('  <maintainer email="todo@todo.todo">todo</maintainer>\n')
        f.write('  <license>MIT</license>\n')
        f.write('  <buildtool_depend>ament_cmake</buildtool_depend>\n')
        f.write('  <depend>rclcpp</depend>\n')
        f.write('  <depend>urdf</depend>\n')
        f.write('  <depend>xacro</depend>\n')
        f.write('  <exec_depend>robot_state_publisher</exec_depend>\n')
        f.write('  <exec_depend>joint_state_publisher_gui</exec_depend>\n')
        f.write('  <exec_depend>ros_gz_sim</exec_depend>\n')
        f.write('  <export>\n')
        f.write('    <build_type>ament_cmake</build_type>\n')
        f.write('  </export>\n')
        f.write('</package>\n')
    cmake_lists = os.path.join(save_dir, 'CMakeLists.txt')
    with open(cmake_lists, 'w') as f:
        f.write('cmake_minimum_required(VERSION 3.8)\n')
        f.write('project(%s)\n' % package_name)
        f.write('find_package(ament_cmake REQUIRED)\n')
        f.write('install(DIRECTORY urdf meshes launch config DESTINATION share/${PROJECT_NAME})\n')
        f.write('ament_package()\n')
    return package_xml, cmake_lists


def write_robot_data_yaml(package_name, robot_name, save_dir, joints_dict, closing_joints,
                          warnings=None):
    """Write robot_data.yaml sidecar: loop-closing joints + export metadata.

    Adriaeik's approach: URDF stays a valid tree; the loop-closing joint
    goes to downstream tools (Isaac Sim, MuJoCo) instead of silently
    breaking the export.
    """
    os.makedirs(os.path.join(save_dir, 'config'), exist_ok=True)
    file_name = os.path.join(save_dir, 'config', 'robot_data.yaml')
    with open(file_name, 'w') as f:
        f.write('# Generated by fusion360tourdf. URDF is a tree; joints that\n')
        f.write('# would close a kinematic loop are listed here for downstream\n')
        f.write('# tools (Isaac Sim / MuJoCo / custom pipelines).\n')
        f.write('robot_name: %s\n' % robot_name)
        f.write('package_name: %s\n' % package_name)
        f.write('links:\n')
        links = ['base_link'] + [joints_dict[j]['child'] for j in joints_dict]
        for link in links:
            f.write('  - %s\n' % link)
        f.write('joints:\n')
        for name in joints_dict:
            joint = joints_dict[name]
            f.write('  - name: %s\n' % name)
            f.write('    parent: %s\n' % joint.get('parent'))
            f.write('    child: %s\n' % joint.get('child'))
            f.write('    type: %s\n' % joint.get('type'))
        f.write('closing_joints:\n')
        if not closing_joints:
            f.write('  []\n')
        for entry in closing_joints:
            f.write('  - joint: %s\n' % entry.get('joint'))
            f.write('    parent: %s\n' % entry.get('parent'))
            f.write('    child: %s\n' % entry.get('child'))
            f.write('    source: %s\n' % entry.get('source', 'auto_detected'))
        if warnings:
            f.write('warnings:\n')
            for warning in warnings:
                f.write('  - %s\n' % str(warning).replace('\n', ' '))
    return file_name

