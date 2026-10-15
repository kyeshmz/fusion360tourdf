#Author-syuntoku14
#Description-Generate URDF file from Fusion 360

import adsk, adsk.core, adsk.fusion, traceback
import os
import sys
from .utils import utils
from .utils import config as config_mod
from .utils import validate as validate_mod
from .core import Link, Joint, Write

"""
# length unit is 'cm' and inertial unit is 'kg/cm^2'
# If there is no 'body' in the root component, maybe the corrdinates are wrong.
"""

# joint effort/velocity defaults live in xacro_export.toml (see template).
# supports "Revolute", "Rigid" and "Slider" joint types

# I'm not sure how prismatic joint acts if there is no limit in fusion model

def run(context):
    ui = None
    success_msg = 'Successfully create URDF file'
    msg = success_msg

    try:
        # --------------------
        # initialize
        app = adsk.core.Application.get()
        ui = app.userInterface
        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        title = 'fusion360tourdf'
        if not design:
            ui.messageBox('No active Fusion design', title)
            return

        root = design.rootComponent  # root component
        components = design.allComponents

        # Export config (xacro_export.toml next to this script, else defaults).
        try:
            script_dir = os.path.abspath(os.path.dirname(__file__))
        except Exception:
            script_dir = None
        config = config_mod.load_config(script_dir)

        # set the names
        robot_name = utils.sanitize_name(root.name.split()[0])
        package_name = robot_name + '_description'
        save_dir = utils.file_dialog(ui)
        if save_dir == False:
            ui.messageBox('fusion360tourdf was canceled', title)
            return 0

        save_dir = os.path.join(save_dir, package_name)
        os.makedirs(save_dir, exist_ok=True)

        package_dir = os.path.join(os.path.abspath(os.path.dirname(__file__)), 'package')

        # --------------------
        # set dictionaries

        # Generate joints_dict. All joints are related to root.
        joints_dict, msg = Joint.make_joints_dict(root, msg, config=config)
        if msg != success_msg:
            ui.messageBox(msg, title)
            return 0

        # Generate inertial_dict
        inertial_dict, msg = Link.make_inertial_dict(root, msg)
        if msg != success_msg:
            ui.messageBox(msg, title)
            return 0
        elif not 'base_link' in inertial_dict:
            msg = 'There is no base_link. Please set base_link and run again.'
            ui.messageBox(msg, title)
            return 0

        # Fail-closed preflight BEFORE mesh export burns time, and split
        # loop-closing joints into the sidecar (URDF stays a valid tree).
        tree_joints, closing_joints, loop_warnings = validate_mod.detect_closing_joints(joints_dict)
        errors, warnings = validate_mod.validate_export(tree_joints, inertial_dict)
        warnings = list(loop_warnings) + list(warnings)
        if errors:
            ui.messageBox('Export aborted:\n- ' + '\n- '.join(errors), title)
            return 0
        if warnings:
            ui.messageBox('Export warnings (continuing):\n- ' + '\n- '.join(warnings), title)
        joints_dict = tree_joints

        links_xyz_dict = {}
        links_rpy_dict = {}

        # --------------------
        # Generate URDF
        link_frames, links_rpy_dict = Write.write_urdf(
            joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir,
            config=config, links_rpy_dict=links_rpy_dict)
        Write.write_materials_xacro(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir)
        if config_mod.wants_ros1(config) and config.get('export_transmissions_ros1'):
            Write.write_transmissions_xacro(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir,
                                            config=config, link_frames=link_frames)
            Write.write_gazebo_xacro(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir)
            Write.write_display_launch(package_name, robot_name, save_dir)
            Write.write_gazebo_launch(package_name, robot_name, save_dir)
            Write.write_control_launch(package_name, robot_name, save_dir, joints_dict)
            Write.write_yaml(package_name, robot_name, save_dir, joints_dict)
            # ROS1 static template (copied before ROS2 so 'both' ends ROS2).
            utils.copy_package(save_dir, package_dir)
            utils.update_cmakelists(save_dir, package_name)
            utils.update_package_xml(save_dir, package_name)
        if config_mod.wants_ros2(config):
            if config.get('export_ros2_control'):
                Write.write_ros2_control_xacro(joints_dict, package_name, robot_name, save_dir)
            Write.write_ros2_controllers_yaml(package_name, robot_name, save_dir, joints_dict)
            Write.write_ros2_display_launch_py(package_name, robot_name, save_dir)
            Write.write_ros2_gazebo_launch_py(package_name, robot_name, save_dir)
            Write.write_ros2_package_files(package_name, robot_name, save_dir)
        Write.write_robot_data_yaml(package_name, robot_name, save_dir, joints_dict,
                                    closing_joints, warnings)

        # Generate STL files (non-destructive default; legacy copy behind config).
        if config.get('use_legacy_copy'):
            utils.copy_occs(root)
        utils.export_stl(design, save_dir, components, config=config)

        ui.messageBox(msg, title)

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
