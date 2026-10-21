# Original exporter by syuntoku14. See the repository MIT license.
import adsk.core
import adsk.fusion
from pathlib import Path
import traceback
import re

from .utils import utils
from .utils import config as config_mod
from .core import Link, Joint, Write


def export_design(design, destination, *, config=None, cancelled=None):
    """Extract the current Fusion pose and publish a fresh ROS 2 package."""
    destination = Path(destination).absolute()
    if not re.fullmatch(r'[a-z][a-z0-9_]*_description', destination.name):
        raise ValueError('Destination must be a fresh <robot>_description directory with a lowercase ROS name.')
    if destination.exists() or destination.is_symlink():
        raise FileExistsError(destination)
    robot_name = destination.name[:-len('_description')]
    config = config_mod.load_config(overrides=config)
    root = design.rootComponent
    joint_records = utils.root_joints(root, 'root inference and joint extraction')
    sources = utils.link_sources(root, config=config, joint_records=joint_records, cancelled=cancelled)
    success = 'ok'
    source_notes = []
    joints, message = Joint.make_joints_dict(root, success, config=config, sources=sources,
                                           source_notes=source_notes, joint_records=joint_records)
    if message != success:
        raise ValueError(message)
    inertial, message = Link.make_inertial_dict(sources, success)
    if message != success:
        raise ValueError(message)
    inertial.update(getattr(joints, 'frame_records', {}))
    if config.get('visual_materials', 'legacy') != 'legacy':
        source_notes.append({'name': 'visual appearance', 'source': 'selected direct native face snapshots',
            'reason': 'Ancestor appearance precedence and inherited visibleOpacity are not live-Fusion verified. '
                      'Only the supplied occurrence override and current face/body appearances are read.'})
    if source_notes:
        inertial['base_link']['source_notes'] = source_notes
    return Write.write_ros2_package(destination, robot_name, joints, inertial, cancelled=cancelled,
        write_meshes=lambda targets: utils.export_stl(design, targets, config=config, sources=sources, cancelled=cancelled),
        config=config, visual_assets=utils.collect_visual_assets(sources, config))


def run(context):
    ui = None
    title = 'fusion360tourdf'
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox('No active Fusion design', title)
            return
        config = config_mod.load_config(Path(__file__).resolve().parent)
        root = design.rootComponent
        robot_name = utils.sanitize_name(root.name.split()[0]).lower()
        folder = utils.file_dialog(ui)
        if folder is False:
            ui.messageBox('fusion360tourdf was canceled', title)
            return
        success = 'Successfully created ROS 2 description package'
        destination = export_design(design, Path(folder) / (robot_name + '_description'), config=config)
        ui.messageBox(success + '\n' + str(destination), title)
    except Exception:
        if ui:
            ui.messageBox('Export failed:\n' + traceback.format_exc(), title)
        else:
            raise
