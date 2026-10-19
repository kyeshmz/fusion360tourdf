"""Create a native Fusion mechanism and an integrity-checked end-to-end test bundle."""

from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path
import sys
import tempfile
import traceback

import adsk.core
import adsk.fusion

REPOSITORY = Path(__file__).resolve().parents[2]
if str(REPOSITORY) not in sys.path:
    sys.path.insert(0, str(REPOSITORY))

from tools.export_bundle import bundle_hashes
from tools.verify_fusion.verification import (
    Box, Checks, IDENTITY, Joint, QUARTER_TURN, assert_entity_identity,
    assert_source_unchanged, capture_provenance, loaded_code_provenance,
    pose_expectations, source_hashes, verify_package,
)
from tools.verify_fusion.verify_fusion import (
    _add_box, _component, _expectations, _matrix, _name, _rigid, source_snapshot,
)


def _motion(root, child, parent, name, kind, direction, limits):
    point = child.component.originConstructionPoint.createForAssemblyContext(child)
    geometry = adsk.fusion.JointGeometry.createByPoint(point)
    joint_input = root.asBuiltJoints.createInput(child, parent, geometry)
    configured = (joint_input.setAsRevoluteJointMotion(direction) if kind == 'revolute'
                  else joint_input.setAsSliderJointMotion(direction))
    if not configured:
        raise RuntimeError('Fusion refused ' + name)
    joint = root.asBuiltJoints.add(joint_input)
    joint.name = name
    motion = joint.jointMotion
    bounds = motion.rotationLimits if kind == 'revolute' else motion.slideLimits
    bounds.minimumValue, bounds.maximumValue = limits
    bounds.isMinimumValueEnabled = bounds.isMaximumValueEnabled = True
    joint.isLightBulbOn = False
    return joint


def _build(design):
    root = design.rootComponent
    parameter = design.userParameters.add('base_thickness', adsk.core.ValueInput.createByString('20 mm'),
                                          'cm', 'Editable plinth thickness')
    sketch = root.sketches.add(root.xYConstructionPlane)
    sketch.name = 'BaseRectangle'
    sketch.sketchCurves.sketchLines.addTwoPointRectangle(adsk.core.Point3D.create(-12, -9, 0),
                                                        adsk.core.Point3D.create(12, 9, 0))
    extrusion = root.features.extrudeFeatures.addSimple(sketch.profiles.item(0),
        adsk.core.ValueInput.createByString(parameter.name), adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    extrusion.name = 'EditableBase'
    extrusion.bodies.item(0).name = 'plinth'
    sketch.isLightBulbOn = False
    base = Box((.24, .18, .02), (0, 0, .01))
    bindings = [('base_link', root, 'plinth', base)]
    hidden = _add_box(design, root, 'hidden_service_jig', Box((.1, .1, .1), (.8, .8, .8)))
    hidden.isLightBulbOn = False

    pivot = (-.07, 0, .24)
    mast = _component(root, 'mast', (-7, 0, 24))
    mast.isGroundToParent = True
    _add_box(design, mast.component, 'mast_solid', Box((.04, .05, .22), (0, 0, -.11)))
    bindings.append((_name(mast), mast, 'mast_solid', Box((.04, .05, .22), (-.07, 0, .13))))
    boom = _component(root, 'boom', (-7, 0, 24), QUARTER_TURN)
    _add_box(design, boom.component, 'rail', Box((.18, .035, .025), (.075, 0, .015)))
    bindings.append((_name(boom), boom, 'rail', Box((.18, .035, .025), (-.07, .075, .255), QUARTER_TURN)))
    _motion(root, boom, mast, 'turn', 'revolute', adsk.fusion.JointDirections.ZAxisJointDirection,
            (-math.pi/3, math.pi/2))
    joints = [Joint('rigid_attachment_' + _name(mast), 'base_link', _name(mast), 'fixed'),
              Joint('turn', _name(mast), _name(boom), 'revolute', limits=(-math.pi/3, math.pi/2),
                    pivot_world=pivot, sample_position=math.pi/4)]
    frames = {'base_link': (IDENTITY, (0, 0, 0)), _name(mast): (IDENTITY, pivot),
              _name(boom): (QUARTER_TURN, pivot)}

    carriage = _component(boom.component, 'carriage', (14, 0, 1.5))
    _add_box(design, carriage.component, 'carriage_solid', Box((.04, .055, .04), (0, 0, .02)))
    carriage_proxy = carriage.createForAssemblyContext(boom)
    carriage_origin = (-.07, .14, .255)
    bindings.append((_name(carriage_proxy), carriage_proxy, 'carriage_solid',
                     Box((.04, .055, .04), (-.07, .14, .275), QUARTER_TURN)))
    _motion(root, carriage_proxy, boom, 'extend', 'prismatic', adsk.fusion.JointDirections.XAxisJointDirection,
            (-2, 4))
    joints.append(Joint(_name(boom) + '_extend', _name(boom), _name(carriage_proxy), 'prismatic', axis_world=(0, 1, 0),
                        limits=(-.02, .04), sample_position=.025))
    frames[_name(carriage_proxy)] = (QUARTER_TURN, carriage_origin)

    sensor = _component(carriage.component, 'sensor', (0, -4, 2))
    sensor_box = Box((.025, .025, .018), (0, 0, .009))
    _add_box(design, sensor.component, 'sensor_solid', sensor_box)
    repeat = carriage.component.occurrences.addExistingComponent(sensor.component, _matrix((0, 4, 2)))
    repeat.isGroundToParent = False
    repeat_proxy = repeat.createForAssemblyContext(carriage_proxy)
    repeat_proxy.transform2 = _matrix((-11, 14, 27.5), QUARTER_TURN)
    if design.snapshots.hasPendingSnapshot:
        design.snapshots.add()
    for index, (native, center) in enumerate(((sensor, (-.03, .14, .284)), (repeat, (-.11, .14, .284))), 1):
        proxy = native.createForAssemblyContext(carriage_proxy)
        joints.append(_rigid(root, proxy, carriage_proxy, 'sensor_mount_' + str(index), context=carriage_proxy))
        bindings.append((_name(proxy), proxy, 'sensor_solid', replace(sensor_box, center_m=center, rotation=QUARTER_TURN)))
        frames[_name(proxy)] = (QUARTER_TURN, carriage_origin)
    return bindings, joints, frames


def capture_image(app, filename):
    """Capture the current model with a consistent Z-up camera, without editing geometry."""
    viewport = app.activeViewport
    camera = viewport.camera
    camera.eye = adsk.core.Point3D.create(40, -50, 45)
    camera.target = adsk.core.Point3D.create(-4, 6, 14)
    camera.upVector = adsk.core.Vector3D.create(0, 0, 1)
    camera.isFitView = True
    camera.isSmoothTransition = False
    viewport.camera = camera
    viewport.refresh()
    filename = Path(filename).resolve()
    filename.parent.mkdir(parents=True, exist_ok=True)
    if not viewport.saveAsImageFile(str(filename), 1280, 960):
        raise RuntimeError('Fusion viewport image capture failed: ' + str(filename))
    return dict(path=filename.name, width=1280, height=960,
                sha256=hashlib.sha256(filename.read_bytes()).hexdigest())


def create_demo(app, output_parent):
    """Leave the new unsaved demo open on success; never alter an existing document."""
    from URDF_Exporter.URDF_Exporter import export_design
    from URDF_Exporter.utils.config import load_config

    directory = Path(tempfile.mkdtemp(prefix='rotary-slide-', dir=str(output_parent)))
    package_name = 'rotary_slide_description'
    config = load_config(overrides={'root_occurrence': 'design_root'})
    report = dict(status='running', evidence_kind='native-fusion', fusion_version=app.version,
                  package_name=package_name, config=config, assertions=[],
                  provenance=capture_provenance(REPOSITORY, config))
    path = directory / 'native.json'

    def save():
        path.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n', encoding='utf-8')

    save()
    previous = app.activeDocument
    previous_modified = previous.isModified if previous else None
    document = None
    try:
        modules = {name: module for name, module in list(sys.modules.items())
                   if name.startswith(('URDF_Exporter.', 'tools.verify_fusion.', 'tools.create_demo.', 'tools.export_bundle'))
                   and getattr(module, '__file__', None)}
        report['loaded_code'] = {name: loaded_code_provenance(vars(module), module.__file__)
                                 for name, module in modules.items()}
        report['loaded_code']['native_runner'] = loaded_code_provenance(globals(), __file__)
        for name, code in report['loaded_code'].items():
            if code['status'] != 'match' or not Path(code['path']).is_relative_to(REPOSITORY):
                raise RuntimeError('Stale or foreign code loaded; restart Fusion: ' + name)
        document = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        if document is None or document == previous:
            document = None
            raise RuntimeError('Fusion did not create a distinct demo document')
        document.name = 'Rotary Slide Demo'
        design = adsk.fusion.Design.cast(app.activeProduct)
        design.designType = adsk.fusion.DesignTypes.ParametricDesignType
        bindings, joints, frames = _build(design)
        checks = Checks(report['assertions'])
        boxes, cad_bounds = _expectations(bindings, checks)
        report['expected_boxes_si'] = {name: [vars(box) for box in entries] for name, entries in boxes.items()}
        report['expected_joints_si'] = [vars(joint) for joint in joints]
        report['direct_cad_bounds_m'] = cad_bounds
        expected = pose_expectations(boxes, joints, frames)
        (directory / 'expected.json').write_text(json.dumps(expected, indent=2) + '\n')
        retained = []
        report['source_before'] = source_snapshot(design, retained)
        save()
        try:
            export_design(design, directory / package_name, config=config)
            archive = design.exportManager.createFusionArchiveExportOptions(str(directory / 'assembly.f3d'))
            if not design.exportManager.execute(archive):
                raise RuntimeError('Fusion archive export failed')
        finally:
            current = []
            report['source_after'] = source_snapshot(design, current)
            save()
            report['source_preservation'] = assert_source_unchanged(report['source_before'], report['source_after'])
            report['entity_identity'] = assert_entity_identity(retained, current)
        verify_package(directory / package_name, boxes, cad_bounds, joints, checks)
        report['outputs_sha256'] = bundle_hashes(directory, package_name)
        if source_hashes(REPOSITORY) != report['provenance']['inputs_sha256']:
            raise RuntimeError('Source files changed during native export')
        if previous and previous.isModified != previous_modified:
            raise RuntimeError('Previously active document modification state changed')
        report['images'] = {'fusion': capture_image(app, directory / 'fusion.png')}
        report['status'] = 'pass'
    except Exception:
        report.update(status='fail', traceback=traceback.format_exc())
        if document is not None and document.isValid:
            document.close(False)
        if previous is not None and previous.isValid:
            previous.activate()
        raise
    finally:
        save()
    return directory


def run(context):
    app = adsk.core.Application.get()
    dialog = app.userInterface.createFolderDialog()
    dialog.title = 'Choose parent folder for a new native demo and E2E bundle'
    if dialog.showDialog() != adsk.core.DialogResults.DialogOK:
        return
    directory = create_demo(app, dialog.folder)
    app.userInterface.messageBox('Created and checked Rotary Slide Demo.\nBundle: ' + str(directory) +
        '\nNext: python3 tools/verify_export.py "' + str(directory) + '" --output artifacts', 'fusion360tourdf')
