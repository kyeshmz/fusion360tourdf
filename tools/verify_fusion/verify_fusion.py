
from dataclasses import replace
from datetime import datetime, timezone
import math
from pathlib import Path
import re
import sys
import tempfile
import traceback

import adsk.core
import adsk.fusion

from .verification import (Box, Checks, IDENTITY, Joint, QUARTER_TURN, Report,
                           assert_entity_identity, assert_source_unchanged, bounds,
                           capture_provenance, entity_evidence, loaded_code_provenance,
                           owned_document, source_hashes, verify_package)


FIXTURES = ('root_bodies', 'rigid_as_built', 'ordinary_revolute', 'ordinary_slider', 'nested_repeated')
MODE_FIXTURES = {'parametric': FIXTURES + ('parametric_history',),
                 'direct': ('root_bodies', 'grounded_assembly', 'ordinary_revolute', 'ordinary_slider', 'nested_grounded')}
BASE = Box((.02, .04, .06), (.01, .02, .03))
BLOCK = Box((.03, .05, .07), (.02, .03, .04))


def _name(occurrence):
    return re.sub(r'[^A-Za-z0-9_]+', '_', occurrence.fullPathName).strip('_')


def _matrix(translation_cm=(0, 0, 0), rotation=IDENTITY):
    matrix = adsk.core.Matrix3D.create()
    for row in range(3):
        for column in range(3):
            matrix.setCell(row, column, rotation[3*row+column])
        matrix.setCell(row, 3, translation_cm[row])
    return matrix


def _component(parent, name, translation_cm=(0, 0, 0), rotation=IDENTITY):
    occurrence = parent.occurrences.addNewComponent(_matrix(translation_cm, rotation))
    occurrence.component.name = name
    occurrence.isGroundToParent = False
    return occurrence


def _add_box(design, component, name, box):
    oriented = adsk.core.OrientedBoundingBox3D.create(
        adsk.core.Point3D.create(*(100*v for v in box.center_m)),
        adsk.core.Vector3D.create(box.rotation[0], box.rotation[3], box.rotation[6]),
        adsk.core.Vector3D.create(box.rotation[1], box.rotation[4], box.rotation[7]),
        *(100*v for v in box.size_m))
    temporary = adsk.fusion.TemporaryBRepManager.get().createBox(oriented)
    if temporary is None:
        raise RuntimeError('Fusion could not create the analytic box ' + name)
    count = component.bRepBodies.count
    if design.designType == adsk.fusion.DesignTypes.ParametricDesignType:
        # Fusion requires an editing base feature when persisting temporary BRep.
        feature = component.features.baseFeatures.add()
        if not feature.startEdit():
            raise RuntimeError('Fusion refused to edit the owned base feature')
        try:
            if component.bRepBodies.add(temporary, feature) is None:
                raise RuntimeError('Fusion could not persist the analytic box')
        finally:
            if not feature.finishEdit():
                raise RuntimeError('Fusion refused to finish the owned base feature')
    elif component.bRepBodies.add(temporary) is None:
        raise RuntimeError('Fusion could not persist the direct analytic box')
    if component.bRepBodies.count != count + 1:
        raise RuntimeError('Unexpected body count after creating ' + name)
    body = component.bRepBodies.item(count)
    body.name = name
    return body


def _rigid(root, child, parent, name, *, context=None):
    joint_input = root.asBuiltJoints.createInput(child, parent, None)
    if not joint_input.setAsRigidJointMotion():
        raise RuntimeError('Fusion refused rigid as-built motion')
    joint = root.asBuiltJoints.add(joint_input)
    joint.name = name
    source_name = _name(context) + '_' + name if context is not None else name
    return Joint(source_name, 'base_link' if parent.component.name == 'base_link' else _name(parent),
                 _name(child), 'fixed')


def _ordinary(root, child, parent, fixture):
    child_point = child.component.originConstructionPoint.createForAssemblyContext(child)
    parent_point = parent.component.originConstructionPoint.createForAssemblyContext(parent)
    geometry_one = adsk.fusion.JointGeometry.createByPoint(child_point)
    geometry_two = adsk.fusion.JointGeometry.createByPoint(parent_point)
    joint_input = root.joints.createInput(geometry_one, geometry_two)
    joint_input.isFlipped = False
    joint_input.angle = adsk.core.ValueInput.createByReal(0)
    joint_input.offset = adsk.core.ValueInput.createByReal(0)
    direction = adsk.fusion.JointDirections.ZAxisJointDirection
    revolute = fixture == 'ordinary_revolute'
    configured = (joint_input.setAsRevoluteJointMotion(direction) if revolute
                  else joint_input.setAsSliderJointMotion(direction))
    if not configured:
        raise RuntimeError('Fusion refused the ordinary joint motion')
    joint = root.joints.add(joint_input)
    joint.name = 'hinge' if revolute else 'slide'
    motion = joint.jointMotion
    if revolute:
        motion = adsk.fusion.RevoluteJointMotion.cast(motion)
        motion.rotationValue = 0
        limits, native_limits = motion.rotationLimits, (-math.pi/3, math.pi/2)
        expected_limits = native_limits
    else:
        motion = adsk.fusion.SliderJointMotion.cast(motion)
        motion.slideValue = 0
        limits, native_limits = motion.slideLimits, (-2, 4)
        expected_limits = (-.02, .04)
    limits.minimumValue, limits.maximumValue = native_limits
    limits.isMinimumValueEnabled = True
    limits.isMaximumValueEnabled = True
    joint.isLightBulbOn = False
    return Joint(joint.name, 'base_link', _name(child), 'revolute' if revolute else 'prismatic',
                 limits=expected_limits, pivot_world=(0, 0, 0), sample_position=math.pi/4 if revolute else .02)


def _fixtures(design, fixture):
    root = design.rootComponent
    bindings, joints = [], []
    if fixture == 'parametric_history':
        parameter = design.userParameters.add('proof_height', adsk.core.ValueInput.createByString('60 mm'),
                                               'cm', 'Editable extrusion depth for source-preservation proof')
        if parameter is None:
            raise RuntimeError('Fusion refused the owned user parameter')
        sketch = root.sketches.add(root.xYConstructionPlane)
        sketch.name = 'ProofRectangle'
        sketch.sketchCurves.sketchLines.addTwoPointRectangle(adsk.core.Point3D.create(0, 0, 0),
                                                            adsk.core.Point3D.create(2, 4, 0))
        extrusion = root.features.extrudeFeatures.addSimple(sketch.profiles.item(0),
                        adsk.core.ValueInput.createByString('proof_height'), adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        if extrusion is None:
            raise RuntimeError('Fusion refused the owned parametric extrusion')
        extrusion.name = 'ProofExtrude'
        body = extrusion.bodies.item(0)
        body.name = 'editable_solid'
        return [('base_link', root, body.name, BASE)], joints
    if fixture == 'root_bodies':
        for name, box in (('first', BASE), ('second', Box((.01, .03, .05), (-.04, .02, .01)))):
            _add_box(design, root, name, box)
            bindings.append(('base_link', root, name, box))
        hidden = _add_box(design, root, 'hidden_solid', Box((.1, .2, .3), (.4, .5, .6)))
        hidden.isLightBulbOn = False
        return bindings, joints
    base = _component(root, 'base_link')
    _add_box(design, base.component, 'base_solid', BASE)
    base.isGrounded = True
    bindings.append(('base_link', base, 'base_solid', BASE))
    if fixture in ('ordinary_revolute', 'ordinary_slider'):
        child = _component(root, 'moving')
        box = replace(BLOCK, rotation=QUARTER_TURN)
        _add_box(design, child.component, 'moving_solid', box)
        joints.append(_ordinary(root, child, base, fixture))
        bindings.append((_name(child), child, 'moving_solid', box))
    elif fixture in ('rigid_as_built', 'grounded_assembly'):
        child = _component(root, 'rigid_child', (10, 20, 30), QUARTER_TURN)
        _add_box(design, child.component, 'rigid_solid', BLOCK)
        if fixture == 'grounded_assembly':
            child.isGroundToParent = True
            joints.append(Joint('rigid_attachment_' + _name(child), 'base_link', _name(child), 'fixed'))
        else:
            joints.append(_rigid(root, child, base, 'rigid_mount'))
        bindings.append((_name(child), child, 'rigid_solid', replace(BLOCK, center_m=(.07, .22, .34), rotation=QUARTER_TURN)))
    elif fixture in ('nested_repeated', 'nested_grounded'):
        carrier = _component(root, 'carrier', (10, 20, 30), QUARTER_TURN)
        _add_box(design, carrier.component, 'carrier_solid', BASE)
        nested = _component(carrier.component, 'block', (0, 10, 0), QUARTER_TURN)
        _add_box(design, nested.component, 'block_solid', BLOCK)
        child = nested.createForAssemblyContext(carrier)
        repeat = root.occurrences.addExistingComponent(nested.component, _matrix((30, 0, 10)))
        repeat.isGroundToParent = False
        if fixture == 'nested_grounded':
            repeat.transform2 = _matrix((30, 0, 10))
            for occurrence in (carrier, nested, repeat):
                occurrence.isGroundToParent = True
            joints.extend(Joint('rigid_attachment_' + _name(occurrence), 'base_link', _name(occurrence), 'fixed')
                          for occurrence in (carrier, child, repeat))
        else:
            joints.extend((_rigid(root, carrier, base, 'carrier_mount'),
                           _rigid(root, child, carrier, 'nested_mount', context=carrier),
                           _rigid(root, repeat, base, 'repeat_mount')))
        bindings.extend(((_name(carrier), carrier, 'carrier_solid', replace(BASE, center_m=(.08, .21, .33), rotation=QUARTER_TURN)),
                         (_name(child), child, 'block_solid', replace(BLOCK, center_m=(-.02, .17, .34), rotation=(-1, 0, 0, 0, -1, 0, 0, 0, 1))),
                         (_name(repeat), repeat, 'block_solid', replace(BLOCK, center_m=(.32, .03, .14)))))
    else:
        raise ValueError('Unknown fixture ' + fixture)
    return bindings, joints


def _body_bounds(body):
    box = body.boundingBox
    return tuple(v / 100 for v in box.minPoint.asArray()), tuple(v / 100 for v in box.maxPoint.asArray())


def _expectations(bindings, checks):
    boxes, cad_points = {}, {}
    for link, source, body_name, box in bindings:
        body = next(body for body in source.bRepBodies if body.name == body_name)
        native = body.nativeObject or body
        properties = native.getPhysicalProperties(adsk.fusion.CalculationAccuracy.VeryHighCalculationAccuracy)
        box = replace(box, mass_kg=properties.mass)
        boxes.setdefault(link, []).append(box)
        cad_points.setdefault(link, []).extend(_body_bounds(body))
        proxy_properties = body.getPhysicalProperties(adsk.fusion.CalculationAccuracy.VeryHighCalculationAccuracy)
        checks.close(link + ' direct CAD world COM', tuple(v / 100 for v in proxy_properties.centerOfMass.asArray()), box.center_m)
    return boxes, {name: bounds(points) for name, points in cad_points.items()}


def _body_state(body):
    return dict(name=body.name, component=body.parentComponent.name,
                is_visible=body.isVisible, light_bulb=body.isLightBulbOn,
                bounds_m=_body_bounds(body))


def source_snapshot(design, entities=None):
    entities = entities if entities is not None else []
    root = design.rootComponent
    timeline = None
    if design.designType == adsk.fusion.DesignTypes.ParametricDesignType:
        features = []
        for i in range(design.timeline.count):
            item = design.timeline.item(i)
            entity = item.entity or item
            signature = dict(index=item.index, name=item.name, is_suppressed=item.isSuppressed,
                             object_type=entity.objectType, entity_name=getattr(entity, 'name', None))
            if hasattr(item, 'isRolledBack'):
                signature['is_rolled_back'] = item.isRolledBack
            for attribute in ('isSuppressed', 'healthState', 'errorOrWarningMessage'):
                if hasattr(entity, attribute):
                    value = getattr(entity, attribute)
                    signature[attribute] = int(value) if attribute == 'healthState' else value
            features.append(signature)
            entities.append(('timeline[%d]' % i, entity))
        timeline = dict(marker=design.timeline.markerPosition, count=design.timeline.count, features=features)
    parametric = design.designType == adsk.fusion.DesignTypes.ParametricDesignType
    user_names = {parameter.name for parameter in design.userParameters} if parametric else set()
    parameters = []
    for parameter in design.allParameters if parametric else ():
        parameters.append(dict(name=parameter.name, expression=parameter.expression, unit=parameter.unit,
                               value=parameter.value, comment=parameter.comment, is_user=parameter.name in user_names))
        entities.append(('parameter[' + parameter.name + ']', parameter))
    occurrences = [dict(name=occurrence.name, path=occurrence.fullPathName,
                        component=occurrence.component.name, transform_cm=list(occurrence.transform2.asArray()),
                        is_visible=occurrence.isVisible, light_bulb=occurrence.isLightBulbOn,
                        is_grounded=occurrence.isGrounded, ground_to_parent=occurrence.isGroundToParent,
                        bodies=[_body_state(body) for body in occurrence.bRepBodies])
                   for occurrence in root.allOccurrences]
    joints = []
    for kind, collection in (('ordinary', root.allJoints), ('as_built', root.allAsBuiltJoints)):
        for joint in collection:
            motion = joint.jointMotion
            state = dict(kind=kind, name=joint.name, motion_type=int(motion.jointType),
                         occurrence_one=joint.occurrenceOne.fullPathName,
                         occurrence_two=joint.occurrenceTwo.fullPathName)
            for attribute in ('isSuppressed', 'isLightBulbOn', 'isLocked'):
                if hasattr(joint, attribute):
                    state[attribute] = getattr(joint, attribute)
            if motion.jointType == adsk.fusion.JointTypes.RevoluteJointType:
                motion = adsk.fusion.RevoluteJointMotion.cast(motion)
                state['rotation_value_rad'] = motion.rotationValue
                limits = motion.rotationLimits
            elif motion.jointType == adsk.fusion.JointTypes.SliderJointType:
                motion = adsk.fusion.SliderJointMotion.cast(motion)
                state['slide_value_cm'] = motion.slideValue
                limits = motion.slideLimits
            else:
                limits = None
            if limits is not None:
                state['native_limits'] = dict(minimum=limits.minimumValue, maximum=limits.maximumValue,
                                              minimum_enabled=limits.isMinimumValueEnabled,
                                              maximum_enabled=limits.isMaximumValueEnabled)
            joints.append(state)
    return dict(design_type=int(design.designType), design_intent=int(design.designIntent) if hasattr(design, 'designIntent') else None,
                timeline=timeline, parameters=parameters, root_name=root.name,
                root_bodies=[_body_state(body) for body in root.bRepBodies], occurrences=occurrences, joints=joints)


def run_verification(app, output_parent):
    """Run the production export on newly owned fixtures and retain every case report."""
    directory = Path(tempfile.mkdtemp(prefix='fusion-proof-', dir=str(output_parent)))
    report = Report(directory, app.version, sum(map(len, MODE_FIXTURES.values())))
    report.data['coverage'].append(dict(name='direct_as_built_creation', status='not_exercised',
        reason='Fusion only creates as-built joints in parametric designs. Direct fixed fixtures use native grounding instead.'))
    report.data['started_at_utc'] = datetime.now(timezone.utc).isoformat()
    report.data['repository'] = str(Path(__file__).resolve().parents[2])
    try:
        repository = str(Path(__file__).resolve().parents[2])
        report.data['provenance'] = capture_provenance(repository, None)
        provenance = report.data['provenance']
        provenance['preexisting_exporter_modules'] = sorted(name for name in sys.modules
            if name == 'URDF_Exporter' or name.startswith('URDF_Exporter.'))
        provenance['restart_required'] = 'Restart Fusion after updating modules and before each verification run. File hashes alone do not prove execution.'
        provenance['loaded_code'] = {
            'native_runner': loaded_code_provenance(globals(), __file__),
            'verification': loaded_code_provenance(vars(sys.modules[verify_package.__module__]),
                                                  sys.modules[verify_package.__module__].__file__)}
        report.save()
        if provenance['preexisting_exporter_modules']:
            raise RuntimeError('Exporter modules were already loaded. Restart Fusion before running the verifier.')
        if repository not in sys.path:
            sys.path.insert(0, repository)
        from URDF_Exporter.URDF_Exporter import export_design
        from URDF_Exporter.utils.config import load_config
        config = load_config()
        provenance['effective_config'] = config
        for name, module in list(sys.modules.items()):
            if name.startswith('URDF_Exporter.') and getattr(module, '__file__', None):
                provenance['loaded_code'][name] = loaded_code_provenance(vars(module), module.__file__)
        report.save()
        for name, code in provenance['loaded_code'].items():
            path = Path(code['path'])
            if not path.is_relative_to(repository):
                raise RuntimeError('Loaded code is outside this checkout. ' + name)
            recorded_hash = provenance['inputs_sha256'].get(path.relative_to(repository).as_posix())
            if code['status'] != 'match' or code['disk_sha256'] != recorded_hash:
                raise RuntimeError('Loaded code differs from recorded source. Restart Fusion. ' + name)
        modes = (('parametric', adsk.fusion.DesignTypes.ParametricDesignType),
                  ('direct', adsk.fusion.DesignTypes.DirectDesignType))
        for mode_name, mode in modes:
            for fixture in MODE_FIXTURES[mode_name]:
                destination = directory / (fixture + '_' + mode_name + '_description')
                case = report.start_case(fixture, mode_name, destination)
                checks = Checks(case['assertions'])
                created = False
                try:
                    with owned_document(app, adsk.core.DocumentTypes.FusionDesignDocumentType) as document:
                        created = True
                        try:
                            checks.equal('owned document is active', app.activeDocument == document, True)
                            design = adsk.fusion.Design.cast(app.activeProduct)
                            checks.equal('new document body count', design.rootComponent.bRepBodies.count, 0)
                            checks.equal('new document occurrence count', design.rootComponent.occurrences.count, 0)
                            checks.equal('new document sketch count', design.rootComponent.sketches.count, 0)
                            design.designType = mode
                            checks.equal('fixture design mode', int(design.designType), int(mode))
                            if hasattr(design, 'designIntent'):
                                checks.equal('owned document Hybrid design intent', int(design.designIntent),
                                             int(adsk.fusion.DesignIntentTypes.HybridDesignIntentType))
                            bindings, joints = _fixtures(design, fixture)
                            boxes, cad_bounds = _expectations(bindings, checks)
                            case['expected_boxes_si'] = {name: [vars(box) for box in entries] for name, entries in boxes.items()}
                            case['expected_joints_si'] = [vars(joint) for joint in joints]
                            case['direct_cad_bounds_m'] = cad_bounds
                            retained = []
                            case['source_before'] = source_snapshot(design, retained)
                            case['source_entities_before'] = entity_evidence(retained)
                            if fixture == 'parametric_history':
                                state = case['source_before']
                                feature_names = [feature['entity_name'] for feature in state['timeline']['features']]
                                checks.equal('editable sketch retained in timeline', 'ProofRectangle' in feature_names, True)
                                checks.equal('editable extrusion retained in timeline', 'ProofExtrude' in feature_names, True)
                                parameter = next(p for p in state['parameters'] if p['name'] == 'proof_height')
                                checks.equal('user parameter expression and units',
                                             (parameter['expression'], parameter['unit'], parameter['is_user']), ('60 mm', 'cm', True))
                                checks.close('user parameter value in cm', parameter['value'], 6., absolute=1e-10)
                                checks.equal('extrusion driven by user parameter',
                                             any(p['expression'] == 'proof_height' and not p['is_user'] for p in state['parameters']), True)
                            report.save()
                            try:
                                exported = export_design(design, destination, config=config)
                            finally:
                                current = []
                                case['source_after'] = source_snapshot(design, current)
                                case['source_entities_after'] = entity_evidence(current)
                                try:
                                    case['source_preservation'] = assert_source_unchanged(case['source_before'], case['source_after'])
                                    case['source_preservation']['entity_identity'] = assert_entity_identity(retained, current)
                                except Exception:
                                    case['source_preservation'] = dict(status='fail', traceback=traceback.format_exc())
                                    raise
                                finally:
                                    report.save()
                            checks.equal('production destination', str(Path(exported).resolve()), str(destination.resolve()))
                            verify_package(destination, boxes, cad_bounds, joints, checks)
                            case['status'] = 'pass'
                        except Exception:
                            case['status'] = 'fail'
                            case['traceback'] = traceback.format_exc()
                            report.save()
                    case['cleanup'] = dict(status='pass', owned_document_closed=True, previous_document_restored=True)
                except Exception:
                    case['status'] = 'fail'
                    case['cleanup'] = dict(status='fail' if created else 'not_exercised', traceback=traceback.format_exc())
                report.save()
    except Exception:
        report.data['fatal_traceback'] = traceback.format_exc()
        report.data['cases'].append(dict(fixture='runner_setup', mode='none', status='fail'))
    finally:
        if report.data['provenance']['status'] == 'recorded':
            try:
                after = source_hashes(report.data['repository'])
                report.data['provenance']['inputs_after_sha256'] = after
                if after != report.data['provenance']['inputs_sha256']:
                    report.data['provenance'].update(status='fail', reason='Input files changed during verification')
            except Exception:
                report.data['provenance'].update(status='fail', traceback=traceback.format_exc())
        report.data['finished_at_utc'] = datetime.now(timezone.utc).isoformat()
        report.save(finished=True)
    return report.path


def run(context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        dialog = ui.createFolderDialog()
        dialog.title = 'Choose parent folder for new Fusion verification evidence'
        if dialog.showDialog() != adsk.core.DialogResults.DialogOK:
            return
        report_path = run_verification(app, dialog.folder)
        import json
        data = json.loads(report_path.read_text(encoding='utf-8'))
        failed = sum(case['status'] == 'fail' for case in data['cases'])
        ui.messageBox('Fusion verification %s. %d failed cases.\n'
                      'Original-geometry invalidation was not exercised.\nReport and packages retained at\n%s'
                      % (data['status'].upper(), failed, report_path), 'fusion360tourdf verification')
    except Exception:
        ui.messageBox('Fusion verification failed.\n' + traceback.format_exc(), 'fusion360tourdf verification')
