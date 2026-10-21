# -*- coding: utf-8 -*-
"""
Created on Sun May 12 20:17:17 2019

@author: syuntoku
"""

import math
from ..utils import utils
from ..utils import transforms as tf
from ..utils import config as config_mod
from ..utils.names import allocate_names, ExportedJoints
from .Write import resolve_joint_limits
from .generalized import SourceIdentity, MotionBinding, expand_motion
from .coupling import read_motion_link, ExcludedMotion
from ..utils import validate

_MOTION_FIELDS = {
    'revolute': ('rotationAxisVector', 'rotationLimits', 'rotationValue', 1.0),
    'prismatic': ('slideDirectionVector', 'slideLimits', 'slideValue', 0.01),
}


def make_joints_dict(root, msg, config=None, *, sources=None, source_notes=None, joint_records=None):
    """Extract SI poses and joint-frame axes, with limits relative to the design pose."""

    joints_dict = {}
    success = msg
    source_notes = [] if source_notes is None else source_notes
    config = config_mod.load_config(overrides=config)
    joint_records = utils.root_joints(root, 'root inference and joint extraction') if joint_records is None else joint_records
    sources = utils.link_sources(root, config=config, joint_records=joint_records) if sources is None else sources
    effort, velocity = config['effort'], config['velocity']

    source_joints = [(joint, is_as_built) for joint, is_as_built in joint_records
                     if not getattr(joint, 'isSuppressed', False)]
    aliases = allocate_names([utils.joint_source_name(joint) for joint, _ in source_joints], 'joint')
    native_inventory, macros, bindings = [], {}, {}

    for joint, is_as_built in source_joints:
        joint_dict = {}
        occ_two, occ_one = utils.joint_occurrences(joint)
        context = getattr(joint, 'assemblyContext', None)
        source_name = utils.joint_source_name(joint)
        name = aliases[source_name]
        inventory_entry = dict(joint=joint, name=name, context=context, status='selected')
        native_inventory.append(inventory_entry)
        endpoints, errors = [], []
        for occurrence in (occ_two, occ_one):
            try:
                endpoints.append(utils.link_name(occurrence, sources, context))
            except ValueError as error:
                errors.append(error)
        if errors:
            if len(errors) == 2 and all(str(error).startswith('CONSTRAINT_TARGET_EXCLUDED') for error in errors):
                source_notes.append({'name': source_name, 'reason': 'All joint endpoints are excluded'})
                inventory_entry['status'] = 'excluded'
                continue
            raise ValueError(source_name + ': ' + '; '.join(map(str, errors))) from errors[0]
        joint_dict['parent'], joint_dict['child'] = endpoints
        try:
            type_value = _joint_type_value(joint.jointMotion)
        except Exception:
            msg = getattr(joint, 'name', 'joint') + ' has an unreadable joint type.'
            break
        joint_type = _urdf_type_for(type_value)
        locked = getattr(joint, 'isLocked', False) and type_value != 0
        if joint_type is None and locked and type_value in _FUSION_JOINT_NAMES:
            joint_type = 'fixed'
        generalized = joint_type is None and type_value in (3, 4, 5, 6) and not locked
        if generalized and config.get('generalized_joints', 'reject') == 'decompose':
            if type_value == 6 and config.get('ball_chart', 'reject') != 'export-local-rx-ry-rz':
                raise ValueError('BALL_CHART_REQUIRED: %s requires explicit ball_chart=export-local-rx-ry-rz; '
                                 'native Ball coordinates and enabled bounds remain blocked' % source_name)
            joint_type = 'compound'
        if joint_type is None:
            fusion_name = _FUSION_JOINT_NAMES.get(type_value, str(type_value))
            msg = ('UNSUPPORTED_FUSION_JOINT_TYPE: ' + source_name + ' uses ' + fusion_name +
                   '; select generalized_joints=decompose explicitly, or use Rigid, Revolute, or Slider joints.')
            break
        joint_dict['type'] = joint_type

        joint_dict['axis'] = [0, 0, 0]
        joint_dict['upper_limit'] = 0.0
        joint_dict['lower_limit'] = 0.0

        if locked:
            original_type = _urdf_type_for(type_value) or _FUSION_JOINT_NAMES[type_value]
            limits = getattr(joint.jointMotion, 'rotationLimits', None)
            if joint_type == 'revolute' and limits is not None and not limits.isMaximumValueEnabled and not limits.isMinimumValueEnabled:
                original_type = 'continuous'
            joint_dict['original_motion'] = {'type': original_type, 'fusion_type': _FUSION_JOINT_NAMES[type_value]}
            joint_dict['fixed_reason'] = 'Fusion joint is locked at the current pose'
            joint_dict['type'] = 'fixed'
        elif joint_type in _MOTION_FIELDS:
            axis_property, limits_property, position_property, scale = _MOTION_FIELDS[joint_type]
            try:
                motion = joint.jointMotion
                raw_axis = list(getattr(motion, axis_property).asArray())
                limits = getattr(motion, limits_property)
                position = float(getattr(motion, position_property)) * scale
                if not math.isfinite(position):
                    raise ValueError('Current joint position must be finite')
                maximum, minimum = limits.isMaximumValueEnabled, limits.isMinimumValueEnabled
                if maximum != minimum:
                    raise ValueError('Set both lower and upper limits')
                if maximum:
                    joint_dict['upper_limit'] = limits.maximumValue * scale - position
                    joint_dict['lower_limit'] = limits.minimumValue * scale - position
                elif joint_type == 'revolute':
                    joint_dict['type'] = 'continuous'
                    joint_dict['lower_limit'] = joint_dict['upper_limit'] = None
                else:
                    raise ValueError('Slider joints need finite lower and upper limits')
            except Exception as error:
                msg = joint.name + ' has invalid motion data: ' + str(error)
                break
            joint_dict['_raw_axis'] = raw_axis
            bindings[name] = (MotionBinding(SourceIdentity(utils._native(joint), context),
                               10 if joint_type == 'revolute' else 11, name, position,
                               'rad' if joint_type == 'revolute' else 'm'),)

        try:
            xyz_of_joint_cm = _resolve_joint_xyz_cm(joint, is_as_built)
            if xyz_of_joint_cm is None and is_as_built and type_value == 0:
                owner = sources[joint_dict['child']].owner
                xyz_of_joint_cm = utils.occurrence_translation(owner) if owner is not None and hasattr(owner, 'component') else [0, 0, 0]
            if xyz_of_joint_cm is None:
                raise ValueError('no origin')
            joint_dict['xyz'] = [i / 100.0 for i in xyz_of_joint_cm]

        except Exception:
            msg = 'JOINT_FRAME_UNAVAILABLE: ' + joint.name + ' has no readable current joint transform.'
            break

        parent_rotation = _source_rotation(joint_dict['parent'], sources)
        child_rotation = _source_rotation(joint_dict['child'], sources)
        joint_dict['rpy'] = list(tf.rpy_from_mat3(tf.mat3_mul(tf.mat3_transpose(parent_rotation), child_rotation)))
        if '_raw_axis' in joint_dict:
            raw_axis = joint_dict.pop('_raw_axis')
            joint_dict['axis'] = tf.normalize(tf.rotate_vector_by_mat3(raw_axis, tf.mat3_transpose(child_rotation)))
        joint_dict['effort'] = effort
        joint_dict['velocity'] = velocity
        joints_dict[name] = joint_dict
        if generalized:
            macros[name] = (joint, type_value, dict(joint_dict))
    identifiers = {
        'links': sources.identifier_mapping(),
        'joints': {alias: identity for identity, alias in aliases.items() if alias in joints_dict},
    }
    if msg == success:
        joints_dict = _rigid_attachments(root, joints_dict, sources, source_notes)
        if not macros:
            joints_dict = resolve_joint_limits(joints_dict, config)
        elif set(macros).intersection(config.get('joint_limits', {})):
            raise ValueError('DOF_SELECTOR: compound joint settings require exact generated DOF names')
        else:
            joints_dict = resolve_joint_limits(joints_dict, dict(config, joint_limits={n: v for n, v in
                config.get('joint_limits', {}).items() if n in joints_dict}))
        joints_dict = _orient_outward(joints_dict, sources)
        macro_tree, closing, loop_warnings = validate.detect_closing_joints(joints_dict)
        source_poses = {name: list(_source_pose_cm(name, sources)) for name in sources}
        for record in macro_tree.values():
            if record['type'] != 'compound':
                for index, value in zip((3, 7, 11), record['xyz']):
                    source_poses[record['child']][index] = value * 100
        frame_records, removed, generated_sources = {}, [], {}
        occupied = set(sources) | set(joints_dict)
        for name, (joint, type_value, original) in macros.items():
            oriented = joints_dict[name]
            xyz = [v * 100 for v in original['xyz']]
            pivot = [1, 0, 0, xyz[0], 0, 1, 0, xyz[1], 0, 0, 1, xyz[2], 0, 0, 0, 1]
            context = getattr(joint, 'assemblyContext', None)
            expansion = expand_motion({3: 'cylindrical', 4: 'pin_slot', 5: 'planar', 6: 'ball'}[type_value],
                joint.jointMotion, source=SourceIdentity(utils._native(joint), context), source_name=name,
                source_path=utils.joint_source_name(joint), parent=original['parent'], child=original['child'],
                parent_pose_cm=source_poses[original['parent']],
                child_pose_cm=source_poses[original['child']], joint_pose_cm=pivot,
                reverse=oriented.get('endpoint_reversed', False), occupied_names=occupied,
                effort=effort, velocity=velocity)
            bindings[name] = expansion.bindings
            if name not in macro_tree:
                records = {p.name: dict(type=p.type, lower_limit=p.lower_limit, upper_limit=p.upper_limit,
                                       effort=p.effort, velocity=p.velocity) for p in expansion.primitives}
                selected_settings = {n: v for n, v in config.get('joint_limits', {}).items() if n in records}
                records = resolve_joint_limits(records, dict(config, joint_limits=selected_settings))
                for entry in closing:
                    if entry['joint'] == name:
                        entry['removed_dofs'] = records
                removed.extend(p.name for p in expansion.primitives)
                removed.append(name)
                continue
            poses = {f.name: f.pose for f in expansion.frames}
            coordinate_bindings = {binding.primitive: binding for binding in expansion.bindings}
            for frame in expansion.frames:
                frame_records[frame.name] = dict(kind='frame', source_path=frame.source_path,
                    generated_from=name, current_pose={'xyz': list(frame.pose.xyz), 'rotation': list(frame.pose.rotation)})
            for primitive in expansion.primitives:
                binding = coordinate_bindings.get(primitive.name)
                if primitive.child in poses:
                    xyz_root = list(poses[primitive.child].xyz)
                else:
                    xyz_root = [v / 100 for v in source_poses[primitive.child][3:12:4]]
                macro_tree[primitive.name] = dict(parent=primitive.parent, child=primitive.child,
                    type=primitive.type, axis=list(primitive.axis), xyz=xyz_root,
                    rpy=list(tf.rpy_from_mat3(primitive.origin.rotation)),
                    lower_limit=primitive.lower_limit, upper_limit=primitive.upper_limit,
                    effort=primitive.effort, velocity=primitive.velocity,
                    source_motion=dict(source_joint=name, source_path=utils.joint_source_name(joint),
                        coordinate=primitive.coordinate, parameterization=expansion.parameterization,
                        native_motion_id=binding.motion_type if binding else None,
                        current_position_si=binding.current_position_si if binding else None,
                        unit=binding.unit if binding else None,
                        endpoint_reversed=oriented.get('endpoint_reversed', False)))
                generated_sources[primitive.name] = utils.joint_source_name(joint)
                occupied.add(primitive.name)
            occupied.update(poses)
            del macro_tree[name]
        removed.extend(entry['joint'] for entry in closing)
        closing_names = set(removed) | {entry['joint'] for entry in closing}
        joints_dict = resolve_joint_limits(macro_tree, dict(config, joint_limits={n: v for n, v in
            config.get('joint_limits', {}).items() if n not in closing_names}))
        native_inventory.extend(dict(joint=joint, name=utils.joint_source_name(joint),
            context=getattr(joint, 'assemblyContext', None), status='suppressed')
            for joint, _ in joint_records if getattr(joint, 'isSuppressed', False))
        relations = _motion_relations(root, sources, native_inventory, bindings, source_notes, config)
        result = ExportedJoints(joints_dict, identifiers)
        result.frame_records = frame_records
        result.motion_relations = relations
        result.removed_joint_names = removed
        result.closing_joints = closing
        result.loop_warnings = loop_warnings
        result.source_identifiers['joints'] = {alias: path for alias, path in identifiers['joints'].items()
                                               if alias in joints_dict}
        result.source_identifiers['joints'].update(generated_sources)
        return result, msg
    return ExportedJoints(joints_dict, identifiers), msg


def _source_pose_cm(name, sources):
    if name == 'base_link' or not hasattr(sources[name].owner, 'component'):
        return [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
    matrix = utils.occurrence_matrix_array(sources[name].owner)
    if matrix is None:
        raise ValueError('LINK_FRAME_UNAVAILABLE: ' + name)
    return matrix


def _motion_relations(root, sources, inventory, bindings, notes, config):
    if config.get('motion_links', 'reject') != 'mimic':
        return []

    def resolve(joint, context, motion_id):
        native = utils._native(joint)
        own_context = getattr(joint, 'assemblyContext', None)
        anchor = own_context if own_context is not None else context
        canonical_context = sources.resolve_context(anchor).owner if anchor is not None else None
        candidates = [entry for entry in inventory
                      if (utils._native(entry['joint']) is native or utils._native(entry['joint']) == native)
                      and ((entry['context'] is None and canonical_context is None) or
                           (entry['context'] is not None and canonical_context is not None and
                            sources.resolve_context(entry['context']).owner is canonical_context))]
        if len(candidates) != 1:
            raise ValueError('MOTION_LINK_UNKNOWN_OR_AMBIGUOUS_JOINT: actual joint/context not in held inventory')
        selected = candidates[0]
        name = selected['name']
        if selected['status'] == 'excluded':
            raise ExcludedMotion('MOTION_LINK_SOURCE_EXCLUDED: ' + name)
        if name not in bindings:
            raise ValueError('MOTION_LINK_DOF_UNAVAILABLE: %s is fixed, locked, suppressed, or excluded' % name)
        for binding in bindings[name]:
            if binding.motion_type == motion_id:
                return binding
        raise ValueError('MOTION_COORDINATE_UNAVAILABLE: %s motion %d' % (name, motion_id))

    return [read_motion_link(link, resolve, enclosing_context=context)
            for link, context in utils.motion_link_inventory(root, sources)]


# Fusion JointTypes enum values, see
# https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_JointTypes.htm
# 0 Rigid, 1 Revolute, 2 Slider, 3 Cylindrical, 4 PinSlot, 5 Planar, 6 Ball,
# 7 Inferred.
_FUSION_JOINT_NAMES = {
    0: 'Rigid',
    1: 'Revolute',
    2: 'Slider',
    3: 'Cylindrical',
    4: 'PinSlot',
    5: 'Planar',
    6: 'Ball',
    7: 'Inferred',
}

_FUSION_TO_URDF = {
    0: 'fixed',
    1: 'revolute',
    2: 'prismatic',
}


def _joint_type_value(joint_motion):
    """Return the integer JointTypes value for a joint motion."""
    joint_type = joint_motion.jointType
    for attr in ('value', 'value__'):
        try:
            value = getattr(joint_type, attr, None)
            if isinstance(value, int):
                return value
        except Exception:
            continue
    try:
        return int(joint_type)
    except Exception:
        pass
    class_name = type(joint_motion).__name__
    by_class = {
        'RigidJointMotion': 0,
        'RevoluteJointMotion': 1,
        'SliderJointMotion': 2,
        'CylindricalJointMotion': 3,
        'PinSlotJointMotion': 4,
        'PlanarJointMotion': 5,
        'BallJointMotion': 6,
    }
    if class_name in by_class:
        return by_class[class_name]
    raise ValueError('Unknown joint type: %r' % (joint_type,))


def _urdf_type_for(type_value):
    """Map a Fusion JointTypes value to a URDF joint type or None."""
    return _FUSION_TO_URDF.get(type_value)


def _matrix_translation_cm(matrix):
    if matrix is None:
        return None
    try:
        if callable(matrix):
            matrix = matrix()
        translation = getattr(matrix, 'translation', None)
        if translation is None:
            return None
        if callable(translation):
            translation = translation()
        return list(translation.asArray())
    except Exception:
        return None


def _resolve_joint_xyz_cm(joint, is_as_built):
    """Return the joint position in cm in the root frame, or None."""
    if is_as_built:
        return _resolve_asbuilt_xyz_cm(joint)
    return _resolve_standard_xyz_cm(joint)


def _resolve_standard_xyz_cm(joint):
    for attr in ('geometryTwoTransform', 'geometryOneTransform'):
        try:
            matrix = getattr(joint, attr, None)
            translation = _matrix_translation_cm(matrix)
            if translation is not None:
                return list(translation)
        except Exception:
            continue

    return None


def _resolve_asbuilt_xyz_cm(joint):
    try:
        translation = _matrix_translation_cm(joint.transform)
        if translation is not None:
            return translation
    except Exception:
        pass
    return None


def _source_rotation(name, sources):
    if name == 'base_link' or not hasattr(sources[name].owner, 'component'):
        return tf.IDENTITY_3X3
    matrix = utils.occurrence_matrix_array(sources[name].owner)
    if matrix is None:
        raise ValueError('LINK_FRAME_UNAVAILABLE: ' + name)
    return tf.rotation3_from_array16(matrix)


def _rigid_members(group, sources):
    members, excluded = set(), []
    context = getattr(group, 'assemblyContext', None)
    for occurrence in utils.collection_items(group.occurrences):
        try:
            selected, hidden = sources.rigid_members(occurrence, context)
        except ValueError as error:
            raise ValueError(group.name + ': ' + str(error)) from error
        members.update(selected)
        excluded.extend(hidden)
    if members and excluded:
        raise ValueError('CONSTRAINT_TARGET_EXCLUDED: rigid group %s crosses excluded sources %s' %
                         (group.name, ', '.join(excluded)))
    return members


def _rigid_attachments(root, joints, sources, source_notes):
    groups = [(entry['name'], members, entry)
              for entry, members in sources.grounding_attachments(source_notes)]
    for group in utils.collection_items(getattr(root, 'allRigidGroups', ())):
        if getattr(group, 'isSuppressed', False):
            continue
        members = _rigid_members(group, sources)
        context = getattr(group, 'assemblyContext', None)
        prefix = getattr(context, 'fullPathName', '')
        source_name = prefix + '+' + group.name if prefix else group.name
        if members:
            groups.append((source_name, members,
                           {'name': source_name, 'source': 'Fusion rigid group', 'members': sorted(members)}))
        else:
            source_notes.append({'name': source_name, 'reason': 'All rigid group members are excluded'})
    if not groups:
        return joints
    leaders = {name: name for name in sources}

    def leader(name):
        while leaders[name] != name:
            name = leaders[name]
        return name

    def union(members):
        roots = sorted({leader(name) for name in members})
        for name in roots[1:]:
            leaders[name] = roots[0]

    for _, members, _ in groups:
        union(members)
    for joint in joints.values():
        if joint['type'] == 'fixed':
            union((joint['parent'], joint['child']))
    clusters = {}
    for name in sources:
        clusters.setdefault(leader(name), set()).add(name)
    grouped = {leader(next(iter(members))) for _, members, _ in groups}
    provenance = {key: [] for key in grouped}
    for _, members, entry in sorted(groups, key=lambda group: group[0]):
        provenance[leader(next(iter(members)))].append(entry)
    adjacency = {key: [] for key in clusters}
    for name, joint in sorted(joints.items()):
        parent, child = leader(joint['parent']), leader(joint['child'])
        if joint['type'] == 'fixed':
            if parent in grouped:
                provenance[parent].append(dict(joint, name=name, source='Fusion fixed joint'))
            continue
        if parent == child and parent in grouped:
            kind = ('GROUNDING_INTERNAL_MOTION' if any(entry['source'] == 'Fusion grounding'
                    for entry in provenance[parent]) else 'RIGID_GROUP_INTERNAL_MOTION')
            raise ValueError('%s: %s conflicts with %s' %
                             (kind, name, ', '.join(entry['name'] for entry in provenance[parent])))
        adjacency[parent].append((name, child, joint['child']))
        adjacency[child].append((name, parent, joint['parent']))
    base = leader('base_link')
    attachments, queue, traversed = {base: 'base_link'}, [base], set()
    predecessors, incoming_edges = {base: None}, {}
    for current in queue:
        for name, other, endpoint in sorted(adjacency[current]):
            if name in traversed:
                continue
            traversed.add(name)
            if other in attachments:
                ancestors, node = set(), current
                while node is not None:
                    ancestors.add(node)
                    node = predecessors[node]
                cycle_edges, node = [name], other
                while node not in ancestors:
                    cycle_edges.append(incoming_edges[node])
                    node = predecessors[node]
                common, node = node, current
                while node != common:
                    cycle_edges.append(incoming_edges[node])
                    node = predecessors[node]
                cycle_members = {key: set() for key in grouped}
                for edge in cycle_edges:
                    for endpoint in (joints[edge]['parent'], joints[edge]['child']):
                        key = leader(endpoint)
                        if key in grouped:
                            cycle_members[key].add(endpoint)
                if any(len(members) > 1 for members in cycle_members.values()):
                    raise ValueError('RIGID_GROUP_KINEMATIC_LOOP: %s; rigid groups %s' %
                                     (', '.join(sorted(cycle_edges)), ', '.join(sorted(name for name, _, _ in groups))))
                continue
            attachments[other] = endpoint
            predecessors[other], incoming_edges[other] = current, name
            queue.append(other)
    result = {name: joint for name, joint in joints.items()
              if joint['type'] != 'fixed' or leader(joint['parent']) not in grouped}
    for key in sorted(grouped):
        if key not in attachments:
            raise ValueError('DISCONNECTED_RIGID_GROUP: ' + ', '.join(sorted(clusters[key])))
        parent = attachments[key]
        for child in sorted(clusters[key] - {parent}):
            name = 'rigid_attachment_' + child
            if name in result or name in joints:
                raise ValueError('Duplicate joint identity: ' + name)
            owner = sources[child].owner
            xyz = utils.occurrence_translation(owner) if hasattr(owner, 'component') else [0, 0, 0]
            if xyz is None:
                raise ValueError('LINK_FRAME_UNAVAILABLE: ' + child)
            result[name] = {'parent': parent, 'child': child, 'type': 'fixed',
                'xyz': [value / 100 for value in xyz], 'axis': [0, 0, 0],
                'rpy': list(tf.rpy_from_mat3(tf.mat3_mul(tf.mat3_transpose(_source_rotation(parent, sources)),
                                                       _source_rotation(child, sources)))),
                'rigid_constraints': provenance[key]}
    return result


def _orient_outward(joints, sources):
    adjacency = {name: [] for name in sources}
    for name, joint in joints.items():
        adjacency[joint['parent']].append(name)
        adjacency[joint['child']].append(name)
    result, visited, queue = {}, {'base_link'}, ['base_link']
    for parent in queue:
        for name in sorted(adjacency[parent]):
            if name in result:
                continue
            record = dict(joints[name])
            old_parent, old_child = record['parent'], record['child']
            child = old_child if old_parent == parent else old_parent
            if child in visited:
                result[name] = record
                continue
            if old_parent != parent:
                world_axis = tf.rotate_vector_by_mat3(record['axis'], _source_rotation(old_child, sources))
                record['axis'] = tf.normalize(tf.rotate_vector_by_mat3(
                    [-value for value in world_axis], tf.mat3_transpose(_source_rotation(child, sources))))
                record['parent'], record['child'] = parent, child
                record['rpy'] = list(tf.rpy_from_mat3(tf.mat3_mul(
                    tf.mat3_transpose(_source_rotation(parent, sources)), _source_rotation(child, sources))))
                record['endpoint_reversed'] = True
            result[name] = record
            visited.add(child)
            queue.append(child)
    if len(visited) != len(sources):
        raise ValueError('DISCONNECTED_LINKS: ' + ', '.join(sorted(sources.keys() - visited)))
    return result
