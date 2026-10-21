# -*- coding: utf-8 -*-
"""
Created on Sun May 12 19:15:34 2019

@author: syuntoku
"""

import adsk, adsk.core, adsk.fusion
from pathlib import Path
import struct
import tempfile
from dataclasses import dataclass
from collections.abc import Mapping
from types import MappingProxyType
from . import config as config_mod
from .stl import read_binary_stl, transform_binary_stl
from .names import allocate_names, sanitize_name
from .mesh_geometry import ResourceLimitError
from .mesh_inputs import MeshInputs, MeshSnapshot, check_cancelled


def link_name(occurrence, sources, context=None):
    return resolve_source(occurrence, sources, context=context)


def collection_items(collection):
    if not any(hasattr(type(collection), method) for method in ('__iter__', '__getitem__')):
        return [collection.item(index) for index in range(collection.count)]
    return list(collection)


def root_joints(root, phase):
    joints = []
    for attribute, is_as_built in (('allJoints', False), ('allAsBuiltJoints', True)):
        try:
            collection = getattr(root, attribute)
            count = len(collection) if isinstance(collection, (list, tuple)) else collection.count
            if not isinstance(count, int) or count < 0:
                raise ValueError('Invalid joint collection count: %r' % (count,))
            items = collection_items(collection)
            if len(items) != count:
                raise ValueError('Expected %d joints, read %d' % (count, len(items)))
            if any(joint is None for joint in items):
                raise ValueError('Collection contains a missing joint')
        except Exception as error:
            raise ValueError('JOINT_COLLECTION_UNAVAILABLE: %s.%s during %s; cannot establish complete '
                             'root-context constraints. Keep constraints and design history intact; '
                             'inspect the Fusion constraint collection. Original error: %s' %
                             (source_path(root), attribute, phase, error)) from error
        joints.extend((joint, is_as_built) for joint in items)
    return joints


def joint_source_name(joint):
    context = getattr(joint, 'assemblyContext', None)
    prefix = getattr(context, 'fullPathName', '')
    return prefix + '+' + joint.name if prefix else joint.name


def motion_link_inventory(root, sources):
    """Keep each component's links in each actual enclosing occurrence context."""
    owners = [(root, None)] + [(occ.component, occ)
                              for occ in collection_items(getattr(root, 'allOccurrences', ()))]
    result, identities = [], []
    for owner, context in owners:
        try:
            links = collection_items(owner.motionLinks)
        except Exception as error:
            raise ValueError('MOTION_LINK_COLLECTION_UNAVAILABLE: ' + source_path(owner)) from error
        for link in links:
            try:
                suppressed = link.isSuppressed
                own_context = link.assemblyContext if not suppressed else None
            except Exception as error:
                raise ValueError('MOTION_LINK_CONTEXT_UNAVAILABLE: ' + source_path(owner)) from error
            anchor = own_context if own_context is not None else context
            canonical = sources.resolve_context(anchor).owner if anchor is not None else None
            identity = link if suppressed else _native(link)
            if not any(flag == suppressed and (held is identity or held == identity) and held_context is canonical
                       for held, held_context, flag in identities):
                result.append((link, canonical))
                identities.append((identity, canonical, suppressed))
    return result


def joint_occurrences(joint):
    name = joint_source_name(joint)
    try:
        valid = joint.isValid
    except Exception as error:
        raise ValueError('JOINT_ENDPOINT_UNAVAILABLE: %s cannot inspect joint validity' % name) from error
    if not valid:
        raise ValueError('INVALID_JOINT_ENDPOINT: %s is an invalid Fusion joint; inspect retained-history '
                         'references without disabling design history' % name)
    endpoints = []
    for attribute in ('occurrenceTwo', 'occurrenceOne'):
        try:
            endpoint = getattr(joint, attribute)
            valid = endpoint is None or endpoint.isValid
        except Exception as error:
            raise ValueError('JOINT_ENDPOINT_UNAVAILABLE: %s.%s cannot be read; inspect missing or '
                             'historical references without disabling design history' % (name, attribute)) from error
        if not valid:
            raise ValueError('INVALID_JOINT_ENDPOINT: %s.%s references an invalid occurrence' % (name, attribute))
        endpoints.append(endpoint)
    if all(endpoint is None for endpoint in endpoints):
        raise ValueError('INVALID_JOINT_ENDPOINT: %s has no occurrence endpoint' % name)
    return endpoints


@dataclass(frozen=True)
class LinkSource:
    owner: object
    bodies: tuple
    kind: str = 'physical'

    def __post_init__(self):
        if self.kind not in ('physical', 'frame') or bool(self.bodies) != (self.kind == 'physical'):
            raise ValueError('Physical sources require bodies; frame sources must not own bodies.')


@dataclass(frozen=True)
class OccurrenceContext:
    owner: object
    path: str
    native: object
    visible: bool
    physical: bool
    grounded: bool
    ground_to_parent: bool


@dataclass(frozen=True)
class GroundingConstraint:
    child: str
    parent: str
    relationships: tuple

    def provenance(self):
        return {'name': self.child + '.' + self.relationships[0][1],
                'source': 'Fusion grounding',
                'relationships': [{'path': path, 'property': flag} for path, flag in self.relationships]}


def _native(occurrence):
    native = getattr(occurrence, 'nativeObject', None)
    return occurrence if native is None else native


def _require_valid_occurrence(owner, path):
    if path == 'design_root':
        return
    try:
        valid = owner.isValid
    except Exception as error:
        raise ValueError('CONSTRAINT_TARGET_UNAVAILABLE: %s cannot inspect occurrence validity' % path) from error
    if not valid:
        raise ValueError('INVALID_CONSTRAINT_TARGET: ' + path)


class SourceRegistry(Mapping):
    """Selected physical/frame links and the complete root-context identity inventory."""

    def __init__(self, sources, contexts):
        self._sources = MappingProxyType(dict(sources))
        self._contexts = tuple(contexts)
        self._by_path = MappingProxyType({entry.path: entry for entry in self._contexts})
        if len(self._by_path) != len(self._contexts):
            raise ValueError('Duplicate occurrence context identity')
        self._names = MappingProxyType({entry.path: name
            for entry in self._contexts for name, source in self._sources.items()
            if entry.owner is source.owner})
        self._paths = MappingProxyType({name: path for path, name in self._names.items()})
        self._context_paths = frozenset('+'.join(entry.path.split('+')[:length])
            for entry in self._contexts for length in range(1, len(entry.path.split('+')) + 1))
        self.grounding = self._grounding_constraints()
        self._grounding_parents = MappingProxyType({constraint.child: constraint.parent
            for constraint in self.grounding if constraint.parent != 'design_root'})

    def __getitem__(self, name):
        return self._sources[name]

    def __iter__(self):
        return iter(self._sources)

    def __len__(self):
        return len(self._sources)

    def identifier_mapping(self):
        return dict(self._paths)

    def source_path(self, name):
        return self._paths[name]

    def _grounding_constraints(self):
        constraints = []
        for entry in self._contexts:
            if not (entry.physical or entry.path in self._names) or entry.path == 'design_root':
                continue
            if entry.path in self._names and (entry.grounded or entry.ground_to_parent):
                _require_valid_occurrence(entry.owner, entry.path)
            if entry.grounded:
                constraints.append(GroundingConstraint(entry.path, 'design_root',
                                                      ((entry.path, 'isGrounded'),)))
            if not entry.ground_to_parent:
                continue
            pending = [(entry.path.rpartition('+')[0] or 'design_root', ((entry.path, 'isGroundToParent'),))]
            for parent, relationships in pending:
                ancestor = self._by_path.get(parent)
                if ancestor is not None and entry.path in self._names:
                    _require_valid_occurrence(ancestor.owner, ancestor.path)
                if parent == 'design_root' or ancestor is not None and (ancestor.physical or parent in self._names):
                    constraints.append(GroundingConstraint(entry.path, parent, relationships))
                    continue
                if ancestor is None or not (ancestor.grounded or ancestor.ground_to_parent):
                    constraints.append(GroundingConstraint(entry.path, parent, relationships))
                    continue
                if ancestor.grounded:
                    pending.append(('design_root', relationships + ((parent, 'isGrounded'),)))
                if ancestor.ground_to_parent:
                    pending.append((parent.rpartition('+')[0] or 'design_root',
                                    relationships + ((parent, 'isGroundToParent'),)))
        return tuple(constraints)

    def grounding_root(self, name):
        path = self._paths[name]
        while path in self._grounding_parents:
            path = self._grounding_parents[path]
        return path

    def world_fixed(self, path):
        if path == 'design_root':
            return True
        return any(constraint.child == path and self.world_fixed(constraint.parent)
                   for constraint in self.grounding)

    def grounding_attachments(self, source_notes):
        base_path = self._paths['base_link']
        anchor = self._names.get('design_root')
        if anchor is None and self.world_fixed(base_path):
            anchor = 'base_link'
        attachments = []
        for constraint in self.grounding:
            child = self._names.get(constraint.child)
            parent = anchor if constraint.parent == 'design_root' else self._names.get(constraint.parent)
            provenance = constraint.provenance()
            if child is None and parent is None:
                source_notes.append(dict(provenance, reason='All grounding endpoints are excluded'))
                continue
            if child is not None and constraint.parent == 'design_root' and anchor is None:
                raise ValueError('GROUNDING_WORLD_ANCHOR_UNAVAILABLE: %s requires a represented design-root frame or a '
                                 'world-grounded selected root; set root_occurrence to a grounded source' %
                                 constraint.child)
            if child is None or parent is None:
                raise ValueError('CONSTRAINT_TARGET_EXCLUDED: grounding %s crosses excluded sources %s -> %s' %
                                 (provenance['name'], constraint.child, constraint.parent))
            if child == parent:
                source_notes.append(dict(provenance, reason='Selected root establishes the world anchor'))
            else:
                attachments.append((provenance, {parent, child}))
        return attachments

    def _resolve_path(self, occurrence, caller=None, seen=()):
        if occurrence is None:
            return 'design_root'
        path = source_path(occurrence)
        if any(occurrence is previous for previous in seen):
            raise ValueError('AMBIGUOUS_CONSTRAINT_TARGET: cyclic assembly context for ' + path)
        exact = [entry.path for entry in self._contexts if entry.owner is occurrence or entry.owner == occurrence]
        if exact:
            candidates = exact
        else:
            seen = seen + (occurrence,)
            if caller is not None and (occurrence is caller or occurrence == caller):
                caller = None
            own_context = getattr(occurrence, 'assemblyContext', None)
            if own_context is not None:
                anchor = self._resolve_path(own_context, caller, seen)
            elif caller is not None:
                anchor = self._resolve_path(caller, seen=seen)
            else:
                anchor = 'design_root'
            native = _native(occurrence)
            aliases = [entry.path for entry in self._contexts if entry.native == native]
            available = aliases
            if not aliases and not hasattr(occurrence, 'nativeObject'):
                available = self._context_paths
            candidates = [candidate for candidate in available
                if (candidate == path or candidate.endswith('+' + path)) and
                   (anchor == 'design_root' or candidate.startswith(anchor + '+'))]
        if len(candidates) == 1:
            return candidates[0]
        if candidates:
            raise ValueError('AMBIGUOUS_CONSTRAINT_TARGET: ' + path)
        raise ValueError('UNRESOLVABLE_CONSTRAINT_TARGET: CONSTRAINT_TARGET_EXCLUDED: ' + path)

    def resolve_context(self, occurrence, context=None):
        path = self._resolve_path(occurrence, context)
        if path not in self._by_path:
            raise ValueError('CONSTRAINT_TARGET_EXCLUDED: ' + path)
        entry = self._by_path[path]
        for owner in (occurrence, entry.owner):
            _require_valid_occurrence(owner, path)
        return entry

    def resolve_source(self, occurrence, context=None):
        entry = self.resolve_context(occurrence, context)
        if entry.path in self._names:
            return self._names[entry.path]
        raise ValueError('CONSTRAINT_TARGET_EXCLUDED: ' + entry.path)

    def rigid_members(self, occurrence, context=None):
        entry = self.resolve_context(occurrence, context)
        if entry.path in self._names:
            return {self._names[entry.path]}, []
        return set(), [entry.path]


def source_path(owner):
    return getattr(owner, 'fullPathName', None) or getattr(owner, 'name', 'design_root')


def visible_bodies(owner):
    return tuple(body for body in collection_items(getattr(owner, 'bRepBodies', ()))
                 if occurrence_visible(body))


def _select_frame_sources(root, sources, contexts, configured, joint_records):
    registry = SourceRegistry(sources, contexts)

    def select(entry):
        if entry.visible and not entry.physical and entry.path not in registry._names:
            _require_valid_occurrence(entry.owner, entry.path)
            name = entry.path
            if name in sources and sources[name].owner is not entry.owner:
                raise ValueError('Duplicate occurrence context identity: ' + name)
            sources[name] = LinkSource(entry.owner, (), kind='frame')

    if configured:
        entries = [entry for entry in contexts if entry.path == configured and entry.visible]
        if len(entries) != 1:
            raise ValueError('ROOT_NOT_FOUND: %s must identify one visible source by full path' % configured)
        select(entries[0])
    for joint, _ in joint_records:
        if getattr(joint, 'isSuppressed', False):
            continue
        for endpoint in joint_occurrences(joint):
            try:
                select(registry.resolve_context(endpoint, getattr(joint, 'assemblyContext', None)))
            except ValueError as error:
                raise ValueError(joint_source_name(joint) + ': ' + str(error)) from error
    for group in collection_items(getattr(root, 'allRigidGroups', ())):
        if getattr(group, 'isSuppressed', False):
            continue
        for endpoint in collection_items(group.occurrences):
            try:
                entry = registry.resolve_context(endpoint, getattr(group, 'assemblyContext', None))
            except ValueError as error:
                raise ValueError(group.name + ': ' + str(error)) from error
            select(entry)
    while True:
        registry = SourceRegistry(sources, contexts)
        previous = len(sources)
        for constraint in registry.grounding:
            if constraint.child not in registry._names or constraint.parent == 'design_root':
                continue
            entry = registry._by_path.get(constraint.parent)
            if entry is None:
                raise ValueError('UNRESOLVABLE_GROUNDING_PARENT: %s -> %s' % (constraint.child, constraint.parent))
            select(entry)
        if len(sources) == previous:
            return registry


def link_sources(root, config=None, *, joint_records=None, cancelled=None):
    """Capture visible direct bodies and explicitly required frame contexts."""
    config = config_mod.load_config(overrides=config)
    configured = config['root_occurrence']
    mesh_inputs = MeshInputs(config.get('mesh_physics'), cancelled)
    sources = {}
    occurrences = collection_items(getattr(root, 'allOccurrences', ()))
    if config.get('motion_links', 'reject') == 'reject':
        for owner in [root] + [occ.component for occ in occurrences]:
            for motion_link in collection_items(getattr(owner, 'motionLinks', ())):
                if not getattr(motion_link, 'isSuppressed', False):
                    raise ValueError('MOTION_LINK_NOT_IMPLEMENTED: %s / %s is an active Fusion motion link; '
                                     'select motion_links=mimic explicitly' %
                                     (source_path(owner), motion_link.name))
    hidden = [source_path(occ) for occ in occurrences if not occurrence_visible(occ)]
    contexts = tuple(OccurrenceContext(owner, 'design_root' if owner is root else source_path(owner),
        _native(owner), occurrence_visible(owner) and not any(source_path(owner).startswith(p + '+') for p in hidden),
        bool(collection_items(getattr(owner, 'bRepBodies', ())) or collection_items(getattr(owner, 'meshBodies', ()))),
        bool(getattr(owner, 'isGrounded', False)), bool(getattr(owner, 'isGroundToParent', False)))
        for owner in occurrences + [root])
    for entry in contexts:
        check_cancelled(cancelled)
        owner, path = entry.owner, entry.path
        if not entry.visible:
            continue
        mesh_bodies = [body for body in collection_items(getattr(owner, 'meshBodies', ()))
                       if occurrence_visible(body)]
        bodies = visible_bodies(owner)
        matrix = occurrence_matrix_array(owner) if mesh_bodies and owner is not root else None
        bodies += tuple(mesh_inputs.capture(body, owner, path, matrix) for body in mesh_bodies)
        if not bodies:
            continue
        name = path
        if name in sources:
            raise ValueError('Duplicate occurrence context identity: ' + name)
        sources[name] = LinkSource(owner, bodies)
    mesh_inputs.finish()
    sources = _select_frame_sources(root, sources, contexts, configured, joint_records or ())
    if not sources:
        if configured:
            raise ValueError('ROOT_NOT_FOUND: %s has no selected source' % configured)
        raise ValueError('NO_VISIBLE_BREP_BODIES: no physical links selected')
    if configured:
        candidates = [name for name, source in sources.items()
                      if ('design_root' if source.owner is root else source_path(source.owner)) == configured]
        if len(candidates) != 1:
            raise ValueError('ROOT_NOT_FOUND: %s must identify one visible source by full path' % configured)
    else:
        candidates = [name for name, source in sources.items()
                      if source.owner is not root and source.owner.component.name == 'base_link']
        if not candidates and 'design_root' in sources:
            candidates = ['design_root']
        if not candidates:
            candidates = [sources._names[entry.path] for entry in contexts
                          if entry.grounded and entry.path in sources._names]
        if not candidates and len(sources) == 1:
            candidates = list(sources)
        if not candidates:
            children = set()
            inference_records = root_joints(root, 'root inference') if joint_records is None else joint_records
            for joint, _ in inference_records:
                if getattr(joint, 'isSuppressed', False):
                    continue
                occ_two, occ_one = joint_occurrences(joint)
                try:
                    parent = resolve_source(occ_two, sources, getattr(joint, 'assemblyContext', None))
                    child = resolve_source(occ_one, sources, getattr(joint, 'assemblyContext', None))
                    if parent == child or sources.grounding_root(parent) != sources.grounding_root(child):
                        children.add(child)
                except ValueError as error:
                    if str(error).startswith(('UNRESOLVABLE_CONSTRAINT_TARGET', 'AMBIGUOUS_CONSTRAINT_TARGET')):
                        raise ValueError(joint_source_name(joint) + ': ' + str(error)) from error
                    continue
            children.update(sources._names[constraint.child] for constraint in sources.grounding
                            if constraint.parent != 'design_root' and constraint.child in sources._names
                            and constraint.parent in sources._names)
            candidates = [name for name in sources if name not in children]
        if len(candidates) != 1:
            raise ValueError('ROOT_AMBIGUOUS: %s; set root_occurrence to an exact full path or design_root' %
                             ', '.join(source_path(sources[name].owner) for name in candidates or sources))
    selected = candidates[0]
    aliases = allocate_names(sources, 'link', root_identity=selected)
    return SourceRegistry({aliases[name]: source for name, source in sources.items()}, contexts)


def resolve_source(occurrence, sources, context=None):
    return sources.resolve_source(occurrence, context)


def transform_point(matrix_array_16, point_3):
    """Apply a 4x4 homogeneous transform (16-element array) to a 3D point."""
    M = matrix_array_16
    a = point_3
    ex = [M[0], M[4], M[8]]
    ey = [M[1], M[5], M[9]]
    ez = [M[2], M[6], M[10]]
    oo = [M[3], M[7], M[11]]
    return [a[0] * ex[i] + a[1] * ey[i] + a[2] * ez[i] + oo[i] for i in range(3)]


def occurrence_matrix_array(occ):
    """Return the 16-element transform array for an occurrence.

    Uses ``transform2`` (the current API, ``transform`` was retired in
    July 2025) with a fallback to ``transform`` for very old clients.
    Returns None when neither is available.
    """
    for attr in ('transform2', 'transform'):
        try:
            matrix = getattr(occ, attr, None)
            if matrix is None:
                continue
            if callable(matrix):
                matrix = matrix()
            if hasattr(matrix, 'asArray'):
                return list(matrix.asArray())
            if isinstance(matrix, (list, tuple)) and len(matrix) == 16:
                return list(matrix)
        except Exception:
            continue
    return None


def occurrence_translation(occ):
    """Return the occurrence translation in Fusion internal units (cm)."""
    for attr in ('transform2', 'transform'):
        try:
            matrix = getattr(occ, attr, None)
            if matrix is None:
                continue
            if callable(matrix):
                matrix = matrix()
            translation = getattr(matrix, 'translation', None)
            if translation is None:
                continue
            if callable(translation):
                translation = translation()
            if hasattr(translation, 'asArray'):
                return list(translation.asArray())
        except Exception:
            continue
    matrix_array = occurrence_matrix_array(occ)
    if matrix_array:
        return [matrix_array[3], matrix_array[7], matrix_array[11]]
    return None


def occurrence_visible(occ):
    """True unless the occurrence (or its component) hides its light bulb.

    newtonjeri's exporter lets users pick the export set with light-bulb
    toggles; Adriaeik treats hidden bodies as not exported. Default True
    when the API shape is unavailable (e.g. offline mocks).
    """
    for source in (occ, getattr(occ, 'component', None)):
        try:
            if getattr(source, 'isVisible', True) is False:
                return False
            bulb = getattr(source, 'isLightBulbOn', None)
            if bulb is None:
                continue
            if callable(bulb):
                bulb = bulb()
            if bulb is False:
                return False
        except Exception:
            continue
    return True


def iter_link_occurrences(root):
    """Yield visible, body-owning occurrences in root assembly context."""
    occurrences = getattr(root, 'allOccurrences', None)
    if occurrences is None:
        occurrences = getattr(root, 'occurrences', ())
    for occurrence in collection_items(occurrences):
        if not occurrence_visible(occurrence):
            continue
        if visible_bodies(occurrence):
            yield occurrence


def export_stl(design, targets, config=None, *, sources=None, cancelled=None):
    """Concatenate validated per-body STL triangles without exporting descendants."""
    config = config_mod.load_config(overrides=config)
    root = design.rootComponent
    sources = link_sources(root, config=config, cancelled=cancelled) if sources is None else sources
    physical = {name: source for name, source in sources.items() if source.kind == 'physical'}
    if physical.keys() != targets.keys():
        raise ValueError('Mesh targets do not match extracted link identities.')
    method = config['collision_method']
    triangle_limit = config[method]['max_triangles'] if method in ('convex_decomposition', 'convex_hull') else None

    def export(geometry, target, name):
        manager = design.exportManager
        options = manager.createSTLExportOptions(geometry, str(target))
        if options is None or not hasattr(options, 'unitType'):
            raise RuntimeError('Fusion cannot guarantee millimeter STL units for ' + name)
        options.unitType = adsk.fusion.DistanceUnits.MillimeterDistanceUnits
        options.sendToPrintUtility = False
        options.isBinaryFormat = True
        options.meshRefinement = config_mod.mesh_refinement_value(config, adsk.fusion)
        if not manager.execute(options):
            raise RuntimeError('Fusion mesh export failed for ' + name)
        if triangle_limit is not None and target.stat().st_size > 84 + 50 * triangle_limit:
            raise ResourceLimitError('Fusion STL exceeds collision max_triangles byte allowance: ' + name)
        return read_binary_stl(target)

    for name, source in physical.items():
        check_cancelled(cancelled)
        target = Path(targets[name])
        with tempfile.TemporaryDirectory(prefix='.link-bodies-', dir=target.parent) as temporary:
            count, payload = 0, bytearray()
            for index, body in enumerate(source.bodies):
                check_cancelled(cancelled)
                if isinstance(body, MeshSnapshot):
                    part_count, part_payload = body.triangle_count, body.payload
                else:
                    native = getattr(body, 'nativeObject', None)
                    part_count, part_payload = export(native if native is not None else body,
                        Path(temporary) / ('%d.stl' % index), name)
                    if native is not None:
                        matrix = occurrence_matrix_array(source.owner)
                        if matrix is None:
                            raise ValueError('MESH_FRAME_UNAVAILABLE: ' + name)
                        part_payload = transform_binary_stl(part_payload, matrix)
                count += part_count
                if triangle_limit is not None and count > triangle_limit:
                    raise ResourceLimitError('Combined STL exceeds collision max_triangles: ' + name)
                payload.extend(part_payload)
            check_cancelled(cancelled)
            target.write_bytes(b'fusion360tourdf direct bodies'.ljust(80, b'\0') + struct.pack('<I', count) + payload)


def collect_visual_assets(sources, config):
    if config.get('visual_materials', 'legacy') == 'legacy':
        return None
    from .visual_assets import read_native_body, build_obj_assets
    identity = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
    assets = {}
    for name, source in sources.items():
        if source.kind != 'physical':
            continue
        if any(isinstance(body, MeshSnapshot) for body in source.bodies):
            raise ValueError('MESH_BODY_VISUAL_MATERIALS_UNSUPPORTED: MeshBody snapshots do not expose BRep faces or UVs; '
                             'use visual_materials=legacy or a validated caller-supplied visual asset bundle')
        matrix = occurrence_matrix_array(source.owner) if hasattr(source.owner, 'component') else identity
        if matrix is None:
            raise ValueError('MESH_FRAME_UNAVAILABLE: ' + name)
        snapshots = [read_native_body(body, owner=source.owner if hasattr(source.owner, 'component') else None,
                     root_from_body_cm=matrix, context_id='%s_body_%d' % (name, index),
                     shader_policy=config['visual_materials']) for index, body in enumerate(source.bodies)]
        if config.get('visual_meshes', 'combined') == 'per_body':
            assets[name] = [build_obj_assets(snapshot, body_context={
                'id': '%s_body_%d' % (name, index), 'source_path': sources.source_path(name),
                'selected_body_count': len(source.bodies)})
                for index, snapshot in enumerate(snapshots)]
        else:
            assets[name] = build_obj_assets(snapshots)
    return assets


def file_dialog(ui):     
    """
    display the dialog to save the file
    """
    # Set styles of folder dialog.
    folderDlg = ui.createFolderDialog()
    folderDlg.title = 'Fusion Folder Dialog' 
    
    # Show folder dialog
    dlgResult = folderDlg.showDialog()
    if dlgResult == adsk.core.DialogResults.DialogOK:
        return folderDlg.folder
    return False


def origin2center_of_mass(inertia, center_of_mass, mass):
    """
    convert the moment of the inertia about the world coordinate into 
    that about center of mass coordinate

    Parameters
    ----------
    moment of inertia about the world coordinate:  [xx, yy, zz, xy, yz, xz]
    center_of_mass: [x, y, z]
    
    Returns
    ----------
    moment of inertia about center of mass : [xx, yy, zz, xy, yz, xz]
    """
    x = center_of_mass[0]
    y = center_of_mass[1]
    z = center_of_mass[2]
    translation_matrix = [y**2 + z**2, x**2 + z**2, x**2 + y**2,
                         -x*y, -y*z, -x*z]
    # No rounding here: tiny mechanisms (screws, sensor heads) have inertia
    # far below 1e-6 kg m^2 and round(x, 6) truncates them to exactly zero
    # (fixed upstream by newtonjeri with scientific notation). Display
    # formatting lives in transforms.format_float.
    return [i - mass*t for i, t in zip(inertia, translation_matrix)]
