# -*- coding: utf-8 -*-
"""
Created on Sun May 12 19:15:34 2019

@author: syuntoku
"""

import adsk, adsk.core, adsk.fusion
import re
from pathlib import Path
import struct
import tempfile
from dataclasses import dataclass
from collections.abc import Mapping
from types import MappingProxyType
from . import config as config_mod
from .stl import read_binary_stl, transform_binary_stl


def sanitize_name(name):
    """Return a ROS/URDF-safe link or joint name.

    Fusion allows spaces, colons, parens, dashes, etc. URDF/ROS
    names should stay in [A-Za-z0-9_]. Collapse runs of invalid
    characters to a single underscore so `base_link (1):1` and
    `base_link__1` converge instead of diverging.
    """
    cleaned = re.sub(r'[^A-Za-z0-9_]+', '_', str(name)).strip('_')
    cleaned = cleaned or 'link'
    return 'link_' + cleaned if cleaned[0].isdigit() else cleaned


def link_name(occurrence, sources, context=None):
    return resolve_source(occurrence, sources, context=context)


def collection_items(collection):
    try:
        return list(collection)
    except TypeError:
        return [collection.item(index) for index in range(collection.count)]


@dataclass(frozen=True)
class LinkSource:
    owner: object
    bodies: tuple


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


class SourceRegistry(Mapping):
    """Selected direct bodies and the unfiltered root-context identity inventory."""

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
        self.grounding, self.grounding_notes = self._grounding_constraints()
        self._grounding_parents = MappingProxyType({constraint.child: constraint.parent
            for constraint in self.grounding if constraint.parent != 'design_root'})

    def __getitem__(self, name):
        return self._sources[name]

    def __iter__(self):
        return iter(self._sources)

    def __len__(self):
        return len(self._sources)

    def _grounding_constraints(self):
        constraints, notes = [], []
        for entry in self._contexts:
            if not entry.physical or entry.path == 'design_root':
                continue
            if entry.grounded:
                constraints.append(GroundingConstraint(entry.path, 'design_root',
                                                      ((entry.path, 'isGrounded'),)))
            if not entry.ground_to_parent:
                continue
            pending = [(entry.path.rpartition('+')[0] or 'design_root', ((entry.path, 'isGroundToParent'),))]
            for parent, relationships in pending:
                ancestor = self._by_path.get(parent)
                if parent == 'design_root' or ancestor is not None and ancestor.physical:
                    constraints.append(GroundingConstraint(entry.path, parent, relationships))
                    continue
                if ancestor is None or not (ancestor.grounded or ancestor.ground_to_parent):
                    error = ('BODYLESS_FRAME_GROUNDING_NOT_IMPLEMENTED: %s requires an explicit fixed '
                             'root-context parent frame at %s' % (entry.path, parent))
                    if entry.path in self._names:
                        raise ValueError(error)
                    notes.append(dict(GroundingConstraint(entry.path, parent, relationships).provenance(),
                                      reason='Excluded source grounding has an unrepresented parent frame', detail=error))
                    continue
                if ancestor.grounded:
                    pending.append(('design_root', relationships + ((parent, 'isGrounded'),)))
                if ancestor.ground_to_parent:
                    pending.append((parent.rpartition('+')[0] or 'design_root',
                                    relationships + ((parent, 'isGroundToParent'),)))
        return tuple(constraints), tuple(notes)

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
        source_notes.extend(self.grounding_notes)
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
                raise ValueError('GROUNDING_WORLD_ANCHOR_UNAVAILABLE: %s requires design-root bodies or a '
                                 'world-grounded selected root; set root_occurrence to a grounded physical source' %
                                 constraint.child)
            if child is None or parent is None:
                raise ValueError('CONSTRAINT_TARGET_EXCLUDED: grounding %s crosses excluded sources %s -> %s' %
                                 (provenance['name'], constraint.child, constraint.parent))
            if child == parent:
                source_notes.append(dict(provenance, reason='Selected physical root establishes the world anchor'))
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
        raise ValueError('CONSTRAINT_TARGET_EXCLUDED: ' + path)

    def resolve_context(self, occurrence, context=None):
        path = self._resolve_path(occurrence, context)
        if path not in self._by_path:
            raise ValueError('CONSTRAINT_TARGET_EXCLUDED: ' + path)
        return self._by_path[path]

    def resolve_source(self, occurrence, context=None):
        entry = self.resolve_context(occurrence, context)
        if entry.path in self._names:
            return self._names[entry.path]
        reason = 'BODYLESS_FRAME_NOT_IMPLEMENTED' if entry.visible and not entry.physical else 'CONSTRAINT_TARGET_EXCLUDED'
        raise ValueError(reason + ': ' + entry.path)

    def rigid_members(self, occurrence, context=None):
        entry = self.resolve_context(occurrence, context)
        if entry.path in self._names:
            return {self._names[entry.path]}, []
        if not entry.visible or entry.physical:
            return set(), [entry.path]
        descendants = [child.path for child in self._contexts
                       if child.physical and (entry.path == 'design_root' or child.path.startswith(entry.path + '+'))]
        if not descendants:
            raise ValueError('BODYLESS_FRAME_NOT_IMPLEMENTED: ' + entry.path)
        return ({self._names[path] for path in descendants if path in self._names},
                [path for path in descendants if path not in self._names])


def source_path(owner):
    return getattr(owner, 'fullPathName', None) or getattr(owner, 'name', 'design_root')


def visible_bodies(owner):
    return tuple(body for body in collection_items(getattr(owner, 'bRepBodies', ()))
                 if occurrence_visible(body))


def link_sources(root, config=None):
    """Capture direct root-context body ownership once for physics and meshes."""
    config = config_mod.load_config(overrides=config)
    configured = config['root_occurrence']
    sources = {}
    occurrences = collection_items(getattr(root, 'allOccurrences', ()))
    for owner in [root] + [occ.component for occ in occurrences]:
        for motion_link in collection_items(getattr(owner, 'motionLinks', ())):
            if not getattr(motion_link, 'isSuppressed', False):
                raise ValueError('MOTION_LINK_NOT_IMPLEMENTED: %s / %s is an active Fusion motion link' %
                                 (source_path(owner), motion_link.name))
    hidden = [source_path(occ) for occ in occurrences if not occurrence_visible(occ)]
    contexts = tuple(OccurrenceContext(owner, 'design_root' if owner is root else source_path(owner),
        _native(owner), occurrence_visible(owner) and not any(source_path(owner).startswith(p + '+') for p in hidden),
        bool(collection_items(getattr(owner, 'bRepBodies', ()))),
        bool(getattr(owner, 'isGrounded', False)), bool(getattr(owner, 'isGroundToParent', False)))
        for owner in occurrences + [root])
    for entry in contexts:
        owner, path = entry.owner, entry.path
        if not entry.visible:
            continue
        mesh_bodies = [body for body in collection_items(getattr(owner, 'meshBodies', ()))
                       if occurrence_visible(body)]
        if mesh_bodies:
            raise ValueError('MESH_BODY_NOT_IMPLEMENTED: %s contains visible mesh bodies %s' %
                             (path, ', '.join(body.name for body in mesh_bodies)))
        bodies = visible_bodies(owner)
        if not bodies:
            continue
        name = 'design_root' if owner is root else sanitize_name(path)
        if name in sources:
            raise ValueError('Duplicate link identity: ' + name)
        sources[name] = LinkSource(owner, bodies)
    if not sources:
        if configured:
            raise ValueError('ROOT_NOT_FOUND: %s has no visible body-owning source' % configured)
        raise ValueError('NO_VISIBLE_BREP_BODIES: no physical links selected')
    sources = SourceRegistry(sources, contexts)
    if configured:
        candidates = [name for name, source in sources.items()
                      if ('design_root' if source.owner is root else source_path(source.owner)) == configured]
        if len(candidates) != 1:
            raise ValueError('ROOT_NOT_FOUND: %s must identify one visible body-owning source by full path' % configured)
    else:
        candidates = [name for name, source in sources.items()
                      if source.owner is not root and source.owner.component.name == 'base_link']
        if not candidates and 'design_root' in sources:
            candidates = ['design_root']
        if not candidates:
            candidates = [sources._names[entry.path] for entry in contexts
                          if entry.grounded and entry.path in sources._names]
        if not candidates:
            children = set()
            for joint in collection_items(getattr(root, 'allJoints', ())) + collection_items(getattr(root, 'allAsBuiltJoints', ())):
                if getattr(joint, 'isSuppressed', False):
                    continue
                try:
                    parent = resolve_source(joint.occurrenceTwo, sources, getattr(joint, 'assemblyContext', None))
                    child = resolve_source(joint.occurrenceOne, sources, getattr(joint, 'assemblyContext', None))
                    if parent == child or sources.grounding_root(parent) != sources.grounding_root(child):
                        children.add(child)
                except ValueError as error:
                    if str(error).startswith(('BODYLESS_FRAME_NOT_IMPLEMENTED', 'AMBIGUOUS_CONSTRAINT_TARGET')):
                        raise ValueError(joint.name + ': ' + str(error)) from error
                    continue
            children.update(sources._names[constraint.child] for constraint in sources.grounding
                            if constraint.parent != 'design_root' and constraint.child in sources._names
                            and constraint.parent in sources._names)
            candidates = [name for name in sources if name not in children]
        if len(candidates) != 1:
            raise ValueError('ROOT_AMBIGUOUS: %s; set root_occurrence to an exact full path or design_root' %
                             ', '.join(source_path(sources[name].owner) for name in candidates or sources))
    selected = candidates[0]
    if selected != 'base_link' and 'base_link' in sources:
        raise ValueError('Duplicate link identity: base_link is reserved for the selected root')
    return SourceRegistry({('base_link' if name == selected else name): source
                           for name, source in sources.items()}, contexts)


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


def export_stl(design, targets, config=None, *, sources=None):
    """Concatenate validated per-body STL triangles without exporting descendants."""
    config = config_mod.load_config(overrides=config)
    root = design.rootComponent
    sources = link_sources(root, config=config) if sources is None else sources
    if sources.keys() != targets.keys():
        raise ValueError('Mesh targets do not match extracted link identities.')
    manager = design.exportManager

    def export(geometry, target, name):
        options = manager.createSTLExportOptions(geometry, str(target))
        if options is None or not hasattr(options, 'unitType'):
            raise RuntimeError('Fusion cannot guarantee millimeter STL units for ' + name)
        options.unitType = adsk.fusion.DistanceUnits.MillimeterDistanceUnits
        options.sendToPrintUtility = False
        options.isBinaryFormat = True
        options.meshRefinement = config_mod.mesh_refinement_value(config, adsk.fusion)
        if not manager.execute(options):
            raise RuntimeError('Fusion mesh export failed for ' + name)
        return read_binary_stl(target)

    for name, source in sources.items():
        target = Path(targets[name])
        with tempfile.TemporaryDirectory(prefix='.link-bodies-', dir=target.parent) as temporary:
            count, payload = 0, bytearray()
            for index, body in enumerate(source.bodies):
                native = getattr(body, 'nativeObject', None)
                part_count, part_payload = export(native if native is not None else body,
                    Path(temporary) / ('%d.stl' % index), name)
                if native is not None:
                    matrix = occurrence_matrix_array(source.owner)
                    if matrix is None:
                        raise ValueError('MESH_FRAME_UNAVAILABLE: ' + name)
                    part_payload = transform_binary_stl(part_payload, matrix)
                count += part_count
                payload.extend(part_payload)
            target.write_bytes(b'fusion360tourdf direct bodies'.ljust(80, b'\0') + struct.pack('<I', count) + payload)


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
