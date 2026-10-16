#!/usr/bin/env python3
"""Offline URDF/xacro linter (stdlib only, no ROS/Fusion needed).

De-facto validation for URDF is ``check_urdf`` from urdfdom, which needs a
ROS install or apt package. This script covers the structural checks that
catch 90% of broken exporter output without any dependency:

- well-formed XML, ``<robot name=...>`` root
- unique link names; inertial mass/inertia sane; visual+collision present
- joints reference existing links; origin xyz/rpy parse; movable joints
  have a nonzero axis and (revolute/prismatic) sane limits
- kinematic graph is a tree: one root, one parent per link, no cycles
- ``package://`` mesh files exist when ``--meshes`` is given

``<xacro:include filename="$(find ...)"/>`` lines cannot resolve offline
and are skipped (noted in the report). For full expansion use ROS xacro
first, then lint the expanded file.

Exit code: 0 = no errors (warnings ok), 1 = errors. ``--strict`` fails on
warnings too.
"""

import argparse
import os
import sys
import xml.etree.ElementTree as ET

JOINT_TYPES = {'fixed', 'revolute', 'continuous', 'prismatic', 'floating', 'planar'}
LIMITED_TYPES = {'revolute', 'prismatic'}
AXIS_TYPES = {'revolute', 'continuous', 'prismatic'}


def _parse_vec(text, count=3):
    try:
        values = [float(v) for v in str(text or '').split()]
    except Exception:
        return None
    if len(values) != count:
        return None
    return values


def lint_file(path, meshes_dir=None):
    """Lint one URDF/xacro file. Returns (errors, warnings) string lists."""
    errors, warnings = [], []
    try:
        tree = ET.parse(path)
    except ET.ParseError as exc:
        return ['XML parse error in %s: %s' % (path, exc)], []
    except OSError as exc:
        return ['Cannot read %s: %s' % (path, exc)], []
    root = tree.getroot()
    if root.tag != 'robot':
        errors.append('Root element is <%s>, expected <robot>.' % root.tag)
        return errors, warnings
    if not root.get('name'):
        errors.append('<robot> is missing the name attribute.')

    links = {}
    for link in root.findall('link'):
        name = link.get('name')
        if not name:
            errors.append('<link> without a name attribute.')
            continue
        if name in links:
            errors.append('Duplicate link name: %s.' % name)
            continue
        links[name] = link
        inertial = link.find('inertial')
        if inertial is None:
            warnings.append('Link %s has no <inertial>.' % name)
        else:
            mass = inertial.find('mass')
            try:
                mass_value = float(mass.get('value')) if mass is not None else None
            except Exception:
                mass_value = None
            if mass_value is None:
                errors.append('Link %s <mass> value is not a number.' % name)
            elif mass_value <= 0:
                warnings.append('Link %s mass is not positive: %s.' % (name, mass_value))
            inertia = inertial.find('inertia')
            if inertia is None:
                warnings.append('Link %s has no <inertia>.' % name)
            else:
                for key in ('ixx', 'iyy', 'izz'):
                    try:
                        value = float(inertia.get(key))
                    except Exception:
                        errors.append('Link %s inertia %s is not a number.' % (name, key))
                        continue
                    if value < 0:
                        errors.append('Link %s inertia %s is negative.' % (name, key))
                    elif value == 0:
                        warnings.append('Link %s inertia %s is zero.' % (name, key))
        for section in ('visual', 'collision'):
            elem = link.find(section)
            if elem is None:
                warnings.append('Link %s has no <%s>.' % (name, section))
                continue
            origin = elem.find('origin')
            if origin is not None:
                if _parse_vec(origin.get('xyz')) is None:
                    errors.append('Link %s <%s> origin xyz malformed.' % (name, section))
                if _parse_vec(origin.get('rpy')) is None:
                    errors.append('Link %s <%s> origin rpy malformed.' % (name, section))
    if not links:
        errors.append('No <link> elements found.')
        return errors, warnings

    joints = {}
    child_to_parent = {}
    for joint in root.findall('joint'):
        name = joint.get('name')
        jtype = joint.get('type')
        if not name:
            errors.append('<joint> without a name attribute.')
            continue
        if name in joints:
            errors.append('Duplicate joint name: %s.' % name)
            continue
        if jtype not in JOINT_TYPES:
            errors.append('Joint %s has unknown type %r.' % (name, jtype))
            continue
        joints[name] = joint
        parent = joint.find('parent')
        child = joint.find('child')
        parent_link = parent.get('link') if parent is not None else None
        child_link = child.get('link') if child is not None else None
        if not parent_link or not child_link:
            errors.append('Joint %s is missing parent/child link.' % name)
            continue
        if parent_link not in links:
            errors.append('Joint %s parent link %s does not exist.' % (name, parent_link))
        if child_link not in links:
            errors.append('Joint %s child link %s does not exist.' % (name, child_link))
        if parent_link == child_link:
            errors.append('Joint %s connects %s to itself.' % (name, child_link))
        if child_link in child_to_parent:
            errors.append('Link %s has two parents (%s, %s); URDF must be a tree.'
                          % (child_link, child_to_parent[child_link], parent_link))
        else:
            child_to_parent[child_link] = parent_link
        origin = joint.find('origin')
        if origin is not None:
            if _parse_vec(origin.get('xyz')) is None:
                errors.append('Joint %s origin xyz malformed.' % name)
            if _parse_vec(origin.get('rpy')) is None:
                errors.append('Joint %s origin rpy malformed.' % name)
        if jtype in AXIS_TYPES:
            axis = joint.find('axis')
            if axis is None:
                errors.append('Joint %s (%s) is missing <axis>.' % (name, jtype))
            else:
                values = _parse_vec(axis.get('xyz'))
                if values is None:
                    errors.append('Joint %s axis xyz malformed.' % name)
                elif sum(v * v for v in values) < 1e-12:
                    errors.append('Joint %s axis is zero.' % name)
        if jtype in LIMITED_TYPES:
            limit = joint.find('limit')
            if limit is None:
                errors.append('Joint %s (%s) is missing <limit>.' % (name, jtype))
            else:
                try:
                    lower = float(limit.get('lower'))
                    upper = float(limit.get('upper'))
                except Exception:
                    errors.append('Joint %s limits are not numbers.' % name)
                else:
                    if upper < lower:
                        errors.append('Joint %s limits inverted (lower %s > upper %s).'
                                      % (name, lower, upper))
                for key in ('effort', 'velocity'):
                    try:
                        float(limit.get(key))
                    except Exception:
                        warnings.append('Joint %s limit %s is missing/not a number.' % (name, key))

    # Tree check: roots are links that are never a child.
    children = set(child_to_parent)
    roots = [link for link in links if link not in children]
    if len(roots) == 0:
        errors.append('Kinematic graph has no root (cycle?).')
    elif len(roots) > 1:
        warnings.append('Multiple roots %s; expected one (usually base_link).' % sorted(roots))
    # Cycle check via DFS from each root over parent->child edges.
    descendants = {}
    for joint in root.findall('joint'):
        parent = joint.find('parent')
        child = joint.find('child')
        if parent is not None and child is not None:
            descendants.setdefault(parent.get('link'), []).append(child.get('link'))
    visited_global = set()

    def visit(node, stack):
        if node in stack:
            errors.append('Kinematic cycle detected at link %s.' % node)
            return
        if node in visited_global:
            return
        visited_global.add(node)
        for nxt in descendants.get(node, []):
            visit(nxt, stack + [node])

    for start in roots:
        visit(start, [])

    # Mesh existence (only when the caller points at the meshes directory).
    if meshes_dir is not None:
        for link_name, link in links.items():
            for section in ('visual', 'collision'):
                elem = link.find(section)
                if elem is None:
                    continue
                mesh = elem.find('geometry/mesh')
                if mesh is None:
                    continue
                filename = mesh.get('filename') or ''
                if filename.startswith('package://'):
                    rest = filename[len('package://'):]
                    parts = rest.split('/', 1)
                    rel = parts[1] if len(parts) == 2 else rest
                    candidate = (rel[7:] if rel.startswith('meshes/')
                                 else os.path.basename(rel))
                    full = os.path.join(meshes_dir, candidate)
                    if not os.path.exists(full):
                        errors.append('Link %s <%s> mesh missing: %s.' % (link_name, section, full))

    seen = set()
    includes = []
    for elem in root:
        tag = elem.tag if isinstance(elem.tag, str) else ''
        if tag.endswith('include'):
            key = (tag, elem.get('filename'))
            if key not in seen:
                seen.add(key)
                includes.append(elem)
    if includes:
        warnings.append('%d <xacro:include> element(s) skipped offline (needs `$(find ...)` resolution).'
                        % len(includes))
    return errors, warnings


def main(argv=None):
    parser = argparse.ArgumentParser(description='Lint a URDF/xacro file (stdlib only).')
    parser.add_argument('path', help='Path to .urdf or .xacro file')
    parser.add_argument('--meshes', default=None, help='Directory holding mesh files')
    parser.add_argument('--strict', action='store_true', help='Fail on warnings too')
    args = parser.parse_args(argv)
    errors, warnings = lint_file(args.path, args.meshes)
    for message in errors:
        print('ERROR: ' + message)
    for message in warnings:
        print('WARNING: ' + message)
    print('URDF lint: %d error(s), %d warning(s): %s'
          % (len(errors), len(warnings), args.path))
    if errors or (args.strict and warnings):
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
