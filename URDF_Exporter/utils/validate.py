"""Fail-closed preflight validation and closed-loop detection (pure Python).

Ideas follow Adriaeik/fusion2URDF (topology warnings before mesh export,
loop-closing joints stashed in a sidecar) and ACDC4Robot (fail-closed
preflight with actionable errors), reduced to this repo's flat
joints_dict model: {joint_name: {parent, child, type, ...}}.
"""


def build_parent_map(joints_dict):
    """Map child link -> list of (joint_name, parent) for tree checks."""
    parent_map = {}
    for joint_name, joint in joints_dict.items():
        child = joint.get('child')
        parent = joint.get('parent')
        if not child or not parent:
            continue
        parent_map.setdefault(child, []).append((joint_name, parent))
    return parent_map


def detect_closing_joints(joints_dict):
    """Split joints into a URDF tree and loop-closing sidecar joints.

    URDF is a tree: every non-root link needs exactly one parent joint.
    When a child link is claimed by two joints (parallel-jaw fingers,
    four-bar followers) or the graph has a cycle, keep the first joint in
    encounter order and mark the rest as closing joints.

    Returns (tree_joints_dict, closing_list, warnings) where each closing
    entry is {joint, parent, child, source: 'auto_detected'}.
    """
    tree = {}
    closing = []
    warnings = []
    claimed_child = {}
    for joint_name, joint in joints_dict.items():
        child = joint.get('child')
        parent = joint.get('parent')
        if not child or not parent:
            warnings.append('Joint %s is missing parent/child; skipped.' % joint_name)
            continue
        if parent == child:
            closing.append({'joint': joint_name, 'parent': parent,
                            'child': child, 'source': 'auto_detected'})
            warnings.append('Joint %s connects %s to itself; moved to closing_joints sidecar.'
                            % (joint_name, child))
            continue
        if child in claimed_child:
            closing.append({'joint': joint_name, 'parent': parent,
                            'child': child, 'source': 'auto_detected'})
            first = claimed_child[child]
            warnings.append('Link %s has two parents (%s via %s, %s via %s); '
                            'keeping the first in the URDF tree and moving %s to closing_joints.'
                            % (child, first[1], first[0], parent, joint_name, joint_name))
            continue
        # Cycle check: is parent already downstream of child in the tree so far?
        if _reaches(child, parent, tree):
            closing.append({'joint': joint_name, 'parent': parent,
                            'child': child, 'source': 'auto_detected'})
            warnings.append('Joint %s (%s -> %s) would close a kinematic loop; '
                            'moved to closing_joints sidecar.' % (joint_name, parent, child))
            continue
        claimed_child[child] = (joint_name, parent)
        tree[joint_name] = joint
    return tree, closing, warnings


def _reaches(start, target, tree):
    """True if target is reachable from start following parent->child edges."""
    children = {}
    for joint in tree.values():
        children.setdefault(joint.get('parent'), []).append(joint.get('child'))
    seen = set()
    stack = [start]
    while stack:
        node = stack.pop()
        if node == target:
            return True
        if node in seen:
            continue
        seen.add(node)
        stack.extend(children.get(node, []))
    return False


def find_roots(joints_dict, links=None):
    """Return candidate root links (never a child), preferring base_link."""
    children = {j.get('child') for j in joints_dict.values() if j.get('child')}
    parents = {j.get('parent') for j in joints_dict.values() if j.get('parent')}
    candidates = [p for p in parents if p not in children]
    if 'base_link' in parents:
        ordered = ['base_link'] + [c for c in candidates if c != 'base_link']
        return ordered
    if links and 'base_link' in links:
        return ['base_link'] + [c for c in candidates if c != 'base_link']
    return candidates


def validate_export(joints_dict, inertial_dict):
    """Fail-closed preflight: returns (errors, warnings) as string lists.

    Errors abort the export; warnings are shown before mesh export burns
    time. Single-link designs (no joints) are valid.
    """
    errors = []
    warnings = []
    if not inertial_dict:
        errors.append('No link components found. Add one component per link.')
        return errors, warnings
    if 'base_link' not in inertial_dict:
        errors.append("There is no base_link. Name the root component 'base_link' "
                      'or let the exporter synthesize it from design-root bodies.')
    for joint_name, joint in joints_dict.items():
        parent = joint.get('parent')
        child = joint.get('child')
        if not parent or not child:
            errors.append('Joint %s is missing parent/child.' % joint_name)
            continue
        if child not in inertial_dict:
            errors.append('Joint %s child link %s has no inertial data. '
                          'Check Component1/Component2 order and visibility.'
                          % (joint_name, child))
        if parent not in inertial_dict:
            errors.append('Joint %s parent link %s has no inertial data.'
                          % (joint_name, parent))
        jtype = joint.get('type')
        if jtype in ('revolute', 'continuous', 'prismatic'):
            axis = joint.get('axis', [0, 0, 0]) or [0, 0, 0]
            try:
                if sum(float(a) * float(a) for a in axis) < 1e-12:
                    warnings.append('Joint %s (%s) has a zero axis vector.' % (joint_name, jtype))
            except Exception:
                warnings.append('Joint %s (%s) has an unreadable axis.' % (joint_name, jtype))
        if jtype == 'revolute':
            try:
                lo = float(joint.get('lower_limit', 0.0))
                hi = float(joint.get('upper_limit', 0.0))
                if hi < lo:
                    errors.append('Joint %s limits inverted: lower %s > upper %s.'
                                  % (joint_name, lo, hi))
            except Exception:
                warnings.append('Joint %s has non-numeric limits.' % joint_name)
    # Disconnected-link check.
    if joints_dict:
        linked = set()
        for joint in joints_dict.values():
            if joint.get('parent'):
                linked.add(joint['parent'])
            if joint.get('child'):
                linked.add(joint['child'])
        for link in inertial_dict:
            if link not in linked and link != 'base_link' and len(inertial_dict) > 1:
                warnings.append('Link %s is not connected to any joint; it will be missing '
                                'from the URDF tree. Put it in a rigid group or joint it.'
                                % link)
    return errors, warnings
