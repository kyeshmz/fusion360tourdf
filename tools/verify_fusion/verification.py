from collections import Counter
from contextlib import contextmanager
from dataclasses import dataclass
import hashlib
import itertools
import json
import math
import marshal
from pathlib import Path
import struct
import subprocess
import sys
import types


IDENTITY = (1, 0, 0, 0, 1, 0, 0, 0, 1)
QUARTER_TURN = (0, -1, 0, 1, 0, 0, 0, 0, 1)


@dataclass(frozen=True)
class Box:
    size_m: tuple
    center_m: tuple
    rotation: tuple = IDENTITY
    mass_kg: float = 1.0


@dataclass(frozen=True)
class Joint:
    name: str
    parent: str
    child: str
    kind: str
    axis_world: tuple = (0, 0, 1)
    limits: tuple = ()
    pivot_world: tuple = (0, 0, 0)
    sample_position: float | None = None


def multiply(a, b):
    return tuple(sum(a[3 * i + k] * b[3 * k + j] for k in range(3))
                 for i in range(3) for j in range(3))


def transpose(a):
    return tuple(a[3 * j + i] for i in range(3) for j in range(3))


def rotate(rotation, point):
    return tuple(sum(rotation[3 * i + k] * point[k] for k in range(3)) for i in range(3))


def transform(pose, point):
    rotation, translation = pose
    return tuple(a + b for a, b in zip(rotate(rotation, point), translation))


def compose(a, b):
    return multiply(a[0], b[0]), transform(a, b[1])


def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def motion_pose(kind, axis, pivot, position):
    if kind == 'fixed':
        return IDENTITY, (0, 0, 0)
    if kind == 'prismatic':
        return IDENTITY, tuple(position*v for v in axis)
    x, y, z = axis
    c, s, t = math.cos(position), math.sin(position), 1-math.cos(position)
    rotation = (t*x*x+c, t*x*y-s*z, t*x*z+s*y,
                t*x*y+s*z, t*y*y+c, t*y*z-s*x,
                t*x*z-s*y, t*y*z+s*x, t*z*z+c)
    return rotation, tuple(a-b for a, b in zip(pivot, rotate(rotation, pivot)))


def bounds(points):
    points = list(points)
    if not points or any(len(p) != 3 or not all(math.isfinite(v) for v in p) for p in points):
        raise ValueError('Bounds require finite three-dimensional points')
    return tuple(min(p[i] for p in points) for i in range(3)), tuple(max(p[i] for p in points) for i in range(3))


def box_corners(box):
    return [transform((box.rotation, box.center_m), corner)
            for corner in itertools.product(*[(-v / 2, v / 2) for v in box.size_m])]


def box_physics(boxes):
    mass = sum(box.mass_kg for box in boxes)
    if mass <= 0:
        raise ValueError('Box mass must be positive')
    center = tuple(sum(box.mass_kg * box.center_m[i] for box in boxes) / mass for i in range(3))
    inertia = [0.0] * 9
    for box in boxes:
        x, y, z = box.size_m
        diagonal = (box.mass_kg * (y*y + z*z) / 12, 0, 0,
                    0, box.mass_kg * (x*x + z*z) / 12, 0,
                    0, 0, box.mass_kg * (x*x + y*y) / 12)
        tensor = multiply(multiply(box.rotation, diagonal), transpose(box.rotation))
        offset = tuple(a - b for a, b in zip(box.center_m, center))
        squared = sum(v*v for v in offset)
        for i in range(3):
            for j in range(3):
                inertia[3*i+j] += tensor[3*i+j] + box.mass_kg * ((squared if i == j else 0) - offset[i]*offset[j])
    return mass, center, tuple(inertia)


def stl_vertices(path):
    """Read binary STL vertices in the file's own units, independently of the exporter."""
    data = Path(path).read_bytes()
    if len(data) < 84:
        raise ValueError('Truncated binary STL header')
    count, = struct.unpack_from('<I', data, 80)
    if count == 0 or len(data) != 84 + 50 * count:
        raise ValueError('Invalid binary STL triangle count or length')
    vertices = []
    for offset in range(84, len(data), 50):
        values = struct.unpack_from('<12fH', data, offset)
        if not all(math.isfinite(v) for v in values[:12]):
            raise ValueError('Non-finite binary STL value')
        vertices.extend(tuple(values[i:i+3]) for i in (3, 6, 9))
    return vertices


def check_box_mesh(vertices, boxes, checks, name):
    groups = [[] for box in boxes]
    for offset in range(0, len(vertices), 3):
        triangle = vertices[offset:offset+3]
        a, b, c = triangle
        normal = cross(tuple(y-x for x, y in zip(a, b)), tuple(y-x for x, y in zip(a, c)))
        checks.equal(name + ' nondegenerate facet %d' % (offset//3), sum(v*v for v in normal) > 1e-24, True)
        owners = []
        for index, box in enumerate(boxes):
            local = [rotate(transpose(box.rotation), tuple(x-y for x, y in zip(p, box.center_m))) for p in triangle]
            inside = all(abs(p[i]) <= box.size_m[i]/2 + 2e-6 for p in local for i in range(3))
            on_face = any(all(abs(p[i]-sign*box.size_m[i]/2) <= 2e-6 for p in local)
                          for i in range(3) for sign in (-1, 1))
            if inside and on_face:
                owners.append(index)
        checks.equal(name + ' facet %d analytic box surface ownership' % (offset//3), len(owners), 1)
        groups[owners[0]].append(triangle)
    for index, (box, triangles) in enumerate(zip(boxes, groups)):
        label = name + ' box %d' % index
        edges = Counter((a, b) for triangle in triangles for a, b in zip(triangle, triangle[1:]+triangle[:1]))
        closed = bool(edges) and all(count == 1 and edges[(b, a)] == 1 for (a, b), count in edges.items())
        checks.equal(label + ' closed oriented manifold', closed, True)
        signed_volume = 0
        for triangle in triangles:
            a, b, c = [tuple(x-y for x, y in zip(p, box.center_m)) for p in triangle]
            signed_volume += sum(x*y for x, y in zip(a, cross(b, c))) / 6
        checks.close(label + ' enclosed volume in m^3', abs(signed_volume), math.prod(box.size_m),
                     absolute=1e-12, relative=1e-5)


class Checks:
    def __init__(self, evidence=None):
        self.evidence = evidence if evidence is not None else []

    def equal(self, name, actual, expected):
        self._record(name, actual == expected, actual, expected)

    def close(self, name, actual, expected, *, absolute=2e-6, relative=0):
        def near(a, b):
            if isinstance(a, (list, tuple)) and isinstance(b, (list, tuple)):
                return len(a) == len(b) and all(near(x, y) for x, y in zip(a, b))
            return math.isfinite(a) and math.isfinite(b) and math.isclose(a, b, abs_tol=absolute, rel_tol=relative)
        self._record(name, near(actual, expected), actual, expected,
                     absolute_tolerance=absolute, relative_tolerance=relative)

    def _record(self, name, passed, actual, expected, **tolerances):
        self.evidence.append(dict(name=name, status='pass' if passed else 'fail',
                                  actual=actual, expected=expected, **tolerances))
        if not passed:
            raise AssertionError('%s differs. Actual %r. Expected %r.' % (name, actual, expected))


def assert_source_unchanged(before, after):
    def compare(a, b, path):
        if type(a) is not type(b):
            raise AssertionError('Source type changed at ' + path)
        if isinstance(a, dict):
            if a.keys() != b.keys():
                raise AssertionError('Source fields changed at ' + path)
            for key in a:
                compare(a[key], b[key], path + '.' + key)
        elif isinstance(a, (list, tuple)):
            if len(a) != len(b):
                raise AssertionError('Source count changed at ' + path)
            for i, (x, y) in enumerate(zip(a, b)):
                compare(x, y, '%s[%s]' % (path, i))
        elif isinstance(a, float):
            if not math.isfinite(a) or not math.isfinite(b) or abs(a - b) > 1e-10:
                raise AssertionError('Source value changed at ' + path)
        elif a != b:
            raise AssertionError('Source value changed at ' + path)
    compare(before, after, 'design')
    return {'status': 'pass', 'numeric_absolute_tolerance': 1e-10}


def entity_evidence(entities):
    result = []
    for path, entity in entities:
        entry = dict(path=path, object_type=entity.objectType)
        try:
            entry['token'] = getattr(entity, 'entityToken', None)
        except RuntimeError as error:
            entry.update(token=None, token_unavailable=str(error))
        result.append(entry)
    return result


def assert_entity_identity(before, after):
    if [path for path, entity in before] != [path for path, entity in after]:
        raise AssertionError('Source entity identities changed')
    for (path, original), (_, current) in zip(before, after):
        if not original.isValid or not current.isValid or original != current:
            raise AssertionError('Source entity identity changed at ' + path)
    return dict(status='pass', retained_entities=len(before), method='retained API references and equality, not token strings')


@contextmanager
def owned_document(app, document_type):
    previous = app.activeDocument
    document = app.documents.add(document_type)
    if document is None or document == previous:
        raise RuntimeError('Fusion did not create a distinct owned document')
    try:
        yield document
    finally:
        try:
            if document.isValid and document.close(False) is False:
                raise RuntimeError('Fusion refused to close the owned fixture document')
        finally:
            if previous is not None:
                if not previous.isValid or previous.activate() is False:
                    raise RuntimeError('Fusion refused to restore the previously active document')


class Report:
    def __init__(self, directory, version, expected_cases, *, provenance=None):
        self.path = Path(directory) / 'report.json'
        self.data = dict(status='running', fusion_version=version, expected_cases=expected_cases,
                         provenance=provenance if provenance is not None else dict(status='unavailable', reason='No provenance supplied'),
                         units=dict(fusion_length='cm', stl_length='mm', checked_length='m',
                                    mass='kg', checked_centroid_inertia='kg m^2'),
                         cases=[], coverage=[dict(name='geometry_reference_invalidation', status='not_exercised',
                         reason='No real original-geometry invalidation has been induced by this runner.')])
        self.save()

    def start_case(self, fixture, mode, output):
        case = dict(fixture=fixture, mode=mode, status='running', output_path=str(output),
                    assertions=[], source_preservation={'status': 'not_exercised'},
                    cleanup={'status': 'not_exercised'})
        self.data['cases'].append(case)
        self.save()
        return case

    def save(self, *, finished=False):
        cases = self.data['cases']
        if any(case['status'] == 'fail' for case in cases) or self.data['provenance']['status'] == 'fail':
            self.data['status'] = 'fail'
        elif finished:
            self.data['status'] = ('pass' if len(cases) == self.data['expected_cases']
                                   and all(case['status'] == 'pass' for case in cases) else 'fail')
        else:
            self.data['status'] = 'running'
        temporary = self.path.with_suffix('.json.tmp')
        temporary.write_text(json.dumps(self.data, indent=2, allow_nan=False) + '\n', encoding='utf-8')
        temporary.replace(self.path)


def source_hashes(repository):
    repository = Path(repository).resolve()
    inputs = {}
    for folder in ('URDF_Exporter', 'tools'):
        for path in sorted((repository / folder).rglob('*')):
            if path.is_file() and '__pycache__' not in path.parts and path.suffix != '.pyc':
                inputs[path.relative_to(repository).as_posix()] = hashlib.sha256(path.read_bytes()).hexdigest()
    return inputs


def capture_provenance(repository, config):
    repository = Path(repository).resolve()
    result = dict(status='recorded', repository=str(repository), python_version=sys.version,
                  requested_config=config, inputs_sha256=source_hashes(repository))
    try:
        def git(*arguments):
            return subprocess.run(['git', '-C', str(repository), *arguments], check=True,
                                  capture_output=True, text=True, timeout=10).stdout.strip()
        revision, status = git('rev-parse', 'HEAD'), git('status', '--porcelain=v1', '--untracked-files=all')
        result['git'] = dict(status='available', revision=revision, dirty=bool(status), porcelain=status)
    except (OSError, subprocess.SubprocessError) as error:
        result['git'] = dict(status='unavailable', reason=str(error))
    return result


def loaded_code_provenance(namespace, path):
    path = Path(path).resolve()
    compiled = compile(path.read_bytes(), str(path), 'exec', dont_inherit=True, optimize=sys.flags.optimize)
    expected = {}
    def collect(code):
        expected[code.co_qualname] = code
        for value in code.co_consts:
            if isinstance(value, types.CodeType):
                collect(value)
    collect(compiled)
    functions = []
    for value in list(namespace.values()):
        if getattr(value, '__module__', None) != namespace.get('__name__'):
            continue
        if isinstance(value, types.FunctionType):
            functions.append(value)
        elif isinstance(value, type):
            for method in vars(value).values():
                method = method.__func__ if isinstance(method, (staticmethod, classmethod)) else method
                if isinstance(method, types.FunctionType):
                    functions.append(method)
    signatures = {}
    for function in functions:
        code = function.__code__
        if Path(code.co_filename).resolve() != path:
            continue
        signatures[code.co_qualname] = dict(bytecode_sha256=hashlib.sha256(marshal.dumps(code)).hexdigest(),
                                           first_line=code.co_firstlineno, matches_disk=code == expected.get(code.co_qualname))
    return dict(path=str(path), disk_sha256=hashlib.sha256(path.read_bytes()).hexdigest(), functions=signatures,
                status='match' if all(item['matches_disk'] for item in signatures.values()) else 'mismatch')


def _vector(element, attribute, default):
    values = tuple(float(v) for v in element.get(attribute, default).split())
    if len(values) != 3 or not all(math.isfinite(v) for v in values):
        raise ValueError('URDF vector must have three finite values')
    return values


def _origin(element):
    if element is None:
        return IDENTITY, (0, 0, 0)
    roll, pitch, yaw = _vector(element, 'rpy', '0 0 0')
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return ((cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr,
             sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr,
             -sp, cp*sr, cp*cr), _vector(element, 'xyz', '0 0 0'))


def _urdf_frames(joints, positions):
    pending, frames = list(joints), {'base_link': (IDENTITY, (0, 0, 0))}
    while pending:
        remaining = []
        for joint in pending:
            parent, child = joint.find('parent').get('link'), joint.find('child').get('link')
            if parent not in frames:
                remaining.append(joint)
                continue
            if child in frames:
                raise AssertionError('Duplicate parent or cycle at ' + child)
            pose = _origin(joint.find('origin'))
            if joint.get('type') != 'fixed':
                axis = _vector(joint.find('axis'), 'xyz', '1 0 0')
                pose = compose(pose, motion_pose(joint.get('type'), axis, (0, 0, 0), positions.get(joint.get('name'), 0)))
            frames[child] = compose(frames[parent], pose)
        if len(remaining) == len(pending):
            raise AssertionError('URDF is disconnected from base_link')
        pending = remaining
    return frames


def _fixture_displacements(joints, positions):
    pending, displacements = list(joints), {'base_link': (IDENTITY, (0, 0, 0))}
    while pending:
        remaining = []
        for joint in pending:
            if joint.parent not in displacements:
                remaining.append(joint)
                continue
            if joint.child in displacements:
                raise ValueError('Fixture joints must be a tree')
            motion = motion_pose(joint.kind, joint.axis_world, joint.pivot_world, positions.get(joint.name, 0))
            displacements[joint.child] = compose(displacements[joint.parent], motion)
        if len(remaining) == len(pending):
            raise ValueError('Fixture joints are disconnected')
        pending = remaining
    return displacements


def pose_expectations(boxes_by_link, joints, zero_frames):
    """Build independent downstream expectations from literal CAD fixture inputs."""
    def matrix(pose):
        rotation, translation = pose
        return [[*rotation[3*i:3*i+3], translation[i]] for i in range(3)] + [[0, 0, 0, 1]]

    target = {joint.name: joint.sample_position for joint in joints if joint.kind != 'fixed'}
    states = {'zero': dict.fromkeys(target, 0.0), 'moved': target}
    result = {}
    for name, positions in states.items():
        displacements = _fixture_displacements(joints, positions)
        result[name] = dict(positions=positions,
            link_poses={link: matrix(compose(displacements[link], zero_frames[link])) for link in boxes_by_link},
            visual_poses={link: matrix(displacements[link]) for link in boxes_by_link},
            bounds={link: bounds(transform(displacements[link], point) for box in boxes for point in box_corners(box))
                    for link, boxes in boxes_by_link.items()})
    return result


def verify_package(package, boxes_by_link, cad_bounds, expected_joints, checks=None):
    """Check real mesh bytes and XML against analytic boxes in root-frame SI units."""
    import xml.etree.ElementTree as ET
    package = Path(package).resolve()
    checks = checks if checks is not None else Checks()
    xml_files = list((package / 'urdf').glob('*.xacro'))
    checks.equal('robot XML count', len(xml_files), 1)
    robot = ET.parse(xml_files[0]).getroot()
    links, joints = robot.findall('link'), robot.findall('joint')
    link_names = [link.get('name') for link in links]
    joint_names = [joint.get('name') for joint in joints]
    checks.equal('unique link names', len(link_names), len(set(link_names)))
    checks.equal('unique joint names', len(joint_names), len(set(joint_names)))
    checks.equal('link identities', sorted(link_names), sorted(boxes_by_link))
    checks.equal('joint identities', sorted(joint_names), sorted(j.name for j in expected_joints))
    checks.equal('tree edge count', len(joints), len(links) - 1)
    frames = _urdf_frames(joints, {})
    checks.equal('connected links', sorted(frames), sorted(boxes_by_link))
    for expected in expected_joints:
        joint = next(j for j in joints if j.get('name') == expected.name)
        checks.equal(expected.name + ' type', joint.get('type'), expected.kind)
        checks.equal(expected.name + ' parent', joint.find('parent').get('link'), expected.parent)
        checks.equal(expected.name + ' child', joint.find('child').get('link'), expected.child)
        if expected.kind != 'fixed':
            axis = _vector(joint.find('axis'), 'xyz', '1 0 0')
            checks.close(expected.name + ' world axis', rotate(frames[expected.child][0], axis), expected.axis_world)
            checks.close(expected.name + ' unit world axis', sum(v*v for v in expected.axis_world), 1, absolute=1e-10)
            if expected.kind == 'revolute':
                offset = tuple(a-b for a, b in zip(frames[expected.child][1], expected.pivot_world))
                checks.close(expected.name + ' world axis line', cross(offset, expected.axis_world), (0, 0, 0))
            limit = joint.find('limit')
            checks.close(expected.name + ' limits', tuple(float(limit.get(k)) for k in ('lower', 'upper')),
                         expected.limits, absolute=1e-8)
    mesh_data, inertial_origins = {}, {}
    for link in links:
        name = link.get('name')
        boxes = boxes_by_link[name]
        expected_bounds = bounds(p for box in boxes for p in box_corners(box))
        checks.close(name + ' analytic versus direct CAD bounds', cad_bounds[name], expected_bounds)
        for kind in ('visual', 'collision'):
            geometry = link.find(kind)
            mesh = geometry.find('geometry/mesh')
            if mesh is None:
                raise AssertionError('Expected mesh ' + kind + ' for ' + name)
            prefix = 'package://' + package.name + '/'
            uri = mesh.get('filename')
            if not uri.startswith(prefix):
                raise ValueError('Unexpected mesh package URI ' + uri)
            path = (package / uri[len(prefix):]).resolve()
            if package not in path.parents:
                raise ValueError('Mesh URI escapes package')
            vertices = stl_vertices(path)
            vertices_si = [tuple(v * .001 for v in p) for p in vertices]
            checks.close(name + ' ' + kind + ' raw root-frame STL bounds in meters',
                         bounds(vertices_si), expected_bounds)
            check_box_mesh(vertices_si, boxes, checks, name + ' ' + kind)
            scale = _vector(mesh, 'scale', '1 1 1')
            origin = _origin(geometry.find('origin'))
            scaled = [tuple(a*b for a, b in zip(p, scale)) for p in vertices]
            mesh_data[name, kind] = origin, scaled
            world = compose(frames[name], origin)
            actual_bounds = bounds(transform(world, p) for p in scaled)
            checks.close(name + ' zero-pose ' + kind + ' world bounds', actual_bounds, expected_bounds)
        mass, center, tensor = box_physics(boxes)
        inertial = link.find('inertial')
        inertial_origins[name] = _origin(inertial.find('origin'))
        physics_pose = compose(frames[name], inertial_origins[name])
        checks.close(name + ' mass in kg', float(inertial.find('mass').get('value')), mass, absolute=1e-9, relative=1e-6)
        checks.close(name + ' world COM in meters', physics_pose[1], center)
        i = inertial.find('inertia')
        local_tensor = tuple(float(i.get(key)) for key in ('ixx', 'ixy', 'ixz', 'ixy', 'iyy', 'iyz', 'ixz', 'iyz', 'izz'))
        world_tensor = multiply(multiply(physics_pose[0], local_tensor), transpose(physics_pose[0]))
        checks.close(name + ' world centroid inertia in kg m^2', world_tensor, tensor, absolute=1e-10, relative=1e-5)
    samples = {}
    for joint in expected_joints:
        if joint.kind == 'fixed':
            continue
        low, high = joint.limits
        sample = joint.sample_position
        if sample is None:
            sample = (high/2 if high > 0 else low/2) if low <= 0 <= high else (low+high)/2
        if not math.isfinite(sample) or sample == 0 or not low <= sample <= high:
            raise ValueError('Fixture needs a finite, nonzero in-range sample for ' + joint.name)
        samples[joint.name] = sample
    states = [(name + ' sample', {name: value}) for name, value in samples.items()]
    if len(samples) > 1:
        states.append(('combined sample', samples))
    for label, positions in states:
        moved_frames = _urdf_frames(joints, positions)
        expected_motion = _fixture_displacements(expected_joints, positions)
        for name, boxes in boxes_by_link.items():
            displacement = expected_motion[name]
            expected_bounds = bounds(transform(displacement, p) for box in boxes for p in box_corners(box))
            for kind in ('visual', 'collision'):
                origin, vertices = mesh_data[name, kind]
                pose = compose(moved_frames[name], origin)
                checks.close(label + ' ' + name + ' ' + kind + ' moved world bounds',
                             bounds(transform(pose, p) for p in vertices), expected_bounds)
            expected_center = transform(displacement, box_physics(boxes)[1])
            actual_center = compose(moved_frames[name], inertial_origins[name])[1]
            checks.close(label + ' ' + name + ' moved world COM', actual_center, expected_center)
    return checks.evidence
