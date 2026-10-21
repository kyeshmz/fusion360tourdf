"""Read-only Fusion MeshBody snapshots with explicit, caller-owned physics."""

from dataclasses import dataclass
import math
from numbers import Real
import struct

from .mesh_geometry import CancelledError, ResourceLimitError, mass_properties, validate_mesh
from .stl import transform_binary_stl


IDENTITY = (1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1)
TENSOR_KEYS = ('ixx', 'iyy', 'izz', 'ixy', 'iyz', 'ixz')


@dataclass(frozen=True)
class MeshSnapshot:
    original: object
    triangle_count: int
    payload: bytes
    mass: float
    center_cm: tuple
    central_kg_cm2: tuple
    lower_cm: tuple
    upper_cm: tuple
    provenance: dict


def check_cancelled(cancelled):
    if cancelled is not None and cancelled():
        raise CancelledError('Mesh input export cancelled')


def _valid(value, label):
    if value.isValid is not True:
        raise ValueError('INVALID_MESH_INPUT: ' + label)


def _count(value, label, maximum):
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError('Invalid mesh ' + label)
    if value > maximum:
        raise ResourceLimitError('Mesh exceeds ' + label + ' allowance')
    return value


def _area_normal(points):
    a, b, c = points
    u, v = ([b[i] - a[i] for i in range(3)], [c[i] - a[i] for i in range(3)])
    normal = (u[1]*v[2] - u[2]*v[1], u[2]*v[0] - u[0]*v[2], u[0]*v[1] - u[1]*v[0])
    length = math.hypot(*normal)
    if not math.isfinite(length) or not length:
        raise ValueError('Degenerate or unrepresentable mesh triangle')
    return tuple(value / length for value in normal)


def _payload(triangles, cancelled):
    payload = bytearray()
    for triangle in triangles:
        check_cancelled(cancelled)
        try:
            packed = struct.pack('<9f', *(v * 10 for point in triangle for v in point))
        except (OverflowError, struct.error) as error:
            raise ValueError('Mesh cannot be represented in binary STL millimeters') from error
        values = struct.unpack('<9f', packed)
        if not all(math.isfinite(value) for value in values):
            raise ValueError('Nonfinite binary STL mesh coordinate')
        points = tuple(tuple(values[k:k+3]) for k in (0, 3, 6))
        payload.extend(struct.pack('<12fH', *_area_normal(points), *values, 0))
    return bytes(payload)


def _triangles(payload):
    return tuple(tuple(tuple(row[k:k+3]) for k in (3, 6, 9))
                 for row in struct.iter_unpack('<12fH', payload))


def _native_triangles(native, controls, cancelled):
    _valid(native, 'native MeshBody')
    if native.assemblyContext is not None:
        raise ValueError('MESH_FRAME_UNAVAILABLE: native mesh still has assembly context')
    mesh = native.displayMesh
    _valid(mesh, 'MeshBody.displayMesh')
    count = _count(mesh.triangleCount, 'max_triangles', controls['max_triangles'])
    nodes = _count(mesh.nodeCount, 'max_nodes', controls['max_nodes'])
    coordinates, indices = mesh.nodeCoordinatesAsDouble, mesh.nodeIndices
    if len(coordinates) != nodes * 3 or len(indices) != count * 3:
        raise ValueError('Mesh coordinate/index/count mismatch')
    points = []
    for offset in range(0, len(coordinates), 3):
        check_cancelled(cancelled)
        point = coordinates[offset:offset+3]
        if any(isinstance(v, bool) or not isinstance(v, Real) for v in point):
            raise ValueError('Mesh coordinates require finite numbers')
        try:
            point = tuple(float(v) for v in point)
        except (OverflowError, ValueError) as error:
            raise ValueError('Nonfinite mesh coordinate') from error
        if not all(math.isfinite(v) for v in point):
            raise ValueError('Nonfinite mesh coordinate')
        points.append(point)
    if any(isinstance(i, bool) or not isinstance(i, int) or not 0 <= i < nodes for i in indices):
        raise ValueError('Mesh node index must be an in-range integer')
    triangles = []
    for k in range(0, len(indices), 3):
        check_cancelled(cancelled)
        triangle = tuple(points[indices[k+j]] for j in range(3))
        _area_normal(triangle)
        triangles.append(triangle)
    return tuple(triangles)


def _supplied_physics(policy, matrix):
    center = policy['center_of_mass_m']
    xx, yy, zz, xy, yz, xz = policy['inertia_kg_m2']
    tensor = ((xx, xy, xz), (xy, yy, yz), (xz, yz, zz))
    if policy['frame'] == 'native':
        rows = [matrix[4*i:4*i+3] for i in range(3)]
        center = [math.fsum(a*b for a, b in zip(row, center)) + matrix[4*i+3] / 100
                  for i, row in enumerate(rows)]
        tensor = [[math.fsum(rows[i][k] * tensor[k][l] * rows[j][l]
                            for k in range(3) for l in range(3)) for j in range(3)] for i in range(3)]
    return policy['mass_kg'], center, (tensor[0][0], tensor[1][1], tensor[2][2],
                                      tensor[0][1], tensor[1][2], tensor[0][2])


class MeshInputs:
    """One export's policy bindings and native-geometry read cache."""

    def __init__(self, controls, cancelled=None):
        self.controls = controls
        self.cancelled = cancelled
        self._native = []
        self._validated_native = []
        self._used = set()
        self._uniform_used = False
        self._triangle_count = 0
        self._pair_checks = 0

    def capture(self, body, owner, path, matrix):
        check_cancelled(self.cancelled)
        controls = self.controls
        if controls is None:
            raise ValueError('MESH_PHYSICS_REQUIRED: %s / %s needs explicit mesh_physics density or supplied SI physics' %
                             (path, body.name))
        selected = [i for i, policy in enumerate(controls['bodies'])
                    if (policy['occurrence'], policy['body']) == (path, body.name)]
        if selected:
            index = selected[0]
            if index in self._used:
                raise ValueError('AMBIGUOUS_MESH_SELECTOR: %s / %s matches multiple bodies; use uniform density without this selector' %
                                 (path, body.name))
            self._used.add(index)
            policy = controls['bodies'][index]
        elif 'density_kg_m3' in controls:
            policy = controls
            self._uniform_used = True
        else:
            raise ValueError('MESH_PHYSICS_REQUIRED: %s / %s has no exact mesh physics selector' % (path, body.name))
        _valid(body, path + ' / ' + body.name)
        native = body.nativeObject
        context = body.assemblyContext
        if path == 'design_root':
            if native is not None or context is not None:
                raise ValueError('MESH_FRAME_UNAVAILABLE: root mesh is not native')
            native, matrix = body, IDENTITY
        else:
            _valid(owner, path)
            if native is None or context is None or not (context is owner or context == owner):
                raise ValueError('MESH_FRAME_UNAVAILABLE: occurrence mesh must have its exact owning proxy context')
            if matrix is None:
                raise ValueError('MESH_FRAME_UNAVAILABLE: ' + path)
        transform_binary_stl(b'', matrix)
        triangles = next((triangles for previous, triangles in self._native
                          if previous is native or previous == native), None)
        if triangles is None:
            triangles = _native_triangles(native, controls, self.cancelled)
            self._native.append((native, triangles))
        self._triangle_count += len(triangles)
        if self._triangle_count > controls['max_triangles']:
            raise ResourceLimitError('Selected mesh inputs exceed aggregate max_triangles')
        if 'density_kg_m3' in policy:
            if not any(previous is native or previous == native for previous in self._validated_native):
                self._consume_pairs(len(triangles))
                validate_mesh(triangles, max_triangles=controls['max_triangles'],
                              max_pair_checks=controls['max_pair_checks'], cancelled=self.cancelled)
                self._validated_native.append(native)
            self._consume_pairs(len(triangles))
        root_triangles = tuple(tuple(tuple(math.fsum(matrix[4*i+j] * point[j] for j in range(3)) + matrix[4*i+3]
                                          for i in range(3)) for point in triangle) for triangle in triangles)
        payload = _payload(root_triangles, self.cancelled)
        emitted = _triangles(payload)
        closed, oriented = native.isClosed, native.isOriented
        if not isinstance(closed, bool) or not isinstance(oriented, bool):
            raise ValueError('MeshBody topology flags must be readable booleans')
        provenance = {'source_type': 'Fusion MeshBody', 'occurrence': path, 'body': body.name,
                      'geometry': 'native.displayMesh', 'source_length_unit': 'cm',
                      'placement': 'native-to-design-root-once', 'triangle_count': len(emitted),
                      'physics_is_native': False, 'geometry_quantization': 'binary-STL-float32-mm',
                      'reported_is_closed': closed, 'reported_is_oriented': oriented}
        if 'density_kg_m3' in policy:
            physics = mass_properties(emitted, length_unit='mm', density_kg_m3=policy['density_kg_m3'],
                                      max_triangles=controls['max_triangles'],
                                      max_pair_checks=controls['max_pair_checks'], cancelled=self.cancelled)
            mass, center = physics['mass_kg'], physics['center_of_mass_m']
            tensor = tuple(physics['inertia_kg_m2'][key] for key in TENSOR_KEYS)
            provenance.update(policy='explicit-uniform-density', integration=physics['provenance'],
                              source_validation='exact-single-embedded-shell-before-STL-quantization')
        else:
            mass, center, tensor = _supplied_physics(policy, matrix)
            provenance.update(policy='caller-supplied-central-SI-physics', frame=policy['frame'],
                              supplied_provenance=policy['provenance'],
                              supplied={'mass_kg': policy['mass_kg'],
                                        'center_of_mass_m': list(policy['center_of_mass_m']),
                                        'inertia_kg_m2': list(policy['inertia_kg_m2'])},
                              geometry_validation='finite-nondegenerate-triangles-only; open surfaces allowed')
        values = (mass, *center, *tensor)
        if not all(math.isfinite(v) for v in values):
            raise ValueError('Mesh physics is not representable in the root frame')
        lower = tuple(min(p[i] for t in emitted for p in t) / 10 for i in range(3))
        upper = tuple(max(p[i] for t in emitted for p in t) / 10 for i in range(3))
        return MeshSnapshot(body, len(emitted), payload, mass, tuple(v * 100 for v in center),
                            tuple(v * 10000 for v in tensor), lower, upper, provenance)

    def _consume_pairs(self, count):
        self._pair_checks += count * (count - 1) // 2
        if self._pair_checks > self.controls['max_pair_checks']:
            raise ResourceLimitError('Selected mesh inputs exceed aggregate max_pair_checks')

    def finish(self):
        if self.controls is None:
            return
        unused = [policy for i, policy in enumerate(self.controls['bodies']) if i not in self._used]
        if unused:
            raise ValueError('UNUSED_MESH_SELECTOR: ' + repr([(p['occurrence'], p['body']) for p in unused]))
        if 'density_kg_m3' in self.controls and not self._uniform_used:
            raise ValueError('UNUSED_MESH_DENSITY: uniform density matched no selected mesh body')
