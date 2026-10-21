"""One exact convex hull of binary STL float32 vertices under an explicit source policy."""

from fractions import Fraction
import math
import struct

from .mesh_geometry import MeshGeometryError, ResourceLimitError, CancelledError, validate_mesh

__all__ = ['hull_stl', 'HULL_DEFAULTS', 'HULL_MAXIMUMS', 'HULL_SOURCE_POLICIES']

HULL_SOURCE_POLICIES = ('closed_shell', 'triangle_vertices')

HULL_DEFAULTS = {'max_vertices': 4096, 'max_faces': 8192, 'max_work': 2000000,
                 'max_triangles': 10000, 'max_pair_checks': 2000000,
                 'max_output_bytes': 4194304, 'max_pieces': 1024}
HULL_MAXIMUMS = {'max_vertices': 8192, 'max_faces': 16384, 'max_work': 10000000,
                 'max_triangles': 100000, 'max_pair_checks': 10000000,
                 'max_output_bytes': 67108864, 'max_pieces': 65536}


def _sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def _normal(points, face):
    a, b, c = (points[i] for i in face)
    return _cross(_sub(b, a), _sub(c, a))


class _Budget:
    def __init__(self, maximum, cancelled):
        self.maximum, self.cancelled, self.used = maximum, cancelled, 0

    def check(self):
        if self.cancelled is not None and self.cancelled():
            raise CancelledError('Convex hull cancelled')

    def spend(self):
        self.used += 1
        if self.used > self.maximum:
            raise ResourceLimitError('Convex hull exceeds max_work')
        if self.used % 128 == 0:
            self.check()


def _face_limit(faces, maximum):
    if len(faces) > maximum:
        raise ResourceLimitError('Convex hull exceeds max_faces')


def _source(data, max_triangles, max_vertices, budget):
    size = memoryview(data).nbytes
    if size > 84 + 50 * max_triangles:
        raise ResourceLimitError('Binary STL exceeds max_triangles byte allowance')
    if size < 84:
        raise MeshGeometryError('Empty or truncated binary STL')
    data = bytes(data)
    count = struct.unpack_from('<I', data, 80)[0]
    if not count or len(data) != 84 + 50 * count:
        raise MeshGeometryError('Binary STL triangle count/length mismatch')
    vertices = set()
    for row in struct.iter_unpack('<12fH', data[84:]):
        budget.spend()
        if not all(math.isfinite(v) for v in row[:12]):
            raise MeshGeometryError('Nonfinite binary STL value')
        triangle = tuple(tuple(row[k:k + 3]) for k in (3, 6, 9))
        for point in triangle:
            budget.spend()
            vertices.add(point)
            if len(vertices) > max_vertices:
                raise ResourceLimitError('Convex hull source exceeds max_vertices')
        yield triangle


def _triangle_vertices(data, max_triangles, max_vertices, budget):
    vertices, denominator, count = set(), 1, 0
    for triangle in _source(data, max_triangles, max_vertices, budget):
        budget.check()
        budget.spend()
        exact = tuple(tuple(Fraction(v) for v in point) for point in triangle)
        if _cross(_sub(exact[1], exact[0]), _sub(exact[2], exact[0])) == (0, 0, 0):
            raise MeshGeometryError('Degenerate source triangle')
        vertices.update(triangle)
        denominator = max(denominator, *(v.denominator for point in exact for v in point))
        count += 1
    points = []
    for point in sorted(vertices):
        budget.spend()
        points.append(tuple(v.as_integer_ratio()[0] * (denominator // v.as_integer_ratio()[1])
                            for v in point))
    return tuple(points), denominator, count


def _canonical_facets(points, faces, budget, maximum):
    planes = {}
    for face in faces:
        budget.spend()
        normal = _normal(points, face)
        divisor = math.gcd(*normal)
        if not divisor:
            continue
        normal = tuple(v // divisor for v in normal)
        key = (*normal, _dot(normal, points[face[0]]))
        planes.setdefault(key, set()).update(face)
    result = []
    for plane, indices in sorted(planes.items()):
        budget.check()
        normal = plane[:3]
        axis = max(range(3), key=lambda i: abs(normal[i]))
        u, v = [i for i in range(3) if i != axis]
        ordered = sorted(indices, key=lambda i: (points[i][u], points[i][v]))
        def turn(a, b, c):
            budget.spend()
            return ((points[b][u] - points[a][u]) * (points[c][v] - points[a][v])
                    - (points[b][v] - points[a][v]) * (points[c][u] - points[a][u]))
        chains = []
        for sequence in (ordered, reversed(ordered)):
            chain = []
            for index in sequence:
                budget.spend()
                while len(chain) >= 2 and turn(chain[-2], chain[-1], index) <= 0:
                    chain.pop()
                chain.append(index)
            chains.append(chain)
        polygon = chains[0][:-1] + chains[1][:-1]
        if len(polygon) < 3:
            raise MeshGeometryError('Convex hull facet is degenerate')
        if _dot(_normal(points, polygon[:3]), normal) < 0:
            polygon.reverse()
        start = polygon.index(min(polygon))
        polygon = polygon[start:] + polygon[:start]
        for i in range(1, len(polygon) - 1):
            budget.spend()
            result.append((polygon[0], polygon[i], polygon[i + 1]))
            _face_limit(result, maximum)
    return sorted(result)


def _incremental(points, budget, maximum):
    a, b = 0, 1
    c = None
    for i in range(2, len(points)):
        budget.spend()
        if _cross(_sub(points[b], points[a]), _sub(points[i], points[a])) != (0, 0, 0):
            c = i
            break
    if c is None:
        raise MeshGeometryError('Convex hull requires noncollinear vertices')
    normal = _normal(points, (a, b, c))
    d = None
    for i in range(2, len(points)):
        budget.spend()
        if _dot(normal, _sub(points[i], points[a])):
            d = i
            break
    if d is None:
        raise MeshGeometryError('Convex hull requires noncoplanar vertices')
    initial = (a, b, c, d)
    center4 = tuple(sum(points[i][axis] for i in initial) for axis in range(3))
    faces = []
    for face in ((a, b, c), (a, d, b), (a, c, d), (b, d, c)):
        budget.spend()
        if _dot(_normal(points, face), tuple(center4[i] - 4 * points[face[0]][i] for i in range(3))) > 0:
            face = (face[0], face[2], face[1])
        faces.append(face)
    _face_limit(faces, maximum)
    for index, point in enumerate(points):
        budget.check()
        if index in initial:
            continue
        visible, retained = [], []
        for face in faces:
            budget.spend()
            (visible if _dot(_normal(points, face), _sub(point, points[face[0]])) > 0 else retained).append(face)
        if not visible:
            continue
        edges = set()
        for face in visible:
            for edge in zip(face, face[1:] + face[:1]):
                budget.spend()
                reverse = (edge[1], edge[0])
                if reverse in edges:
                    edges.remove(reverse)
                else:
                    edges.add(edge)
        for u, v in sorted(edges):
            budget.spend()
            face = (u, v, index)
            if _normal(points, face) != (0, 0, 0):
                retained.append(face)
                _face_limit(retained, maximum)
        faces = _canonical_facets(points, retained, budget, maximum)
    return _canonical_facets(points, faces, budget, maximum)


def _audit(points, faces, budget):
    edges, used, volume6 = set(), set(), 0
    origin = points[0]
    for face in faces:
        budget.check()
        a, b, c = (points[i] for i in face)
        normal = _normal(points, face)
        if normal == (0, 0, 0):
            raise MeshGeometryError('Convex hull has a zero-area face')
        for point in points:
            budget.spend()
            if _dot(normal, _sub(point, a)) > 0:
                raise MeshGeometryError('Convex hull does not contain every source vertex')
        for edge in zip(face, face[1:] + face[:1]):
            budget.spend()
            if edge in edges:
                raise MeshGeometryError('Convex hull has repeated directed edges')
            edges.add(edge)
        used.update(face)
        volume6 += _dot(_sub(a, origin), _cross(_sub(b, origin), _sub(c, origin)))
    if (not edges or any((b, a) not in edges for a, b in edges)
            or len(used) - len(edges) // 2 + len(faces) != 2 or volume6 <= 0):
        raise MeshGeometryError('Convex hull is not an outward closed manifold')
    return volume6, len(used)


def hull_stl(data, *, max_vertices=4096, max_faces=8192, max_work=2000000,
             max_triangles=10000, max_pair_checks=2000000, max_output_bytes=4194304,
             max_pieces=1024, cancelled=None, source_policy=HULL_SOURCE_POLICIES[0]):
    """Return one hull in mm, with exact containment and zero vertex rounding.

    The default requires one closed embedded consistently wound shell.
    triangle_vertices instead requires finite, nonzero-area triangles, without
    source topology verification. Both require a three-dimensional point set.
    Limits are positive bounded integers. Failures return no pieces. The hull
    intentionally fills gaps and concavities and is not decomposition.
    """
    limits = dict(max_vertices=max_vertices, max_faces=max_faces, max_work=max_work,
                  max_triangles=max_triangles, max_pair_checks=max_pair_checks,
                  max_output_bytes=max_output_bytes, max_pieces=max_pieces)
    for key, value in limits.items():
        if isinstance(value, bool) or not isinstance(value, int) or not 1 <= value <= HULL_MAXIMUMS[key]:
            raise MeshGeometryError('%s must be an integer in [1, %d]' % (key, HULL_MAXIMUMS[key]))
    if source_policy not in HULL_SOURCE_POLICIES:
        raise MeshGeometryError('source_policy must be one of ' + ', '.join(HULL_SOURCE_POLICIES))
    if not isinstance(data, (bytes, bytearray, memoryview)):
        raise MeshGeometryError('hull_stl requires binary STL bytes in millimeters')
    budget = _Budget(max_work, cancelled)
    budget.check()
    if source_policy == 'closed_shell':
        mesh = validate_mesh(_source(data, max_triangles, max_vertices, budget),
                             max_triangles=max_triangles, max_pair_checks=max_pair_checks, cancelled=cancelled)
        points, denominator = tuple(sorted(mesh.vertices)), mesh.denominator
        triangle_count, pair_checks = len(mesh.triangles), mesh.pair_checks
    else:
        points, denominator, triangle_count = _triangle_vertices(data, max_triangles, max_vertices, budget)
        pair_checks = 0
    faces = _incremental(points, budget, max_faces)
    size = 84 + 50 * len(faces)
    if size > max_output_bytes:
        raise ResourceLimitError('Convex hull exceeds max_output_bytes')
    volume6, vertex_count = _audit(points, faces, budget)
    payload = bytearray(b'exact-source-vertex-convex-hull-v1; coordinates mm'.ljust(80, b'\0'))
    payload.extend(struct.pack('<I', len(faces)))
    for face in faces:
        budget.check()
        budget.spend()
        normal = _normal(points, face)
        scale = max(abs(v) for v in normal)
        scaled = tuple(v / scale for v in normal)
        length = math.sqrt(sum(v * v for v in scaled))
        vertices = tuple(tuple(v / denominator for v in points[i]) for i in face)
        payload.extend(struct.pack('<12fH', *(v / length for v in scaled),
                                   *(v for point in vertices for v in point), 0))
    volume = Fraction(volume6, 6 * denominator**3 * 10**9)
    budget.check()
    metadata = {
        'policy': 'exact-source-vertex-convex-hull-v1', 'algorithm': 'exact-incremental-canonical-planar-facets',
        'input_length_unit': 'mm', 'piece_length_unit': 'mm', 'piece_count': 1,
        'coverage': 'convex-hull-of-all-input-float32-vertices', 'serialization_bound_m': 0.0,
        'bound_definition': 'source-float32-vertices-reemitted-exactly-no-coordinate-rounding',
        'fills_concavities': True, 'decomposition': False, 'topology_preservation': False,
        'hull_volume_m3': float(volume), 'hull_volume_m3_ratio': (volume.numerator, volume.denominator),
        'source_vertex_count': len(points), 'hull_vertex_count': vertex_count,
        'source_triangle_count': triangle_count, 'face_count': len(faces),
        'work_used': budget.used, 'validation_pair_checks': pair_checks,
        'output_bytes': len(payload), 'limits': limits,
        'shell_policy': 'single-embedded-shell', 'ordering': 'lexicographic-source-vertices-and-canonical-facets',
        'physics_policy': 'not-computed-or-modified', 'visual_policy': 'not-modified'}
    if source_policy == 'triangle_vertices':
        metadata.update(source_policy=source_policy, source_validation='finite-nonzero-area-triangles',
                        source_topology_verified=False, shell_policy='topology-not-verified',
                        fills_gaps_and_cavities=True)
    return {'pieces': [bytes(payload)], 'metadata': metadata}
