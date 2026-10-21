"""Exact predicates and SI integrals for one embedded, closed triangle shell."""

from dataclasses import dataclass
from fractions import Fraction
import math
import struct

__all__ = ['MeshGeometryError', 'ResourceLimitError', 'CancelledError', 'ClosedMesh',
           'validate_mesh', 'mass_properties']


class MeshGeometryError(ValueError):
    """The geometry or its physical interpretation is invalid."""


class ResourceLimitError(MeshGeometryError):
    """An explicit work or size limit prevents a complete result."""


class CancelledError(RuntimeError):
    """The caller cancelled before a complete result was available."""


@dataclass(frozen=True)
class ClosedMesh:
    """Coordinates are exact integers divided by denominator, in input units."""

    vertices: tuple
    triangles: tuple
    denominator: int
    orientation: str
    pair_checks: int


def _limit(value, name, maximum):
    if isinstance(value, bool) or not isinstance(value, int) or not 1 <= value <= maximum:
        raise MeshGeometryError('%s must be an integer in [1, %d]' % (name, maximum))
    return value


def _cancel(cancelled):
    if cancelled is not None and cancelled():
        raise CancelledError('Mesh operation cancelled')


def _sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def _on_segment(p, a, b):
    return _cross(_sub(p, a), _sub(b, a)) == (0, 0, 0) and all(
        min(a[i], b[i]) <= p[i] <= max(a[i], b[i]) for i in range(3))


def _in_triangle(p, triangle, normal):
    return all(_dot(normal, _cross(_sub(b, a), _sub(p, a))) >= 0
               for a, b in zip(triangle, triangle[1:] + triangle[:1]))


def _intersection_points(a, b):
    na = _cross(_sub(a[1], a[0]), _sub(a[2], a[0]))
    nb = _cross(_sub(b[1], b[0]), _sub(b[2], b[0]))
    if _cross(na, nb) != (0, 0, 0):
        for first, second, normal in ((a, b, nb), (b, a, na)):
            for p, q in zip(first, first[1:] + first[:1]):
                dp, dq = _dot(normal, _sub(p, second[0])), _dot(normal, _sub(q, second[0]))
                if dp == 0 and _in_triangle(p, second, normal):
                    yield p
                if dp * dq < 0:
                    t = Fraction(dp, dp - dq)
                    hit = tuple(p[i] + t * (q[i] - p[i]) for i in range(3))
                    if _in_triangle(hit, second, normal):
                        yield hit
        return
    if _dot(na, _sub(b[0], a[0])) != 0:
        return
    for first, second, normal in ((a, b, nb), (b, a, na)):
        for p in first:
            if _in_triangle(p, second, normal):
                yield p
    axis = max(range(3), key=lambda i: abs(na[i]))
    axes = [i for i in range(3) if i != axis]
    def cross2(u, v):
        return u[axes[0]] * v[axes[1]] - u[axes[1]] * v[axes[0]]
    for p, q in zip(a, a[1:] + a[:1]):
        u = _sub(q, p)
        for r, s in zip(b, b[1:] + b[:1]):
            v, w = _sub(s, r), _sub(r, p)
            determinant = cross2(u, v)
            if determinant:
                t, z = Fraction(cross2(w, v), determinant), Fraction(cross2(w, u), determinant)
                if 0 <= t <= 1 and 0 <= z <= 1:
                    yield tuple(p[i] + t * u[i] for i in range(3))
            else:
                for point in (p, q):
                    if _on_segment(point, r, s):
                        yield point
                for point in (r, s):
                    if _on_segment(point, p, q):
                        yield point


def _source_triangles(source, max_triangles):
    if isinstance(source, (bytes, bytearray, memoryview)):
        size = memoryview(source).nbytes
        if size < 84:
            raise MeshGeometryError('Empty or truncated binary STL')
        if size > 84 + max_triangles * 50:
            raise ResourceLimitError('Binary STL exceeds max_triangles byte allowance')
        source = bytes(source)
        count = struct.unpack_from('<I', source, 80)[0]
        if not count or len(source) != 84 + count * 50:
            raise MeshGeometryError('Binary STL triangle count/length mismatch')
        if count > max_triangles:
            raise ResourceLimitError('Mesh exceeds max_triangles')
        for row in struct.iter_unpack('<12fH', source[84:]):
            if not all(math.isfinite(v) for v in row[:12]):
                raise MeshGeometryError('Nonfinite binary STL value')
            yield tuple(tuple(row[k:k + 3]) for k in (3, 6, 9))
    else:
        yield from source


def validate_mesh(source, *, max_triangles=10000, max_pair_checks=2000000, cancelled=None):
    """Reject nonembedded/nonmanifold meshes and anything but one closed shell.

    The finite limits bound stored triangles and all broad-phase triangle pairs.
    Cancellation predicates may also raise a caller-owned exception.
    """
    _limit(max_triangles, 'max_triangles', 100000)
    _limit(max_pair_checks, 'max_pair_checks', 10000000)
    _cancel(cancelled)
    points, triangles, lookup = [], [], {}
    denominator = 1
    for triangle in _source_triangles(source, max_triangles):
        _cancel(cancelled)
        if len(triangles) >= max_triangles:
            raise ResourceLimitError('Mesh exceeds max_triangles')
        if len(triangle) != 3:
            raise MeshGeometryError('A triangle requires three points')
        indices = []
        for point in triangle:
            if len(point) != 3:
                raise MeshGeometryError('A point requires three finite coordinates')
            try:
                p = tuple(float(v) for v in point)
            except (ValueError, TypeError, OverflowError) as error:
                raise MeshGeometryError('A point requires three finite coordinates') from error
            if not all(math.isfinite(v) for v in p):
                raise MeshGeometryError('Nonfinite mesh coordinate')
            if p not in lookup:
                lookup[p] = len(points)
                points.append(p)
                denominator = max(denominator, *(v.as_integer_ratio()[1] for v in p))
            indices.append(lookup[p])
        triangles.append(tuple(indices))
    if not triangles:
        raise MeshGeometryError('Mesh contains no triangles')
    vertices = tuple(tuple(v.as_integer_ratio()[0] * (denominator // v.as_integer_ratio()[1])
                           for v in p) for p in points)
    edges, links = {}, [{} for _ in vertices]
    for index, triangle in enumerate(triangles):
        a, b, c = (vertices[i] for i in triangle)
        if _cross(_sub(b, a), _sub(c, a)) == (0, 0, 0):
            raise MeshGeometryError('Degenerate mesh triangle')
        for u, v in zip(triangle, triangle[1:] + triangle[:1]):
            edges.setdefault(tuple(sorted((u, v))), []).append((u, v, index))
        for k in range(3):
            vertex, u, v = triangle[k], triangle[(k + 1) % 3], triangle[(k + 2) % 3]
            links[vertex].setdefault(u, set()).add(v)
            links[vertex].setdefault(v, set()).add(u)
    adjacency = [set() for _ in triangles]
    for uses in edges.values():
        if len(uses) != 2:
            raise MeshGeometryError('Mesh has open or nonmanifold edges')
        a, b = uses
        if a[:2] != (b[1], b[0]):
            raise MeshGeometryError('Mesh has inconsistent winding')
        adjacency[a[2]].add(b[2])
        adjacency[b[2]].add(a[2])
    def connected(graph, start):
        seen, pending = set(), [start]
        while pending:
            node = pending.pop()
            if node not in seen:
                seen.add(node)
                pending.extend(graph[node] - seen)
        return seen
    for link in links:
        if any(len(neighbors) != 2 for neighbors in link.values()) or len(connected(link, next(iter(link)))) != len(link):
            raise MeshGeometryError('Mesh has a nonmanifold vertex')
    if len(connected(adjacency, 0)) != len(triangles):
        raise MeshGeometryError('Multiple shells and cavities are not supported')
    pair_count = len(triangles) * (len(triangles) - 1) // 2
    if pair_count > max_pair_checks:
        raise ResourceLimitError('Mesh exceeds max_pair_checks for embedded-shell validation')
    geometry = [tuple(vertices[i] for i in t) for t in triangles]
    boxes = [(tuple(min(p[i] for p in t) for i in range(3)),
              tuple(max(p[i] for p in t) for i in range(3))) for t in geometry]
    count = 0
    for i, a in enumerate(geometry):
        for j in range(i):
            count += 1
            if count % 128 == 0:
                _cancel(cancelled)
            if any(boxes[i][1][k] < boxes[j][0][k] or boxes[j][1][k] < boxes[i][0][k] for k in range(3)):
                continue
            shared = set(triangles[i]) & set(triangles[j])
            allowed = tuple(vertices[k] for k in sorted(shared))
            if len(allowed) == 3:
                raise MeshGeometryError('Duplicate mesh face')
            for hit in _intersection_points(a, geometry[j]):
                if hit in allowed or (len(allowed) == 2 and _on_segment(hit, *allowed)):
                    continue
                raise MeshGeometryError('Mesh has intersecting or overlapping triangles')
    origin = vertices[0]
    volume6 = sum(_dot(_sub(a, origin), _cross(_sub(b, origin), _sub(c, origin))) for a, b, c in geometry)
    if not volume6:
        raise MeshGeometryError('Mesh has zero enclosed volume')
    _cancel(cancelled)
    return ClosedMesh(vertices, tuple(triangles), denominator,
                      'outward' if volume6 > 0 else 'inward', pair_count)


def _finite_float(value, label, positive=False):
    try:
        result = float(value)
    except (OverflowError, ValueError) as error:
        raise MeshGeometryError('%s is not representable as a finite SI float' % label) from error
    if not math.isfinite(result) or (positive and result <= 0) or (value and result == 0):
        raise MeshGeometryError('%s is not representable as a finite SI float' % label)
    return result


def mass_properties(source, *, length_unit, density_kg_m3, max_triangles=10000,
                    max_pair_checks=2000000, cancelled=None):
    """Return SI volume, mass, center, and six central inertia tensor entries.

    No density default, welding, mass replacement, tolerance, or inertia floor.
    Uniform density and a single embedded shell are mandatory.
    """
    units = {'m': Fraction(1), 'cm': Fraction(1, 100), 'mm': Fraction(1, 1000)}
    if not isinstance(length_unit, str) or length_unit not in units:
        raise MeshGeometryError('length_unit must explicitly be m, cm, or mm')
    if isinstance(density_kg_m3, bool) or not isinstance(density_kg_m3, (float, int)):
        raise MeshGeometryError('density_kg_m3 must be finite and positive')
    try:
        density_float = float(density_kg_m3)
    except OverflowError as error:
        raise MeshGeometryError('density_kg_m3 must be finite and positive') from error
    if not math.isfinite(density_float) or density_float <= 0:
        raise MeshGeometryError('density_kg_m3 must be finite and positive')
    density = Fraction(density_float)
    mesh = validate_mesh(source, max_triangles=max_triangles, max_pair_checks=max_pair_checks, cancelled=cancelled)
    origin = mesh.vertices[0]
    d, first, second = 0, [0] * 3, [[0] * 3 for _ in range(3)]
    for triangle in mesh.triangles:
        _cancel(cancelled)
        points = tuple(_sub(mesh.vertices[i], origin) for i in triangle)
        determinant = _dot(points[0], _cross(points[1], points[2]))
        d += determinant
        sums = [sum(p[i] for p in points) for i in range(3)]
        for i in range(3):
            first[i] += determinant * sums[i]
            for j in range(3):
                second[i][j] += determinant * (sums[i] * sums[j] + sum(p[i] * p[j] for p in points))
    sign = 1 if d > 0 else -1
    volume = Fraction(abs(d), 6)
    center = [Fraction(v, 4 * d) for v in first]
    covariance = [[sign * Fraction(second[i][j], 120) - volume * center[i] * center[j]
                   for j in range(3)] for i in range(3)]
    scale = units[length_unit] / mesh.denominator
    inertia_scale = density * scale ** 5
    entries = {key: _finite_float(value * inertia_scale, key, positive=i == j)
               for key, i, j, value in (
                   ('ixx', 0, 0, covariance[1][1] + covariance[2][2]),
                   ('iyy', 1, 1, covariance[0][0] + covariance[2][2]),
                   ('izz', 2, 2, covariance[0][0] + covariance[1][1]),
                   ('ixy', 0, 1, -covariance[0][1]),
                   ('ixz', 0, 2, -covariance[0][2]),
                   ('iyz', 1, 2, -covariance[1][2]))}
    return {'volume_m3': _finite_float(volume * scale ** 3, 'volume', True),
            'mass_kg': _finite_float(volume * scale ** 3 * density, 'mass', True),
            'center_of_mass_m': tuple(_finite_float((origin[i] + center[i]) * scale, 'center') for i in range(3)),
            'inertia_kg_m2': entries,
            'provenance': {'method': 'reference-relative-exact-rational-tetrahedra-v1',
                           'length_unit': length_unit, 'density_kg_m3': density_float,
                           'density_policy': 'explicit-uniform', 'shell_policy': 'single-embedded-shell',
                           'orientation': mesh.orientation, 'triangle_count': len(mesh.triangles),
                           'vertex_count': len(mesh.vertices), 'pair_checks': mesh.pair_checks}}
