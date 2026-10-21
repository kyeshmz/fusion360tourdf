"""Opt-in conservative convex voxel cells. No visual or physical replacement."""

from fractions import Fraction
import itertools
import math
import struct

from .mesh_geometry import MeshGeometryError, ResourceLimitError, CancelledError, validate_mesh

__all__ = ['decompose_stl']


def _sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def _control(value, name, maximum):
    if isinstance(value, bool) or not isinstance(value, int) or not 1 <= value <= maximum:
        raise MeshGeometryError('%s must be an integer in [1, %d]' % (name, maximum))


class _Budget:
    def __init__(self, maximum, cancelled):
        self.maximum, self.cancelled, self.used = maximum, cancelled, 0

    def check(self):
        if self.cancelled is not None and self.cancelled():
            raise CancelledError('Collision decomposition cancelled')

    def spend(self):
        self.used += 1
        if self.used > self.maximum:
            raise ResourceLimitError('Collision decomposition exceeds max_work')
        if self.used % 128 == 0:
            self.check()


def _triangle_cell(triangle, cell, width):
    center = tuple((2 * index + 1) * width for index in cell)
    points = tuple(tuple(2 * p[i] - center[i] for i in range(3)) for p in triangle)
    edges = tuple(_sub(b, a) for a, b in zip(points, points[1:] + points[:1]))
    axes = [(1, 0, 0), (0, 1, 0), (0, 0, 1), _cross(edges[0], edges[1])]
    axes.extend(_cross(edge, axis) for edge in edges for axis in ((1, 0, 0), (0, 1, 0), (0, 0, 1)))
    for axis in axes:
        projections = [_dot(p, axis) for p in points]
        radius = width * sum(abs(v) for v in axis)
        if min(projections) > radius or max(projections) < -radius:
            return False
    return True


def _inside_by_exact_ray(point, triangles, budget):
    for k in range(1, 17):
        direction = (1, k, k * k)
        crossings, ambiguous = 0, False
        for triangle in triangles:
            budget.spend()
            a, b, c = triangle
            e1, e2, offset = _sub(b, a), _sub(c, a), _sub(point, a)
            h = _cross(direction, e2)
            determinant = _dot(e1, h)
            if not determinant:
                if _dot(_cross(e1, e2), offset) == 0:
                    ambiguous = True
                    break
                continue
            u = _dot(offset, h)
            q = _cross(offset, e1)
            v, t = _dot(direction, q), _dot(e2, q)
            if determinant < 0:
                determinant, u, v, t = -determinant, -u, -v, -t
            if t <= 0 or u < 0 or v < 0 or u + v > determinant:
                continue
            if u == 0 or v == 0 or u + v == determinant:
                ambiguous = True
                break
            crossings += 1
        if not ambiguous:
            return crossings % 2 == 1
    raise MeshGeometryError('No unambiguous exact interior ray within 16 attempts')


def _float32_outward(value, lower):
    try:
        rounded = struct.unpack('<f', struct.pack('<f', float(value)))[0]
    except (OverflowError, struct.error) as error:
        raise MeshGeometryError('Voxel boundary exceeds binary STL range') from error
    if not math.isfinite(rounded):
        raise MeshGeometryError('Voxel boundary exceeds binary STL range')
    if (lower and Fraction(rounded) > value) or (not lower and Fraction(rounded) < value):
        bits = struct.unpack('<I', struct.pack('<f', rounded))[0]
        if rounded == 0:
            bits = 0x80000001 if lower else 1
        elif (rounded > 0) == lower:
            bits -= 1
        else:
            bits += 1
        rounded = struct.unpack('<f', struct.pack('<I', bits))[0]
    if not math.isfinite(rounded):
        raise MeshGeometryError('Outward-rounded voxel boundary exceeds binary STL range')
    return rounded


def _box_stl(low, high):
    points = tuple(itertools.product(*zip(low, high)))
    faces = ((0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1),
             (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3))
    normals = ((-1, 0, 0), (1, 0, 0), (0, -1, 0), (0, 1, 0), (0, 0, -1), (0, 0, 1))
    payload = []
    for (a, b, c, d), normal in zip(faces, normals):
        for indices in ((a, b, c), (a, c, d)):
            triangle = tuple(points[i] for i in indices)
            payload.append(struct.pack('<12fH', *normal, *(v for p in triangle for v in p), 0))
    return b'conservative-voxel-cells-v1; coordinates mm'.ljust(80, b'\0') + struct.pack('<I', 12) + b''.join(payload)


def decompose_stl(data, *, resolution=16, max_pieces=32768, max_cells=32768,
                  max_work=2000000, max_triangles=10000, max_pair_checks=2000000,
                  cancelled=None):
    """Return convex binary STL cells in mm plus a conservative SI error bound.

    This is an explicit approximation policy, not a hull or exact decomposition.
    Surface intersection and interior classification use exact integer predicates.
    No result is published when a size/work limit, ambiguity, or callback fails.
    """
    for value, name, maximum in ((resolution, 'resolution', 64), (max_pieces, 'max_pieces', 262144),
                                  (max_cells, 'max_cells', 262144), (max_work, 'max_work', 10000000)):
        _control(value, name, maximum)
    if not isinstance(data, (bytes, bytearray, memoryview)):
        raise MeshGeometryError('decompose_stl requires binary STL bytes in millimeters')
    budget = _Budget(max_work, cancelled)
    budget.check()
    mesh = validate_mesh(data, max_triangles=max_triangles, max_pair_checks=max_pair_checks, cancelled=cancelled)
    low = tuple(min(p[i] for p in mesh.vertices) for i in range(3))
    extent = tuple(max(p[i] for p in mesh.vertices) - low[i] for i in range(3))
    width = max(extent)
    shape = tuple((e * resolution + width - 1) // width for e in extent)
    if math.prod(shape) > max_cells:
        raise ResourceLimitError('Collision grid exceeds max_cells')
    vertices = tuple(tuple((p[i] - low[i]) * resolution for i in range(3)) for p in mesh.vertices)
    triangles = tuple(tuple(vertices[i] for i in t) for t in mesh.triangles)
    occupied = set()
    surface_tests = 0
    for triangle in triangles:
        budget.check()
        ranges = []
        for axis in range(3):
            minimum, maximum = min(p[axis] for p in triangle), max(p[axis] for p in triangle)
            start = max(0, (minimum + width - 1) // width - 1)
            stop = min(shape[axis] - 1, maximum // width)
            ranges.append(range(start, stop + 1))
        for cell in itertools.product(*ranges):
            budget.spend()
            surface_tests += 1
            if cell not in occupied and _triangle_cell(triangle, cell, width):
                occupied.add(cell)
                if len(occupied) > max_pieces:
                    raise ResourceLimitError('Collision decomposition exceeds max_pieces')
    surface_cells = len(occupied)
    doubled_triangles = tuple(tuple(tuple(2 * x for x in p) for p in t) for t in triangles)
    for cell in itertools.product(*(range(n) for n in shape)):
        budget.spend()
        if cell in occupied:
            continue
        point = tuple((2 * i + 1) * width for i in cell)
        if _inside_by_exact_ray(point, doubled_triangles, budget):
            occupied.add(cell)
            if len(occupied) > max_pieces:
                raise ResourceLimitError('Collision decomposition exceeds max_pieces')
    pieces, rounding = [], [Fraction(0)] * 3
    for cell in sorted(occupied):
        budget.check()
        lows, highs = [], []
        for axis in range(3):
            a = Fraction(low[axis], mesh.denominator) + Fraction(cell[axis] * width, resolution * mesh.denominator)
            b = a + Fraction(width, resolution * mesh.denominator)
            lower, upper = _float32_outward(a, True), _float32_outward(b, False)
            if lower >= upper:
                raise MeshGeometryError('Voxel boundaries cannot form a nondegenerate binary STL cell')
            lows.append(lower)
            highs.append(upper)
            rounding[axis] = max(rounding[axis], a - Fraction(lower), Fraction(upper) - b)
        pieces.append(_box_stl(lows, highs))
    side_m = Fraction(width, resolution * mesh.denominator * 1000)
    rounding_m = sum(rounding) / 1000
    rounding_bound_m = math.nextafter(float(rounding_m), math.inf) if rounding_m else 0.0
    sqrt3_upper = Fraction(17320508075688774, 10 ** 16)
    bound_m = math.nextafter(float(sqrt3_upper * side_m + rounding_m), math.inf)
    budget.check()
    return {'pieces': pieces,
            'metadata': {'policy': 'conservative-voxel-cells-v1', 'approximation': True,
                         'input_length_unit': 'mm', 'piece_length_unit': 'mm',
                         'coverage': 'input-solid-subset-of-piece-union',
                         'hausdorff_bound_m': bound_m, 'cell_side_m': float(side_m),
                         'cell_side_m_ratio': (side_m.numerator, side_m.denominator),
                         'bound_definition': 'rational-upper-sqrt3-times-cell-side-plus-outward-rounding-l1',
                         'outward_rounding_l1_bound_m': rounding_bound_m,
                         'resolution': resolution, 'grid_shape': shape, 'grid_cell_count': math.prod(shape),
                         'piece_count': len(pieces), 'surface_cell_count': surface_cells,
                         'surface_candidate_tests': surface_tests, 'work_used': budget.used,
                         'validation_pair_checks': mesh.pair_checks, 'triangle_count': len(triangles),
                         'limits': {'max_pieces': max_pieces, 'max_cells': max_cells, 'max_work': max_work,
                                    'max_triangles': max_triangles, 'max_pair_checks': max_pair_checks},
                         'shell_policy': 'single-embedded-shell', 'ordering': 'lexicographic-grid-index',
                         'physics_policy': 'not-computed-or-modified', 'visual_policy': 'not-modified'}}
