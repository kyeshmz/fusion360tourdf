import math
import struct
import unittest

from URDF_Exporter.utils.mesh_geometry import (
    MeshGeometryError, ResourceLimitError, CancelledError, mass_properties, validate_mesh,
)


def cell_surface(cells):
    cells = set(cells)
    result = []
    faces = (
        ((-1, 0, 0), ((0, 0, 0), (0, 0, 1), (0, 1, 1), (0, 1, 0))),
        ((1, 0, 0), ((1, 0, 0), (1, 1, 0), (1, 1, 1), (1, 0, 1))),
        ((0, -1, 0), ((0, 0, 0), (1, 0, 0), (1, 0, 1), (0, 0, 1))),
        ((0, 1, 0), ((0, 1, 0), (0, 1, 1), (1, 1, 1), (1, 1, 0))),
        ((0, 0, -1), ((0, 0, 0), (0, 1, 0), (1, 1, 0), (1, 0, 0))),
        ((0, 0, 1), ((0, 0, 1), (1, 0, 1), (1, 1, 1), (0, 1, 1))),
    )
    for cell in sorted(cells):
        for direction, offsets in faces:
            if tuple(a + b for a, b in zip(cell, direction)) in cells:
                continue
            points = [tuple(a + b for a, b in zip(cell, offset)) for offset in offsets]
            result.extend(((points[0], points[1], points[2]), (points[0], points[2], points[3])))
    return result


def transformed(triangles, scale=(1, 1, 1), rotation=None, translation=(0, 0, 0)):
    rotation = rotation or ((1, 0, 0), (0, 1, 0), (0, 0, 1))
    return [tuple(tuple(sum(rotation[i][j] * p[j] * scale[j] for j in range(3)) + translation[i]
                        for i in range(3)) for p in triangle) for triangle in triangles]


def stl_bytes(triangles):
    return (b'independent analytic fixture'.ljust(80, b'\0') + struct.pack('<I', len(triangles))
            + b''.join(struct.pack('<12fH', 0, 0, 0, *(v for p in tri for v in p), 0)
                       for tri in triangles))


def box_oracle(dimensions, density, rotation=None, translation=(0, 0, 0)):
    rotation = rotation or ((1, 0, 0), (0, 1, 0), (0, 0, 1))
    mass = math.prod(dimensions) * density
    diagonal = [mass * sum(dimensions[j] ** 2 for j in range(3) if j != i) / 12 for i in range(3)]
    tensor = [[sum(rotation[i][k] * diagonal[k] * rotation[j][k] for k in range(3))
               for j in range(3)] for i in range(3)]
    center = [translation[i] + sum(rotation[i][j] * dimensions[j] / 2 for j in range(3))
              for i in range(3)]
    return mass, center, tensor


class MeshGeometryTests(unittest.TestCase):
    def assert_physics(self, result, mass, center, tensor, tolerance=2e-12):
        self.assertAlmostEqual(result['mass_kg'] / mass, 1, delta=tolerance)
        for a, b in zip(result['center_of_mass_m'], center):
            self.assertAlmostEqual(a, b, delta=max(math.ulp(b) * 4, abs(b) * tolerance))
        keys = (('ixx', 0, 0), ('iyy', 1, 1), ('izz', 2, 2),
                ('ixy', 0, 1), ('ixz', 0, 2), ('iyz', 1, 2))
        scale = max(tensor[i][i] for i in range(3))
        for key, i, j in keys:
            self.assertAlmostEqual(result['inertia_kg_m2'][key], tensor[i][j], delta=scale * tolerance)
        actual = result['inertia_kg_m2']
        matrix = [[actual['ixx'], actual['ixy'], actual['ixz']],
                  [actual['ixy'], actual['iyy'], actual['iyz']],
                  [actual['ixz'], actual['iyz'], actual['izz']]]
        normalized = [[v / scale for v in row] for row in matrix]
        covariance = [[(sum(normalized[k][k] for k in range(3)) / 2 if i == j else 0) - normalized[i][j]
                       for j in range(3)] for i in range(3)]
        for tested in (normalized, covariance):
            for i in range(3):
                self.assertGreater(tested[i][i], 0)
                for j in range(i):
                    self.assertGreater(tested[i][i] * tested[j][j] - tested[i][j] ** 2, 0)
            determinant = sum(tested[0][i] * (tested[1][(i + 1) % 3] * tested[2][(i + 2) % 3]
                                             - tested[1][(i + 2) % 3] * tested[2][(i + 1) % 3]) for i in range(3))
            self.assertGreater(determinant, 0)

    def test_asymmetric_box_all_units_and_binary_input(self):
        for unit, factor in (('m', 1), ('cm', 100), ('mm', 1000)):
            with self.subTest(unit=unit):
                tri = transformed(cell_surface([(0, 0, 0)]), scale=(2 * factor, 3 * factor, 5 * factor))
                expected = box_oracle((2, 3, 5), 7800)
                for source in (tri, stl_bytes(tri)):
                    result = mass_properties(source, length_unit=unit, density_kg_m3=7800)
                    self.assert_physics(result, *expected)
                    self.assertEqual(result['volume_m3'], 30)
                    self.assertEqual(result['provenance']['length_unit'], unit)
                    self.assertEqual(result['provenance']['density_kg_m3'], 7800)

    def test_rotated_translated_box_has_all_cross_terms(self):
        rotation = ((0.36, -0.48, 0.8), (0.8, 0.6, 0), (-0.48, 0.64, 0.6))
        shift = (1000000, -2000000, 3000000)
        tri = transformed(cell_surface([(0, 0, 0)]), scale=(2, 3, 5), rotation=rotation, translation=shift)
        self.assert_physics(mass_properties(tri, length_unit='m', density_kg_m3=11),
                            *box_oracle((2, 3, 5), 11, rotation, shift), tolerance=2e-9)

    def test_concave_union_independent_cuboid_parallel_axis_oracle(self):
        cells = [(0, 0, 0), (1, 0, 0), (0, 1, 0)]
        density = 7
        centers = [(x + .5, y + .5, z + .5) for x, y, z in cells]
        center = [sum(c[i] for c in centers) / 3 for i in range(3)]
        tensor = [[0.0] * 3 for _ in range(3)]
        for c in centers:
            d = [c[i] - center[i] for i in range(3)]
            for i in range(3):
                for j in range(3):
                    tensor[i][j] += density * ((1 / 6 + sum(v * v for v in d)) if i == j else 0) - density * d[i] * d[j]
        result = mass_properties(cell_surface(cells), length_unit='m', density_kg_m3=density)
        self.assert_physics(result, 21, center, tensor)
        self.assertEqual(result['volume_m3'], 3)

    def test_reversed_winding_and_triangle_permutation(self):
        tri = transformed(cell_surface([(0, 0, 0)]), scale=(2, 3, 5))
        a = mass_properties(tri, length_unit='m', density_kg_m3=1)
        b = mass_properties([tuple(reversed(t)) for t in reversed(tri)], length_unit='m', density_kg_m3=1)
        for key in ('volume_m3', 'mass_kg', 'center_of_mass_m', 'inertia_kg_m2'):
            self.assertEqual(a[key], b[key])
        self.assertEqual(b['provenance']['orientation'], 'inward')

    def test_tiny_representable_physics_is_not_floored(self):
        for size, density in ((1e-50, 1e100), (1e-70, 1e100), (1e-100, 1e200)):
            with self.subTest(size=size, density=density):
                dimensions = (size, 2 * size, 3 * size)
                tri = transformed(cell_surface([(0, 0, 0)]), scale=dimensions)
                self.assert_physics(mass_properties(tri, length_unit='m', density_kg_m3=density),
                                    *box_oracle(dimensions, density))

    def test_tetrahedron_literal_full_tensor(self):
        a, b, c, d = (0, 0, 0), (2, 0, 0), (0, 3, 0), (0, 0, 5)
        triangles = [(a, c, b), (a, b, d), (a, d, c), (b, c, d)]
        result = mass_properties(triangles, length_unit='m', density_kg_m3=4)
        expected = [[25.5, 1.5, 2.5], [1.5, 21.75, 3.75], [2.5, 3.75, 9.75]]
        self.assert_physics(result, 20, (.5, .75, 1.25), expected)
        self.assertEqual(result['volume_m3'], 5)

    def test_far_reference_small_extent_and_final_float_failure(self):
        shift = (2 ** 40, -(2 ** 41), 2 ** 42)
        tri = transformed(cell_surface([(0, 0, 0)]), scale=(2, 3, 5), translation=shift)
        self.assert_physics(mass_properties(tri, length_unit='m', density_kg_m3=3),
                            *box_oracle((2, 3, 5), 3, translation=shift))
        for size in (1e-100, 1e100):
            with self.subTest(size=size), self.assertRaises(MeshGeometryError):
                mass_properties(transformed(cell_surface([(0, 0, 0)]), scale=(size,) * 3),
                                length_unit='m', density_kg_m3=1)

    def test_explicit_density_units_and_finite_input(self):
        tri = cell_surface([(0, 0, 0)])
        with self.assertRaises(TypeError):
            mass_properties(tri)
        for density in (0, -1, math.nan, math.inf, True, 'steel'):
            with self.subTest(density=density), self.assertRaises((MeshGeometryError, TypeError)):
                mass_properties(tri, length_unit='m', density_kg_m3=density)
        with self.assertRaises(MeshGeometryError):
            mass_properties(tri, length_unit='inch', density_kg_m3=1)
        invalid = list(tri)
        invalid[0] = ((math.nan, 0, 0), *invalid[0][1:])
        with self.assertRaises(MeshGeometryError):
            validate_mesh(invalid)

    def test_open_nonmanifold_degenerate_and_inconsistent_winding(self):
        tri = cell_surface([(0, 0, 0)])
        cases = [tri[:-1], tri + [tri[0]], [((0, 0, 0), (1, 0, 0), (2, 0, 0))],
                 [tuple(reversed(tri[0])), *tri[1:]], []]
        for case in cases:
            with self.subTest(count=len(case)), self.assertRaises(MeshGeometryError):
                validate_mesh(case)

    def test_disconnected_cavities_pinches_and_self_intersections_rejected(self):
        cube = cell_surface([(0, 0, 0)])
        cases = [cube + transformed(cube, translation=(3, 0, 0)),
                 transformed(cube, scale=(4, 4, 4)) + [tuple(reversed(t)) for t in transformed(cube, translation=(1, 1, 1))],
                 cube + transformed(cube, translation=(1, 1, 1))]
        crossing = list(cube)
        crossing = [tuple(((-.5, .5, .5) if p == (1, 1, 1) else p) for p in t) for t in crossing]
        cases.append(crossing)
        for case in cases:
            with self.subTest(count=len(case)), self.assertRaises(MeshGeometryError):
                validate_mesh(case)

    def test_binary_errors_resource_and_cancellation_contract(self):
        data = stl_bytes(cell_surface([(0, 0, 0)]))
        for invalid in (b'', data[:-1], data + b'0'):
            with self.assertRaises(MeshGeometryError):
                validate_mesh(invalid)
        nonfinite_normal = bytearray(data)
        struct.pack_into('<f', nonfinite_normal, 84, math.inf)
        with self.assertRaises(MeshGeometryError):
            validate_mesh(nonfinite_normal)
        with self.assertRaises(ResourceLimitError):
            validate_mesh(data, max_triangles=1)
        with self.assertRaises(ResourceLimitError):
            validate_mesh(data, max_pair_checks=1)
        with self.assertRaises(CancelledError):
            validate_mesh(data, cancelled=lambda: True)


if __name__ == '__main__':
    unittest.main()
