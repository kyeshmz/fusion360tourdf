import math
from pathlib import Path
import struct


def read_binary_stl(path):
    path = Path(path)
    if not path.is_file() or path.is_symlink():
        raise ValueError('Missing mesh file: %s' % path.name)
    data = path.read_bytes()
    if len(data) < 84:
        raise ValueError('Empty or truncated binary STL: %s' % path.name)
    count = struct.unpack_from('<I', data, 80)[0]
    if count == 0 or len(data) != 84 + count * 50:
        raise ValueError('Binary STL triangle count/length mismatch: %s' % path.name)
    payload = data[84:]
    for triangle in struct.iter_unpack('<12fH', payload):
        if not all(math.isfinite(value) for value in triangle[:12]):
            raise ValueError('Nonfinite binary STL coordinates: %s' % path.name)
    return count, payload


def transform_binary_stl(payload, matrix_cm):
    """Place native-body millimeter triangles in the root assembly frame."""
    if len(matrix_cm) != 16 or not all(math.isfinite(value) for value in matrix_cm):
        raise ValueError('STL placement requires a finite 4x4 occurrence transform')
    rows = [matrix_cm[4*i:4*i+3] for i in range(3)]
    if any(abs(sum(a*b for a, b in zip(rows[i], rows[j])) - (1 if i == j else 0)) > 1e-6
           for i in range(3) for j in range(3)) or any(
               abs(a-b) > 1e-6 for a, b in zip(matrix_cm[12:], (0, 0, 0, 1))):
        raise ValueError('STL placement requires a rigid occurrence transform')
    determinant = (rows[0][0]*(rows[1][1]*rows[2][2]-rows[1][2]*rows[2][1])
                   - rows[0][1]*(rows[1][0]*rows[2][2]-rows[1][2]*rows[2][0])
                   + rows[0][2]*(rows[1][0]*rows[2][1]-rows[1][1]*rows[2][0]))
    if abs(determinant - 1) > 1e-6:
        raise ValueError('STL placement requires a proper rotation')
    translation_mm = [matrix_cm[4*i+3] * 10 for i in range(3)]
    result = bytearray()
    for triangle in struct.iter_unpack('<12fH', payload):
        values = []
        for offset in (0, 3, 6, 9):
            point = triangle[offset:offset+3]
            values.extend(sum(a*b for a, b in zip(row, point)) + (translation_mm[i] if offset else 0)
                          for i, row in enumerate(rows))
        if not all(math.isfinite(value) for value in values):
            raise ValueError('Nonfinite transformed STL coordinates')
        result.extend(struct.pack('<12fH', *values, triangle[12]))
    return result
