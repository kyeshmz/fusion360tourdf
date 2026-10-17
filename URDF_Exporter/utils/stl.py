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
