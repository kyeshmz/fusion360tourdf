"""Read-only face snapshots and portable diffuse OBJ assets, without adsk imports.

This is an opt-in backend, not a package writer. Unsupported native shaders
raise VisualAssetError. The legacy single-RGBA/STL path is unchanged.
"""

import base64
import hashlib
import inspect
import json
import math
import os
import re
import stat
import struct
import zlib
from pathlib import PurePosixPath


__all__ = ['VisualAssetError', 'read_local_image', 'read_native_body', 'build_obj_assets',
           'validate_asset_bundles', 'visual_descriptors']
_MISSING = object()
_DIFFUSE_IDS = frozenset(('opaque_albedo', 'generic_diffuse'))
_CONTEXT = re.compile(r'[A-Za-z0-9_.:-]+\Z')
_DIGEST = re.compile(r'[0-9a-f]{64}\Z')
_SCHEMA = 'fusion-visual-snapshot/1'
_MAX_IMAGE_BYTES = 16 * 1024 * 1024
_PROPERTY_ID = re.compile(r'[A-Za-z0-9_.:-]{1,256}\Z')
_PROPERTY_TYPE = re.compile(r'adsk::core::[A-Za-z][A-Za-z0-9]{0,127}\Z')
_SUBSET_WARNING = 'lossy-diffuse-subset-not-shader-equivalent'
_UV_WARNING = 'provided-uvs-not-native-render-proof'


class VisualAssetError(ValueError):
    """An explicit preservation failure with a stable, source-path-free code."""

    def __init__(self, code):
        self.code = code
        super().__init__(code)


def _get(source, name, default=_MISSING):
    try:
        return getattr(source, name)
    except AttributeError:
        if default is not _MISSING and inspect.getattr_static(source, name, _MISSING) is _MISSING:
            return default
        raise VisualAssetError('unreadable-native-' + name) from None
    except Exception:
        raise VisualAssetError('unreadable-native-' + name) from None


def _items(collection):
    try:
        return [collection.item(i) for i in range(collection.count)]
    except Exception:
        raise VisualAssetError('unreadable-native-collection') from None


def _array(source, name):
    try:
        return list(_get(source, name))
    except VisualAssetError:
        raise
    except Exception:
        raise VisualAssetError('unreadable-native-' + name) from None


def _number(value):
    try:
        valid = not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)
    except OverflowError:
        valid = False
    if not valid:
        raise VisualAssetError('nonfinite-or-nonnumeric-value')
    return float(value)


def _vector(value, size):
    try:
        result = [_number(v) for v in value]
    except TypeError:
        raise VisualAssetError('invalid-vector') from None
    if len(result) != size:
        raise VisualAssetError('invalid-vector-size')
    return result


def _unit(value):
    result = _vector(value, 3)
    length = math.hypot(*result)
    if length == 0 or not math.isfinite(length):
        raise VisualAssetError('invalid-normal')
    return [v / length for v in result]


def _alpha(value):
    value = _number(value)
    if not 0 <= value <= 1:
        raise VisualAssetError('invalid-opacity')
    return value


def _rgba(color):
    if _get(color, 'objectType') != 'adsk::core::Color':
        raise VisualAssetError('unsupported-native-color')
    channels = [_number(_get(color, key)) for key in ('red', 'green', 'blue', 'opacity')]
    if any(not 0 <= channel <= 255 for channel in channels):
        raise VisualAssetError('invalid-native-color')
    return [channel / 255 for channel in channels]


def _validate_png(data):
    if not isinstance(data, bytes) or not 0 < len(data) <= _MAX_IMAGE_BYTES:
        raise VisualAssetError('invalid-image-size')
    if data[:8] != b'\x89PNG\r\n\x1a\n':
        raise VisualAssetError('unsupported-image-format')
    offset, seen, previous, width, height, row_size = 8, set(), None, 0, 0, 0
    compressed, palette_size = bytearray(), 0
    while offset < len(data):
        if offset + 12 > len(data):
            raise VisualAssetError('invalid-png')
        length = int.from_bytes(data[offset:offset + 4], 'big')
        kind = data[offset + 4:offset + 8]
        end = offset + 12 + length
        if end > len(data):
            raise VisualAssetError('invalid-png')
        payload = data[offset + 8:end - 4]
        crc = int.from_bytes(data[end - 4:end], 'big')
        if zlib.crc32(kind + payload) != crc:
            raise VisualAssetError('invalid-png-crc')
        if not seen:
            if kind != b'IHDR' or length != 13:
                raise VisualAssetError('invalid-png-header')
            width, height, depth, color, compression, filtering, interlace = struct.unpack('>IIBBBBB', payload)
            depths = {0: (1, 2, 4, 8, 16), 2: (8, 16), 3: (1, 2, 4, 8), 4: (8, 16), 6: (8, 16)}
            if (not 0 < width <= 16384 or not 0 < height <= 16384 or width * height > 64 * 1024 * 1024
                    or depth not in depths.get(color, ()) or compression or filtering or interlace != 0):
                raise VisualAssetError('unsupported-png-header')
            channels = {0: 1, 2: 3, 3: 1, 4: 2, 6: 4}[color]
            row_size = (width * channels * depth + 7) // 8 + 1
            if height * row_size > 64 * 1024 * 1024:
                raise VisualAssetError('decoded-image-too-large')
        elif kind == b'IHDR':
            raise VisualAssetError('invalid-png-header')
        elif kind == b'PLTE':
            if b'IDAT' in seen or b'PLTE' in seen or color in (0, 4) or not 0 < length <= 768 or length % 3:
                raise VisualAssetError('invalid-png-palette')
            palette_size = length // 3
            if color == 3 and palette_size > 2 ** depth:
                raise VisualAssetError('invalid-png-palette')
        elif kind == b'IDAT':
            if (b'IDAT' in seen and previous != b'IDAT') or (color == 3 and not palette_size):
                raise VisualAssetError('invalid-png-image-data')
            compressed.extend(payload)
        elif kind == b'tRNS':
            if (b'tRNS' in seen or b'IDAT' in seen or color in (4, 6)
                    or (color == 0 and length != 2) or (color == 2 and length != 6)
                    or (color == 3 and not 0 < length <= palette_size)):
                raise VisualAssetError('invalid-png-transparency')
        elif kind in (b'acTL', b'fcTL', b'fdAT') or (kind != b'IEND' and not kind[0] & 32):
            raise VisualAssetError('unsupported-png-chunk')
        if kind in (b'IHDR', b'PLTE', b'IDAT', b'tRNS'):
            seen.add(kind)
        previous = kind
        offset = end
        if kind == b'IEND':
            if length or offset != len(data) or b'IDAT' not in seen:
                raise VisualAssetError('invalid-png-end')
            try:
                decoder = zlib.decompressobj()
                raw = decoder.decompress(compressed, height * row_size + 1)
                if (len(raw) != height * row_size or not decoder.eof or decoder.unused_data
                        or decoder.unconsumed_tail or any(raw[row * row_size] > 4 for row in range(height))):
                    raise VisualAssetError('invalid-png-pixels')
            except zlib.error:
                raise VisualAssetError('invalid-png-compression') from None
            return
    raise VisualAssetError('invalid-png-end')


def _jpeg_tables(payload, marker, tables):
    offset = 0
    if not payload:
        raise VisualAssetError('invalid-jpeg-table')
    while offset < len(payload):
        kind, identifier = payload[offset] >> 4, payload[offset] & 15
        offset += 1
        if identifier > 3 or kind > (1 if marker == 0xc4 else 0):
            raise VisualAssetError('unsupported-jpeg-table')
        if marker == 0xdb:
            values = payload[offset:offset+64]
            if len(values) != 64 or 0 in values:
                raise VisualAssetError('invalid-jpeg-quantization')
            offset += 64
        else:
            counts = payload[offset:offset+16]
            if len(counts) != 16 or not 0 < sum(counts) <= 256:
                raise VisualAssetError('invalid-jpeg-huffman-table')
            space = 1
            for count in counts:
                space = 2 * space - count
                if space <= 0:
                    raise VisualAssetError('invalid-jpeg-huffman-codes')
            offset += 16
            symbols = payload[offset:offset+sum(counts)]
            if (len(symbols) != sum(counts) or len(set(symbols)) != len(symbols)
                    or (kind == 0 and any(value > 11 for value in symbols))
                    or (kind == 1 and any(value & 15 > 10 for value in symbols))):
                raise VisualAssetError('invalid-jpeg-huffman-symbols')
            offset += len(symbols)
        tables.add((kind, identifier))


def _jpeg_entropy_end(data, offset, restart_interval):
    count, restart = 0, 0
    while offset < len(data):
        if data[offset] != 255:
            count += 1
            offset += 1
            continue
        start = offset
        while offset < len(data) and data[offset] == 255:
            offset += 1
        if offset == len(data):
            break
        marker = data[offset]
        if marker == 0:
            if offset != start + 1:
                raise VisualAssetError('invalid-jpeg-byte-stuffing')
            count += 1
            offset += 1
        elif 0xd0 <= marker <= 0xd7:
            if not restart_interval or not count or marker != 0xd0 + restart:
                raise VisualAssetError('invalid-jpeg-restart')
            restart = (restart + 1) % 8
            count = 0
            offset += 1
        else:
            if not count:
                raise VisualAssetError('empty-jpeg-scan')
            return start
    raise VisualAssetError('truncated-jpeg-scan')


def _validate_jpeg(data):
    """Check bounded JPEG structure, not Huffman entropy or decoded pixels."""
    if not isinstance(data, bytes) or not 0 < len(data) <= _MAX_IMAGE_BYTES:
        raise VisualAssetError('invalid-image-size')
    if data[:2] != b'\xff\xd8':
        raise VisualAssetError('unsupported-image-format')
    offset, frame, components, scans, restart_interval = 2, None, {}, 0, 0
    quantization, huffman, coefficients = set(), set(), {}
    while offset < len(data):
        if data[offset] != 255:
            raise VisualAssetError('invalid-jpeg-marker')
        while offset < len(data) and data[offset] == 255:
            offset += 1
        if offset >= len(data):
            break
        marker = data[offset]
        offset += 1
        if marker == 0xd9:
            if (offset != len(data) or not scans or not components
                    or any(any(value is None for value in values) for values in coefficients.values())):
                raise VisualAssetError('incomplete-jpeg')
            return
        if offset + 2 > len(data):
            raise VisualAssetError('truncated-jpeg-segment')
        length = int.from_bytes(data[offset:offset+2], 'big')
        if length < 2 or offset + length > len(data):
            raise VisualAssetError('invalid-jpeg-segment')
        payload = data[offset+2:offset+length]
        offset += length
        if marker in (0xdb, 0xc4):
            _jpeg_tables(payload, marker, quantization if marker == 0xdb else huffman)
        elif marker in (0xc0, 0xc2):
            if frame is not None or len(payload) < 6:
                raise VisualAssetError('invalid-jpeg-frame')
            precision, height, width, count = struct.unpack('>BHHB', payload[:6])
            if (precision != 8 or count not in (1, 3) or len(payload) != 6 + 3 * count
                    or not 0 < width <= 16384 or not 0 < height <= 16384
                    or width * height * count > 64 * 1024 * 1024):
                raise VisualAssetError('unsupported-jpeg-frame')
            sampling = 0
            for index in range(count):
                identifier, factors, table = payload[6+3*index:9+3*index]
                horizontal, vertical = factors >> 4, factors & 15
                if (identifier in components or not 1 <= horizontal <= 4 or not 1 <= vertical <= 4
                        or table > 3):
                    raise VisualAssetError('invalid-jpeg-component')
                components[identifier] = table
                coefficients[identifier] = [None] * 64
                sampling += horizontal * vertical
            if sampling > 10:
                raise VisualAssetError('unsupported-jpeg-sampling')
            frame = marker
        elif marker == 0xdd:
            if len(payload) != 2:
                raise VisualAssetError('invalid-jpeg-restart-interval')
            restart_interval = int.from_bytes(payload, 'big')
        elif marker == 0xda:
            if frame is None or not payload or not 1 <= payload[0] <= len(components):
                raise VisualAssetError('invalid-jpeg-scan')
            count = payload[0]
            if len(payload) != 4 + 2 * count:
                raise VisualAssetError('invalid-jpeg-scan')
            first, last, approximation = payload[-3:]
            high, low = approximation >> 4, approximation & 15
            if (frame == 0xc0 and (first, last, high, low) != (0, 63, 0, 0)
                    or frame == 0xc2 and (not 0 <= first <= last <= 63 or (first == 0 and last != 0)
                                             or (first > 0 and count != 1) or low > 13
                                             or (high != 0 and high != low + 1))):
                raise VisualAssetError('unsupported-jpeg-scan')
            selected = set()
            for index in range(count):
                identifier, tables = payload[1+2*index:3+2*index]
                dc, ac = tables >> 4, tables & 15
                if (identifier not in components or identifier in selected or dc > 3 or ac > 3
                        or (0, components[identifier]) not in quantization
                        or ((frame == 0xc0 or (first == 0 and high == 0)) and (0, dc) not in huffman)
                        or ((frame == 0xc0 or first > 0) and (1, ac) not in huffman)
                        or (frame == 0xc2 and ((first == 0 and ac != 0) or (first > 0 and dc != 0)))):
                    raise VisualAssetError('invalid-jpeg-scan-tables')
                selected.add(identifier)
                values = coefficients[identifier]
                if first > 0 and values[0] is None:
                    raise VisualAssetError('invalid-jpeg-scan-order')
                for coefficient in range(first, last+1):
                    if values[coefficient] != (None if high == 0 else high):
                        raise VisualAssetError('invalid-jpeg-scan-order')
                    values[coefficient] = low
            scans += 1
            if scans > 1024:
                raise VisualAssetError('too-many-jpeg-scans')
            offset = _jpeg_entropy_end(data, offset, restart_interval)
        elif not (0xe0 <= marker <= 0xef or marker == 0xfe):
            raise VisualAssetError('unsupported-jpeg-marker')
    raise VisualAssetError('truncated-jpeg')


def _image_format(data):
    if isinstance(data, bytes) and data.startswith(b'\xff\xd8'):
        _validate_jpeg(data)
        return 'jpeg'
    _validate_png(data)
    return 'png'


def read_local_image(path, *, max_bytes=_MAX_IMAGE_BYTES):
    """Read only this selected regular local PNG or JPEG. No search or fetching.

    PNG framing, CRCs, dimensions and bounded scanline decompression are checked.
    JPEG checks structure only, not entropy decoding. Pixels remain loader gates.
    """
    if isinstance(max_bytes, bool) or not isinstance(max_bytes, int) or not 0 < max_bytes <= _MAX_IMAGE_BYTES:
        raise VisualAssetError('invalid-image-bound')
    try:
        name = os.fspath(path)
        if (not isinstance(name, str) or not name or '\x00' in name or '://' in name
                or name.startswith(('//', '\\\\'))):
            raise VisualAssetError('nonlocal-image-source')
        flags = os.O_RDONLY | getattr(os, 'O_NOFOLLOW', 0) | getattr(os, 'O_NONBLOCK', 0)
        if os.path.islink(name):
            raise VisualAssetError('symlink-image-source')
        fd = os.open(name, flags)
        with os.fdopen(fd, 'rb') as stream:
            info = os.fstat(stream.fileno())
            if not stat.S_ISREG(info.st_mode) or not 0 < info.st_size <= max_bytes:
                raise VisualAssetError('invalid-image-size-or-type')
            data = stream.read(max_bytes + 1)
        if not 0 < len(data) <= max_bytes:
            raise VisualAssetError('invalid-image-size')
    except VisualAssetError:
        raise
    except (OSError, TypeError, ValueError):
        raise VisualAssetError('unreadable-image-source') from None
    image_format = _image_format(data)
    suffix = os.path.splitext(name)[1].lower()
    expected = {'.png': 'png', '.jpg': 'jpeg', '.jpeg': 'jpeg', '.jpe': 'jpeg'}
    if ((suffix in expected and expected[suffix] != image_format)
            or suffix in ('.gif', '.bmp', '.tif', '.tiff', '.webp', '.avif', '.apng')):
        raise VisualAssetError('conflicting-image-format')
    return {'format': image_format, 'data': data}


def _property_identity(identifier, object_type):
    if (not isinstance(identifier, str) or not _PROPERTY_ID.fullmatch(identifier)
            or identifier in ('.', '..') or not isinstance(object_type, str)
            or not _PROPERTY_TYPE.fullmatch(object_type)):
        raise VisualAssetError('nonportable-property-identity')
    return {'id': identifier, 'type': object_type}


def _native_material(appearance, opacity, explicit_source, shader_policy, diffuse_id, source_kind):
    if appearance is None:
        raise VisualAssetError('missing-appearance')
    properties = _items(_get(appearance, 'appearanceProperties'))
    if shader_policy == 'strict' and len(properties) != 1:
        raise VisualAssetError('unsupported-or-ambiguous-shader')
    identities = [_property_identity(_get(p, 'id'), _get(p, 'objectType')) for p in properties]
    candidates = [index for index, identity in enumerate(identities) if identity['id'] in _DIFFUSE_IDS
                  and identity['type'] == 'adsk::core::ColorProperty'
                  and (diffuse_id is None or identity['id'] == diffuse_id)]
    if len(candidates) != 1:
        raise VisualAssetError('unsupported-or-ambiguous-diffuse-shader')
    selected = candidates[0]
    if sum(identity['id'] == identities[selected]['id'] for identity in identities) != 1:
        raise VisualAssetError('unsupported-or-ambiguous-diffuse-shader')
    prop = properties[selected]
    if _get(prop, 'hasMultipleValues') is not False:
        raise VisualAssetError('unsupported-shader-property')
    receipt = {'shader_policy': shader_policy, 'source': source_kind,
               'selected_property': identities[selected],
               'losses': [dict(identity, reason='omitted-shader-property')
                          for index, identity in enumerate(identities) if index != selected],
               'warnings': [_SUBSET_WARNING] if shader_policy == 'diffuse-subset' else []}
    connected = _get(prop, 'hasConnectedTexture')
    if connected is False:
        if explicit_source is not None:
            raise VisualAssetError('texture-source-without-texture')
        rgba = _rgba(_get(prop, 'value'))
        rgba[3] *= opacity
        return {'rgba': rgba, 'image': None}, None, receipt
    if connected is not True:
        raise VisualAssetError('unreadable-texture-state')
    texture = _get(prop, 'connectedTexture')
    if _get(texture, 'textureType') != 1:
        raise VisualAssetError('unsupported-procedural-texture')
    texture_properties = _items(_get(texture, 'properties'))
    if any(_get(item, 'objectType') != 'adsk::core::FilenameProperty' for item in texture_properties):
        raise VisualAssetError('unsupported-image-texture-properties')
    if explicit_source is None:
        filenames = texture_properties
        if len(filenames) != 1:
            raise VisualAssetError('missing-or-ambiguous-image-source')
        filename = filenames[0]
        if (_get(filename, 'hasMultipleValues') is not False or not isinstance(_get(filename, 'id'), str)
                or not _get(filename, 'id')):
            raise VisualAssetError('unsupported-image-source-property')
        source = _get(filename, 'value')
        if not isinstance(source, str) or not source:
            raise VisualAssetError('missing-image-source')
    else:
        source = explicit_source
    image = read_local_image(source)
    digest = hashlib.sha256(image['data']).hexdigest()
    receipt['warnings'].append(_UV_WARNING)
    return {'rgba': [1.0, 1.0, 1.0, opacity], 'image': digest}, (digest, image), receipt


def _transform(matrix):
    m = _vector(matrix, 16)
    if m[12:] != [0.0, 0.0, 0.0, 1.0]:
        raise VisualAssetError('nonaffine-transform')
    a, b, c, d, e, f, g, h, i = (m[j] for j in (0, 1, 2, 4, 5, 6, 8, 9, 10))
    cofactors = [e*i-f*h, f*g-d*i, d*h-e*g, c*h-b*i, a*i-c*g, b*g-a*h,
                 b*f-c*e, c*d-a*f, a*e-b*d]
    determinant = a*cofactors[0] + b*cofactors[1] + c*cofactors[2]
    if not math.isfinite(determinant) or abs(determinant) < 1e-15:
        raise VisualAssetError('singular-transform')
    return m, [v / determinant for v in cofactors], determinant < 0


def read_native_body(body, *, root_from_body_cm, context_id, owner=None, quality=None,
                     texture_sources=None, shader_policy='strict', diffuse_properties=None):
    """Snapshot one selected body context, with an explicit native-to-root pose.

    Faces supply current appearance in the selected context. Tessellation is
    read from each native face, never its posed proxy. visibleOpacity is read
    from the selected body context, already including inherited overrides.
    texture_sources maps face ordinals to explicitly selected local image paths.
    diffuse-subset selects a known diffuse channel and records omitted properties.
    diffuse_properties maps face ordinals to explicit known diffuse property IDs.
    quality, when supplied, is an SDK TriangleMeshQualityOptions enum value.
    """
    if not isinstance(context_id, str) or not _CONTEXT.fullmatch(context_id) or context_id in ('.', '..'):
        raise VisualAssetError('invalid-context-id')
    if shader_policy not in ('strict', 'diffuse-subset'):
        raise VisualAssetError('invalid-shader-policy')
    if diffuse_properties is not None and shader_policy != 'diffuse-subset':
        raise VisualAssetError('diffuse-selection-requires-subset-policy')
    matrix, normal_matrix, mirrored = _transform(root_from_body_cm)
    override = _get(owner, 'appearance') if owner is not None else None
    opacity = _alpha(_get(body, 'visibleOpacity'))
    faces = _items(_get(body, 'faces'))
    try:
        sources = {} if texture_sources is None else dict(texture_sources)
        selectors = {} if diffuse_properties is None else dict(diffuse_properties)
    except (TypeError, ValueError):
        raise VisualAssetError('invalid-explicit-texture-sources') from None
    if any(type(index) is not int or not 0 <= index < len(faces) for index in sources):
        raise VisualAssetError('unselected-texture-source')
    if any(type(index) is not int or not 0 <= index < len(faces) or not isinstance(identifier, str)
           or identifier not in _DIFFUSE_IDS
           for index, identifier in selectors.items()):
        raise VisualAssetError('invalid-diffuse-selection')
    surfaces, images = [], {}
    for ordinal, face in enumerate(faces):
        appearance = override
        source_kind = 'owner'
        if appearance is None:
            appearance = _get(face, 'appearance')
            source_kind = 'face'
        if appearance is None:
            appearance = _get(body, 'appearance')
            source_kind = 'body'
        material, image, receipt = _native_material(appearance, opacity, sources.get(ordinal),
                                                   shader_policy, selectors.get(ordinal), source_kind)
        if image is not None:
            if image[0] in images and images[image[0]] != image[1]:
                raise VisualAssetError('image-hash-collision')
            images[image[0]] = image[1]
        native_face = _get(face, 'nativeObject', None)
        if native_face is None:
            if _get(face, 'assemblyContext', None) is not None:
                raise VisualAssetError('missing-native-face')
            native_face = face
        if _get(native_face, 'assemblyContext', None) is not None:
            raise VisualAssetError('unresolved-native-face')
        try:
            calculator = _get(native_face, 'meshManager').createMeshCalculator()
            if quality is not None and calculator.setQuality(quality) is not True:
                raise VisualAssetError('mesh-quality-failed')
            mesh = calculator.calculate()
        except VisualAssetError:
            raise
        except Exception:
            raise VisualAssetError('native-tessellation-failed') from None
        vertices, normals = [], []
        for node in _array(mesh, 'nodeCoordinates'):
            xyz = [_number(_get(node, axis)) for axis in ('x', 'y', 'z')]
            vertices.append([10 * (sum(matrix[4*r+k] * xyz[k] for k in range(3)) + matrix[4*r+3])
                             for r in range(3)])
        for normal in _array(mesh, 'normalVectors'):
            xyz = [_number(_get(normal, axis)) for axis in ('x', 'y', 'z')]
            normals.append(_unit([sum(normal_matrix[3*r+k] * xyz[k] for k in range(3)) for r in range(3)]))
        indices = _array(mesh, 'nodeIndices')
        triangle_count = _get(mesh, 'triangleCount')
        if type(triangle_count) is not int or len(indices) % 3 or len(indices) != 3 * triangle_count:
            raise VisualAssetError('invalid-triangle-count')
        triangles = [indices[i:i+3] for i in range(0, len(indices), 3)]
        if mirrored:
            triangles = [[t[0], t[2], t[1]] for t in triangles]
        uvs = [[_number(_get(uv, axis)) for axis in ('x', 'y')]
               for uv in _array(mesh, 'textureCoordinates')] if material['image'] else []
        surfaces.append({'context_id': context_id, 'face_index': ordinal, 'vertices_mm': vertices,
                         'normals': normals, 'uvs': uvs, 'triangles': triangles, 'material': material,
                         'appearance': receipt})
    snapshot = {'descriptor': {'schema': _SCHEMA, 'frame': 'root', 'units': 'mm', 'surfaces': surfaces},
                'images': images}
    _validated_snapshots([snapshot])
    return snapshot


def _validated_receipt(receipt, textured):
    if receipt is None:
        return None
    if (not isinstance(receipt, dict)
            or set(receipt) != {'shader_policy', 'source', 'selected_property', 'losses', 'warnings'}
            or receipt['shader_policy'] not in ('strict', 'diffuse-subset')
            or receipt['source'] not in ('owner', 'face', 'body')
            or not isinstance(receipt['losses'], list)):
        raise VisualAssetError('invalid-appearance-receipt')
    selected = receipt['selected_property']
    if (not isinstance(selected, dict) or set(selected) != {'id', 'type'}
            or selected['id'] not in _DIFFUSE_IDS or selected['type'] != 'adsk::core::ColorProperty'):
        raise VisualAssetError('invalid-selected-property-receipt')
    losses = []
    for loss in receipt['losses']:
        if (not isinstance(loss, dict) or set(loss) != {'id', 'type', 'reason'}
                or loss['reason'] != 'omitted-shader-property'):
            raise VisualAssetError('invalid-loss-receipt')
        losses.append(dict(_property_identity(loss['id'], loss['type']), reason=loss['reason']))
    warnings = [_SUBSET_WARNING] if receipt['shader_policy'] == 'diffuse-subset' else []
    if textured:
        warnings.append(_UV_WARNING)
    if receipt['warnings'] != warnings or (receipt['shader_policy'] == 'strict' and losses):
        raise VisualAssetError('invalid-policy-warning-receipt')
    return {'shader_policy': receipt['shader_policy'], 'source': receipt['source'],
            'selected_property': dict(selected), 'losses': losses, 'warnings': warnings}


def _validated_snapshots(snapshots):
    surfaces, images, identities = [], {}, set()
    for snapshot in snapshots:
        descriptor = snapshot['descriptor']
        if (descriptor.get('schema') != _SCHEMA or descriptor.get('frame') != 'root'
                or descriptor.get('units') != 'mm'):
            raise VisualAssetError('invalid-snapshot-coordinate-contract')
        for digest, image in snapshot['images'].items():
            if (not isinstance(digest, str) or not _DIGEST.fullmatch(digest) or not isinstance(image, dict)
                    or set(image) != {'format', 'data'} or image.get('format') not in ('png', 'jpeg')):
                raise VisualAssetError('invalid-image-descriptor')
            data = image['data']
            if _image_format(data) != image['format']:
                raise VisualAssetError('conflicting-image-format')
            if hashlib.sha256(data).hexdigest() != digest:
                raise VisualAssetError('image-content-hash-mismatch')
            if digest in images and images[digest] != image:
                raise VisualAssetError('image-hash-collision')
            images[digest] = {'format': image['format'], 'data': data}
        for source in descriptor['surfaces']:
            context, face = source['context_id'], source['face_index']
            if (not isinstance(context, str) or not _CONTEXT.fullmatch(context) or context in ('.', '..')
                    or type(face) is not int or face < 0):
                raise VisualAssetError('invalid-surface-provenance')
            if (context, face) in identities:
                raise VisualAssetError('duplicate-surface-context')
            identities.add((context, face))
            vertices = [_vector(v, 3) for v in source['vertices_mm']]
            normals = [_unit(v) for v in source['normals']]
            uvs = [_vector(v, 2) for v in source['uvs']]
            if not vertices or len(normals) != len(vertices) or (uvs and len(uvs) != len(vertices)):
                raise VisualAssetError('invalid-vertex-attribute-count')
            triangles = []
            for triangle in source['triangles']:
                if (len(triangle) != 3 or any(type(i) is not int or not 0 <= i < len(vertices) for i in triangle)
                        or len(set(triangle)) != 3):
                    raise VisualAssetError('invalid-triangle-index')
                a, b, c = [vertices[i] for i in triangle]
                ab, ac = [b[i]-a[i] for i in range(3)], [c[i]-a[i] for i in range(3)]
                area = [ab[1]*ac[2]-ab[2]*ac[1], ab[2]*ac[0]-ab[0]*ac[2], ab[0]*ac[1]-ab[1]*ac[0]]
                if not math.isfinite(math.hypot(*area)) or math.hypot(*area) == 0:
                    raise VisualAssetError('degenerate-triangle')
                triangles.append(list(triangle))
            if not triangles:
                raise VisualAssetError('empty-face-mesh')
            rgba = _vector(source['material']['rgba'], 4)
            if any(not 0 <= channel <= 1 for channel in rgba):
                raise VisualAssetError('invalid-rgba')
            image = source['material']['image']
            if image is not None and (image not in snapshot['images'] or not uvs):
                raise VisualAssetError('missing-image-or-uvs')
            surfaces.append(dict(context_id=context, face_index=face, vertices_mm=vertices, normals=normals,
                                 uvs=uvs, triangles=triangles, material={'rgba': rgba, 'image': image},
                                 appearance=_validated_receipt(source.get('appearance'), image is not None)))
    if not surfaces:
        raise VisualAssetError('empty-visual')
    referenced = {s['material']['image'] for s in surfaces} - {None}
    if set(images) != referenced:
        raise VisualAssetError('unselected-image-data')
    return surfaces, images


def _image_path(digest, image):
    return 'textures/' + digest + ('.jpg' if image['format'] == 'jpeg' else '.png')


def _text_vector(vector):
    return ' '.join('0' if v == 0 else format(v, '.17g') for v in vector)


def _material_name(digest):
    return 'mat_' + base64.b32encode(bytes.fromhex(digest)).decode('ascii').lower().rstrip('=')


def build_obj_assets(snapshots, *, body_context=None):
    """Return descriptor plus package-relative file bytes. Does not write files.

    Accept one snapshot or a sequence, including multiple selected body contexts
    belonging to one physical link. The descriptor is JSON-serializable. Only
    the separate files mapping contains bytes. Input ordering is preserved.
    """
    if isinstance(snapshots, dict):
        snapshots = [snapshots]
    try:
        surfaces, images = _validated_snapshots(snapshots)
    except VisualAssetError:
        raise
    except (TypeError, KeyError, ValueError, OverflowError):
        raise VisualAssetError('malformed-snapshot') from None
    materials = {}
    for surface in surfaces:
        material = surface['material']
        key = hashlib.sha256(json.dumps(material, sort_keys=True, allow_nan=False,
                                        separators=(',', ':')).encode()).hexdigest()
        if key in materials and materials[key] != material:
            raise VisualAssetError('material-hash-collision')
        materials[key] = material
        surface['material_key'] = key
    mtl = []
    for key, material in sorted(materials.items()):
        rgba = material['rgba']
        mtl.extend(['newmtl ' + _material_name(key), 'Kd ' + _text_vector(rgba[:3]),
                    'd ' + _text_vector(rgba[3:]), 'illum 1'])
        if material['image'] is not None:
            mtl.append('map_Kd ' + _image_path(material['image'], images[material['image']]))
        mtl.append('')
    mtl_data = ('\n'.join(mtl) + '\n').encode('ascii')
    mtl_name = 'visual_' + hashlib.sha256(mtl_data).hexdigest() + '.mtl'
    obj, provenance = ['mtllib ' + mtl_name], []
    if body_context is not None:
        if (type(body_context) is not dict or set(body_context) != {'id', 'source_path', 'selected_body_count'}
                or type(body_context['id']) is not str or not _CONTEXT.fullmatch(body_context['id'])
                or type(body_context['source_path']) is not str or not body_context['source_path']
                or type(body_context['selected_body_count']) is not int or body_context['selected_body_count'] <= 0
                or {s['context_id'] for s in surfaces} != {body_context['id']}):
            raise VisualAssetError('invalid-or-unselected-body-context')
        obj.append('o ' + body_context['id'])
    vertex_offset, uv_offset, normal_offset, triangle_offset = 1, 1, 1, 0
    for surface in surfaces:
        obj.extend('v ' + _text_vector(v) for v in surface['vertices_mm'])
        obj.extend('vt ' + _text_vector(v) for v in surface['uvs'])
        obj.extend('vn ' + _text_vector(v) for v in surface['normals'])
        obj.append('usemtl ' + _material_name(surface['material_key']))
        for triangle in surface['triangles']:
            references = ['%s/%s/%s' % (vertex_offset+i, uv_offset+i if surface['uvs'] else '', normal_offset+i)
                          for i in triangle]
            obj.append('f ' + ' '.join(references))
        provenance.append({'context_id': surface['context_id'], 'face_index': surface['face_index'],
                            'first_triangle': triangle_offset, 'triangle_count': len(surface['triangles']),
                            'material': _material_name(surface['material_key']), 'appearance': surface['appearance']})
        vertex_offset += len(surface['vertices_mm'])
        uv_offset += len(surface['uvs'])
        normal_offset += len(surface['normals'])
        triangle_offset += len(surface['triangles'])
    obj_data = ('\n'.join(obj) + '\n').encode('ascii')
    obj_path = 'meshes/visual_' + hashlib.sha256(obj_data).hexdigest() + '.obj'
    files = {obj_path: obj_data, 'meshes/' + mtl_name: mtl_data}
    files.update(('meshes/' + _image_path(digest, image), image['data'])
                 for digest, image in sorted(images.items()))
    warnings = {warning for surface in surfaces if surface['appearance'] is not None
                for warning in surface['appearance']['warnings']}
    if any(image['format'] == 'jpeg' for image in images.values()):
        warnings.add('jpeg-structure-only-not-entropy-decoded')
    if any(surface['appearance'] is None for surface in surfaces):
        warnings.add('missing-appearance-provenance')
    descriptor = {'schema': 'fusion-visual-assets/1', 'format': 'obj', 'mesh_path': obj_path,
                  'frame': 'root', 'units': 'mm', 'urdf_scale': [0.001, 0.001, 0.001],
                  'urdf_material_override': None, 'triangle_count': triangle_offset,
                  'surfaces': provenance,
                  'images': [{'path': 'meshes/' + _image_path(digest, image), 'format': image['format'],
                              'validation': 'structure-only-not-entropy-decoded' if image['format'] == 'jpeg'
                              else 'framing-crc-bounded-scanlines'} for digest, image in sorted(images.items())],
                  'warnings': sorted(warnings),
                  'files': [
                      {'path': path, 'sha256': hashlib.sha256(data).hexdigest(), 'size': len(data)}
                      for path, data in sorted(files.items())]}
    if body_context is not None:
        descriptor['body_context'] = dict(body_context)
    return {'descriptor': descriptor, 'files': files}


def visual_descriptors(value):
    """Iterate validated visual descriptors without changing the combined schema."""
    return value if type(value) is list else (value,)


def _plain_descriptor(value):
    if type(value) in (str, int, float, bool, type(None)):
        return
    if type(value) is list:
        for item in value:
            _plain_descriptor(item)
        return
    if type(value) is dict and all(type(key) is str for key in value):
        for item in value.values():
            _plain_descriptor(item)
        return
    raise VisualAssetError('nonplain-asset-descriptor')


def validate_asset_bundles(bundles, inertial, policy, mesh_mode='combined'):
    """Validate the complete portable bundle at the public publication boundary."""
    physical = {name for name, entry in inertial.items() if entry.get('kind', 'physical') == 'physical'}
    if mesh_mode not in ('combined', 'per_body') or policy not in ('legacy', 'strict', 'diffuse-subset'):
        raise VisualAssetError('invalid-visual-policy')
    if policy == 'legacy':
        if mesh_mode != 'combined':
            raise VisualAssetError('per-body-requires-explicit-material-policy')
        if bundles:
            raise VisualAssetError('visual-assets-require-explicit-policy')
        return {}, {}
    if type(bundles) is not dict or set(bundles) != physical:
        raise VisualAssetError('visual-assets-physical-membership')
    descriptors, files, total, work = {}, {}, 0, []
    for name, value in bundles.items():
        if mesh_mode == 'per_body':
            if type(value) is not list or not 1 <= len(value) <= 4096:
                raise VisualAssetError('invalid-per-body-bundle-list')
            descriptors[name] = []
            work.extend((name, index, bundle) for index, bundle in enumerate(value))
        else:
            work.append((name, None, value))
    for name, index, bundle in work:
        if type(bundle) is not dict or set(bundle) != {'descriptor', 'files'} or type(bundle['files']) is not dict:
            raise VisualAssetError('invalid-asset-bundle')
        try:
            _plain_descriptor(bundle['descriptor'])
            descriptor = json.loads(json.dumps(bundle['descriptor'], allow_nan=False))
        except (TypeError, ValueError, OverflowError, RecursionError):
            raise VisualAssetError('nonplain-asset-descriptor') from None
        if descriptor != bundle['descriptor'] or not isinstance(descriptor, dict):
            raise VisualAssetError('nonplain-asset-descriptor')
        expected_fields = {'schema', 'format', 'mesh_path', 'frame', 'units', 'urdf_scale',
                           'urdf_material_override', 'triangle_count', 'surfaces', 'images', 'warnings', 'files'}
        if index is not None:
            expected_fields.add('body_context')
        if (set(descriptor) != expected_fields or descriptor['schema'] != 'fusion-visual-assets/1'
                or descriptor['format'] != 'obj' or descriptor['frame'] != 'root' or descriptor['units'] != 'mm'
                or descriptor['urdf_scale'] != [.001]*3 or descriptor['urdf_material_override'] is not None):
            raise VisualAssetError('invalid-asset-coordinate-contract')
        if any(type(descriptor[key]) is not list for key in ('surfaces', 'images', 'warnings', 'files')):
            raise VisualAssetError('invalid-asset-list-shape')
        if index is not None:
            context = descriptor['body_context']
            expected_id = '%s_body_%d' % (name, index)
            if (type(context) is not dict or set(context) != {'id', 'source_path', 'selected_body_count'}
                    or context['id'] != expected_id or type(context['source_path']) is not str
                    or not context['source_path'] or '\x00' in context['source_path']
                    or type(context['selected_body_count']) is not int
                    or context['selected_body_count'] != len(bundles[name])
                    or (inertial[name].get('source_path') is not None
                        and context['source_path'] != inertial[name]['source_path'])
                    or not descriptor['surfaces']
                    or any(type(s) is not dict or s.get('context_id') != expected_id for s in descriptor['surfaces'])):
                raise VisualAssetError('invalid-or-unselected-body-context')
        payloads = bundle['files']
        if not 2 <= len(payloads) <= 2048 or len(descriptor['files']) != len(payloads):
            raise VisualAssetError('invalid-asset-file-count')
        manifest = {}
        for entry in descriptor['files']:
            if type(entry) is not dict or set(entry) != {'path', 'sha256', 'size'}:
                raise VisualAssetError('invalid-asset-manifest')
            path = entry['path']
            if (not isinstance(path, str) or not re.fullmatch(r'meshes/(?:textures/)?[A-Za-z0-9_.]+', path)
                    or path in manifest or path not in payloads):
                raise VisualAssetError('unsafe-or-duplicate-asset-path')
            data = payloads[path]
            if type(data) is not bytes or not 0 < len(data) <= 64*1024*1024:
                raise VisualAssetError('invalid-asset-byte-size')
            digest = hashlib.sha256(data).hexdigest()
            suffix = PurePosixPath(path).suffix
            canonical = ('meshes/textures/' + digest + suffix if suffix in ('.png', '.jpg')
                         else 'meshes/visual_' + digest + suffix)
            if (path != canonical or suffix not in ('.obj', '.mtl', '.png', '.jpg')
                    or entry['sha256'] != digest or type(entry['size']) is not int or entry['size'] != len(data)):
                raise VisualAssetError('asset-content-hash-size-or-name')
            manifest[path] = entry
            if path in files and files[path] != data:
                raise VisualAssetError('asset-file-conflict')
            if path not in files:
                total += len(data)
                if total > 128*1024*1024 or len(files) >= 4096:
                    raise VisualAssetError('asset-package-budget')
                files[path] = data
        if set(manifest) != set(payloads):
            raise VisualAssetError('unmanifested-asset-file')
        _validate_obj_bundle(descriptor, payloads, policy)
        if index is None:
            descriptors[name] = descriptor
        else:
            descriptors[name].append(descriptor)
    return descriptors, files


def _validate_obj_bundle(descriptor, files, policy):
    try:
        mesh_path = descriptor['mesh_path']
        vertices, normals, uvs, triangles, libraries, selected, objects = [], [], [], [], [], None, []
        for line in files[mesh_path].decode('ascii').splitlines():
            fields = line.split()
            if not fields:
                continue
            key, values = fields[0], fields[1:]
            if key == 'v':
                vertices.append(_vector([float(v) for v in values], 3))
            elif key == 'vn':
                normals.append(_unit([float(v) for v in values]))
            elif key == 'vt':
                uvs.append(_vector([float(v) for v in values], 2))
            elif key == 'mtllib' and len(values) == 1:
                libraries.append('meshes/' + values[0])
            elif key == 'usemtl' and len(values) == 1:
                selected = values[0]
            elif key == 'o' and len(values) == 1 and 'body_context' in descriptor:
                objects.append(values[0])
            elif key == 'f' and len(values) == 3:
                corners = []
                for value in values:
                    v, uv, n = value.split('/')
                    vi, ni, ti = int(v)-1, int(n)-1, int(uv)-1 if uv else None
                    if not (0 <= vi < len(vertices) and 0 <= ni < len(normals)) or (ti is not None and not 0 <= ti < len(uvs)):
                        raise VisualAssetError('invalid-obj-index')
                    corners.append((vi, ti, ni))
                a, b, c = [vertices[v] for v, _, _ in corners]
                ab, ac = [b[i]-a[i] for i in range(3)], [c[i]-a[i] for i in range(3)]
                area = math.hypot(ab[1]*ac[2]-ab[2]*ac[1], ab[2]*ac[0]-ab[0]*ac[2], ab[0]*ac[1]-ab[1]*ac[0])
                if not math.isfinite(area) or area == 0:
                    raise VisualAssetError('degenerate-obj-triangle')
                triangles.append((selected, corners))
            else:
                raise VisualAssetError('unsupported-obj-record')
        if len(libraries) != 1 or libraries[0] not in files or not mesh_path.endswith('.obj'):
            raise VisualAssetError('invalid-obj-library')
        if objects != ([descriptor['body_context']['id']] if 'body_context' in descriptor else []):
            raise VisualAssetError('invalid-obj-body-context')
        materials, current, images = {}, None, set()
        for line in files[libraries[0]].decode('ascii').splitlines():
            fields = line.split()
            if not fields:
                continue
            key, values = fields[0], fields[1:]
            if key == 'newmtl' and len(values) == 1 and values[0] not in materials:
                current = materials.setdefault(values[0], {})
            elif key in ('Kd', 'd', 'illum', 'map_Kd') and current is not None and key not in current:
                current[key] = values
            else:
                raise VisualAssetError('unsupported-mtl-record')
        for material_name, material in materials.items():
            rgba = _vector([float(v) for v in material['Kd'] + material['d']], 4)
            if any(not 0 <= v <= 1 for v in rgba) or material['illum'] != ['1']:
                raise VisualAssetError('invalid-mtl-material')
            if 'map_Kd' in material:
                if len(material['map_Kd']) != 1:
                    raise VisualAssetError('invalid-mtl-image')
                path = 'meshes/' + material['map_Kd'][0]
                if path not in files or not path.startswith('meshes/textures/'):
                    raise VisualAssetError('missing-mtl-image')
                image_format = _image_format(files[path])
                if not path.endswith('.jpg' if image_format == 'jpeg' else '.png'):
                    raise VisualAssetError('conflicting-image-format')
                images.add(path)
            fingerprint = {'rgba': rgba, 'image': PurePosixPath(path).stem if 'map_Kd' in material else None}
            digest = hashlib.sha256(json.dumps(fingerprint, sort_keys=True, allow_nan=False,
                                               separators=(',', ':')).encode()).hexdigest()
            if material_name != _material_name(digest):
                raise VisualAssetError('material-content-hash-mismatch')
        expected_images = [{'path': path, 'format': 'jpeg' if path.endswith('.jpg') else 'png',
                            'validation': 'structure-only-not-entropy-decoded' if path.endswith('.jpg')
                            else 'framing-crc-bounded-scanlines'} for path in sorted(images)]
        if sorted(descriptor['images'], key=lambda i: i['path']) != expected_images:
            raise VisualAssetError('invalid-image-validation-receipt')
        if ({mesh_path, libraries[0]} | images) != set(files) or {m for m, _ in triangles} != set(materials):
            raise VisualAssetError('unselected-or-missing-asset-resource')
        offset, contexts, warnings = 0, set(), set()
        for surface in descriptor['surfaces']:
            if set(surface) != {'context_id', 'face_index', 'first_triangle', 'triangle_count', 'material', 'appearance'}:
                raise VisualAssetError('invalid-asset-surface')
            context, face = surface['context_id'], surface['face_index']
            if (not isinstance(context, str) or not _CONTEXT.fullmatch(context) or context in ('.', '..')
                    or type(face) is not int or face < 0 or (context, face) in contexts):
                raise VisualAssetError('invalid-asset-surface-identity')
            contexts.add((context, face))
            count = surface['triangle_count']
            if (type(count) is not int or count <= 0 or type(surface['first_triangle']) is not int
                    or surface['first_triangle'] != offset):
                raise VisualAssetError('invalid-asset-triangle-range')
            material = materials[surface['material']]
            for selected, corners in triangles[offset:offset+count]:
                if selected != surface['material'] or ('map_Kd' in material and any(uv is None for _, uv, _ in corners)):
                    raise VisualAssetError('asset-face-material-uv-mismatch')
            receipt = _validated_receipt(surface['appearance'], 'map_Kd' in material)
            if receipt is None or receipt['shader_policy'] != policy:
                raise VisualAssetError('asset-policy-provenance-required')
            warnings.update(receipt['warnings'])
            offset += count
        if any(path.endswith('.jpg') for path in images):
            warnings.add('jpeg-structure-only-not-entropy-decoded')
        if (not triangles or offset != len(triangles) or type(descriptor['triangle_count']) is not int
                or descriptor['triangle_count'] != offset or descriptor['warnings'] != sorted(warnings)):
            raise VisualAssetError('asset-triangle-or-warning-receipt-mismatch')
    except VisualAssetError:
        raise
    except (TypeError, KeyError, ValueError, IndexError, UnicodeError, OverflowError):
        raise VisualAssetError('malformed-obj-asset-bundle') from None
