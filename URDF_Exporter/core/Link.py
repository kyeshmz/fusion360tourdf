# Original exporter by syuntoku14. See the repository MIT license.
import adsk
import math
from ..utils import utils
from ..utils import materials as matlib
from ..utils.mesh_inputs import MeshSnapshot


def _physical_data(source):
    """Return mass, center in cm, and central inertia in kg*cm^2."""
    if isinstance(source, MeshSnapshot):
        return source.mass, source.center_cm, source.central_kg_cm2
    prop = source.getPhysicalProperties(adsk.fusion.CalculationAccuracy.VeryHighCalculationAccuracy)
    center = prop.centerOfMass.asArray()
    success, *moments = prop.getXYZMomentsOfInertia()
    if not success:
        raise ValueError('Fusion could not calculate moments of inertia.')
    # Shift before SI conversion to avoid subtracting separately rounded large values.
    return prop.mass, center, utils.origin2center_of_mass(moments, center, prop.mass)


def _require_solid_brep(body, path, index):
    try:
        name = body.name
    except Exception:
        name = 'body[%d]' % index
    label = '%s/%s' % (path, name)
    try:
        valid = body.isValid
    except Exception as error:
        raise ValueError('BREP_BODY_METADATA_UNAVAILABLE: %s cannot read isValid' % label) from error
    if valid is not True:
        raise ValueError('BREP_BODY_INVALID: %s requires a valid body' % label)
    try:
        solid = body.isSolid
    except Exception as error:
        raise ValueError('BREP_BODY_METADATA_UNAVAILABLE: %s cannot read isSolid' % label) from error
    if solid is False:
        raise ValueError('BREP_SURFACE_PHYSICS_UNSUPPORTED: %s is non-solid. Solid BRep CAD physics is required; '
                         'surface thickness and mass are not inferred.' % label)
    if solid is not True:
        raise ValueError('BREP_BODY_METADATA_INVALID: %s isSolid must be a boolean' % label)


def _inertial_entry(source, source_path=None):
    path = source_path if source_path is not None else utils.source_path(source.owner)
    for index, body in enumerate(source.bodies):
        if not isinstance(body, MeshSnapshot):
            _require_solid_brep(body, path, index)
    properties = [_physical_data(body) for body in source.bodies]
    mass = math.fsum(value[0] for value in properties)
    if mass <= 0:
        raise ValueError('Link %s needs positive mass.' % utils.source_path(source.owner))
    reference = properties[0][1]
    relative_center = [math.fsum(m * (c[index] - reference[index]) for m, c, _ in properties) / mass
                       for index in range(3)]
    center = [(a + b) / 100.0 for a, b in zip(reference, relative_center)]
    shifted = []
    for body_mass, body_center, tensor in properties:
        x, y, z = [(c - r) - offset for c, r, offset in zip(body_center, reference, relative_center)]
        shift = [y*y + z*z, x*x + z*z, x*x + y*y, -x*y, -y*z, -x*z]
        shifted.append([value + body_mass * term for value, term in zip(tensor, shift)])
    moments = [math.fsum(tensor[index] for tensor in shifted) / 10000.0 for index in range(6)]
    bbox_center, bbox_size = _bbox_m(source.bodies)
    color = matlib.occurrence_rgba(source.owner, tuple(body.original if isinstance(body, MeshSnapshot) else body
                                                    for body in source.bodies))
    result = {
        'mass': mass,
        'center_of_mass': center,
        'inertia': moments,
        'color_rgba': color,
        'bbox_center_m': bbox_center,
        'bbox_size_m': bbox_size,
    }
    meshes = [body.provenance for body in source.bodies if isinstance(body, MeshSnapshot)]
    if meshes:
        result['mesh_physics'] = meshes
    return result


def make_inertial_dict(sources, msg):
    entries = {}
    for name, source in sources.items():
        path = sources.source_path(name) if isinstance(sources, utils.SourceRegistry) else utils.source_path(source.owner)
        entry = _inertial_entry(source, source_path=path) if source.kind == 'physical' else {}
        entry['kind'] = source.kind
        entry['source_path'] = path
        entries[name] = entry
    return entries, msg


def _bbox_m(bodies):
    lower, upper = [], []
    for body in bodies:
        if isinstance(body, MeshSnapshot):
            lower.append(body.lower_cm)
            upper.append(body.upper_cm)
        else:
            box = getattr(body, 'boundingBox', None)
            if box is None:
                return None, None
            lower.append(box.minPoint.asArray())
            upper.append(box.maxPoint.asArray())
    lo = [min(point[index] for point in lower) / 100.0 for index in range(3)]
    hi = [max(point[index] for point in upper) / 100.0 for index in range(3)]
    if any(isinstance(body, MeshSnapshot) for body in bodies) and any(b <= a for a, b in zip(lo, hi)):
        return None, None
    return [(a + b) / 2 for a, b in zip(lo, hi)], [b - a for a, b in zip(lo, hi)]
