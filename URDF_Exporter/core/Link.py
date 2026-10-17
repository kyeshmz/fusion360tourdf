# Original exporter by syuntoku14. See the repository MIT license.
import adsk
from ..utils import utils
from ..utils import materials as matlib


def _physical_data(source):
    prop = source.getPhysicalProperties(adsk.fusion.CalculationAccuracy.VeryHighCalculationAccuracy)
    center = [value / 100.0 for value in prop.centerOfMass.asArray()]
    success, *moments = prop.getXYZMomentsOfInertia()
    if not success:
        raise ValueError('Fusion could not calculate moments of inertia.')
    return prop.mass, center, [value / 10000.0 for value in moments]


def _inertial_entry(source):
    properties = [_physical_data(body) for body in source.bodies]
    mass = sum(value[0] for value in properties)
    if mass <= 0:
        raise ValueError('Link %s needs positive mass.' % utils.source_path(source.owner))
    center = [sum(m * c[index] for m, c, _ in properties) / mass for index in range(3)]
    moments = [sum(tensor[index] for _, _, tensor in properties) for index in range(6)]
    bbox_center, bbox_size = _bbox_m(source.bodies)
    color = matlib.occurrence_rgba(source.owner, source.bodies)
    return {
        'mass': mass,
        'center_of_mass': center,
        'inertia': utils.origin2center_of_mass(moments, center, mass),
        'color_rgba': color,
        'bbox_center_m': bbox_center,
        'bbox_size_m': bbox_size,
    }


def make_inertial_dict(sources, msg):
    return {name: _inertial_entry(source) for name, source in sources.items()}, msg


def _bbox_m(bodies):
    boxes = [getattr(body, 'boundingBox', None) for body in bodies]
    if any(box is None for box in boxes):
        return None, None
    lower = [box.minPoint.asArray() for box in boxes]
    upper = [box.maxPoint.asArray() for box in boxes]
    lo = [min(point[index] for point in lower) / 100.0 for index in range(3)]
    hi = [max(point[index] for point in upper) / 100.0 for index in range(3)]
    return [(a + b) / 2 for a, b in zip(lo, hi)], [b - a for a, b in zip(lo, hi)]
