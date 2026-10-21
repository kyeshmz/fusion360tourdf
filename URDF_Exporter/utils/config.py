import math
from collections.abc import Mapping
from numbers import Real
from pathlib import Path
from . import transforms as tf
from .validate import _inertia_error
from .convex_hull import HULL_DEFAULTS, HULL_MAXIMUMS, HULL_SOURCE_POLICIES

CONFIG_FILENAME = 'xacro_export.toml'
DEFAULTS = {
    'mesh_refinement': 'low',
    'collision_method': 'visual',
    'effort': 100.0,
    'velocity': 100.0,
    'root_occurrence': '',
}
POLICY_DEFAULTS = {'generalized_joints': 'reject', 'ball_chart': 'reject',
                    'motion_links': 'reject', 'visual_materials': 'legacy', 'visual_meshes': 'combined'}
POLICY_OPTIONS = ('output_rotation', 'output_rotation_preset', 'frame_convention', 'minimum_principal_inertia')
JOINT_LIMIT_FIELDS = ('effort', 'velocity')
COLLISION_DEFAULTS = {'resolution': 16, 'max_pieces': 32768, 'max_cells': 32768,
                      'max_work': 2000000, 'max_triangles': 10000, 'max_pair_checks': 2000000}
COLLISION_MAXIMUMS = {'resolution': 64, 'max_pieces': 262144, 'max_cells': 262144,
                      'max_work': 10000000, 'max_triangles': 100000, 'max_pair_checks': 10000000}
MESH_LIMITS = {'max_triangles': 10000, 'max_nodes': 30000, 'max_pair_checks': 2000000}
MESH_MAXIMUMS = {'max_triangles': 100000, 'max_nodes': 300000, 'max_pair_checks': 10000000}


def _positive_number(value, label):
    message = '%s must be a finite positive number' % label
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(message)
    try:
        number = float(value)
    except (OverflowError, ValueError):
        raise ValueError(message) from None
    if not math.isfinite(number) or number <= 0:
        raise ValueError(message)
    return number


def _joint_limits(values):
    if not isinstance(values, Mapping):
        raise ValueError('joint_limits must be a table keyed by exact exported joint names')
    result = {}
    for name, fields in values.items():
        if not isinstance(name, str) or not name:
            raise ValueError('joint_limits selector must be a nonempty exact exported joint name')
        if not isinstance(fields, Mapping):
            raise ValueError('joint_limits[%r] must be a table of effort and velocity options' % name)
        unknown = fields.keys() - JOINT_LIMIT_FIELDS
        if unknown:
            raise ValueError('Unknown joint_limits[%r] options: %s' %
                             (name, ', '.join(sorted(map(str, unknown)))))
        result[name] = {key: _positive_number(value, 'joint_limits[%r].%s' % (name, key))
                        for key, value in fields.items()}
    return result


def _export_policies(config):
    matrix = config.get('output_rotation')
    preset = config.get('output_rotation_preset')
    if preset not in (None, 'y_up_to_z_up'):
        raise ValueError('output_rotation_preset must be None or y_up_to_z_up')
    if matrix is not None:
        if preset is not None:
            raise ValueError('output_rotation and output_rotation_preset are mutually exclusive')
        if not isinstance(matrix, (list, tuple)) or len(matrix) != 9:
            raise ValueError('output_rotation must be a row-major 9-number rotation')
        for value in matrix:
            try:
                finite = isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
            except OverflowError:
                finite = False
            if not finite:
                raise ValueError('output_rotation must contain finite non-boolean numbers')
        gram = tf.mat3_mul(tf.mat3_transpose(matrix), matrix)
        determinant = (matrix[0]*(matrix[4]*matrix[8]-matrix[5]*matrix[7])
                       - matrix[1]*(matrix[3]*matrix[8]-matrix[5]*matrix[6])
                       + matrix[2]*(matrix[3]*matrix[7]-matrix[4]*matrix[6]))
        if max(abs(a-b) for a, b in zip(gram, tf.IDENTITY_3X3)) > 1e-12 or abs(determinant-1) > 1e-12:
            raise ValueError('output_rotation must be a proper orthonormal rotation')
        config['output_rotation'] = list(matrix)
    if config.get('frame_convention') not in (None, 'dh_common_normal'):
        raise ValueError('frame_convention must be None or dh_common_normal')
    value = config.get('minimum_principal_inertia')
    if value is not None:
        try:
            valid = isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value) and value >= 0
        except OverflowError:
            valid = False
        if not valid:
            raise ValueError('minimum_principal_inertia must be finite and nonnegative in kg*m^2')


def _finite_vector(value, size, label):
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise ValueError('%s must contain %d finite SI numbers' % (label, size))
    result = []
    for item in value:
        if isinstance(item, bool) or not isinstance(item, Real):
            raise ValueError('%s must contain finite SI numbers' % label)
        try:
            number = float(item)
        except (OverflowError, ValueError):
            raise ValueError('%s must contain finite SI numbers' % label) from None
        if not math.isfinite(number):
            raise ValueError('%s must contain finite SI numbers' % label)
        result.append(number)
    return result


def _mesh_physics(values):
    if not isinstance(values, Mapping):
        raise ValueError('mesh_physics must be a table')
    unknown = values.keys() - {'density_kg_m3', 'bodies'} - MESH_LIMITS.keys()
    if unknown:
        raise ValueError('Unknown mesh_physics options: ' + ', '.join(sorted(map(str, unknown))))
    result = dict(MESH_LIMITS)
    for key, maximum in MESH_MAXIMUMS.items():
        value = values.get(key, result[key])
        if isinstance(value, bool) or not isinstance(value, int) or not 1 <= value <= maximum:
            raise ValueError('mesh_physics.%s must be an integer in [1, %d]' % (key, maximum))
        result[key] = value
    if 'density_kg_m3' in values:
        result['density_kg_m3'] = _positive_number(values['density_kg_m3'], 'mesh density_kg_m3')
    bodies = values.get('bodies', [])
    if not isinstance(bodies, (list, tuple)):
        raise ValueError('mesh_physics.bodies must be an array of exact occurrence/body tables')
    result['bodies'] = []
    selectors = set()
    for body in bodies:
        if not isinstance(body, Mapping):
            raise ValueError('Each mesh_physics body must be a table')
        selector = tuple(body.get(key) for key in ('occurrence', 'body'))
        if any(not isinstance(value, str) or not value for value in selector):
            raise ValueError('Mesh selector requires exact nonempty occurrence and body strings')
        if selector in selectors:
            raise ValueError('Duplicate mesh selector: %r' % (selector,))
        selectors.add(selector)
        item = dict(zip(('occurrence', 'body'), selector))
        if 'density_kg_m3' in body:
            fields = {'occurrence', 'body', 'density_kg_m3'}
            item['density_kg_m3'] = _positive_number(body['density_kg_m3'], 'mesh density_kg_m3')
        else:
            fields = {'occurrence', 'body', 'mass_kg', 'center_of_mass_m', 'inertia_kg_m2', 'frame', 'provenance'}
            if fields - body.keys():
                raise ValueError('Supplied mesh physics requires mass_kg, center_of_mass_m, inertia_kg_m2, frame and provenance')
            item['mass_kg'] = _positive_number(body['mass_kg'], 'mesh mass_kg')
            item['center_of_mass_m'] = _finite_vector(body['center_of_mass_m'], 3, 'mesh center_of_mass_m')
            tensor = _finite_vector(body['inertia_kg_m2'], 6, 'mesh inertia_kg_m2')
            if any(value <= 0 for value in tensor[:3]) or _inertia_error(tensor):
                raise ValueError('Supplied mesh central inertia must be physical with positive diagonal and principal-moment triangle inequalities')
            item['inertia_kg_m2'] = tensor
            if body['frame'] not in ('native', 'design_root'):
                raise ValueError('Mesh physics frame must explicitly be native or design_root')
            if not isinstance(body['provenance'], str) or not body['provenance'].strip():
                raise ValueError('Supplied mesh physics requires nonempty SI provenance')
            item.update(frame=body['frame'], provenance=body['provenance'])
        if body.keys() - fields:
            raise ValueError('Unknown or mixed mesh body physics options: ' + ', '.join(sorted(map(str, body.keys() - fields))))
        result['bodies'].append(item)
    return result


def load_config(script_dir=None, overrides=None):
    config = dict(DEFAULTS)
    collision = dict(COLLISION_DEFAULTS)
    hull = dict(HULL_DEFAULTS)
    hull_source_policy = HULL_SOURCE_POLICIES[0]
    data = {}
    if script_dir:
        path = Path(script_dir) / CONFIG_FILENAME
        if path.exists():
            try:
                import tomllib
            except ImportError as error:
                raise ValueError('Reading TOML config requires Python 3.11 or newer') from error
            try:
                with path.open('rb') as stream:
                    data = tomllib.load(stream)
            except (OSError, ValueError) as error:
                raise ValueError('Cannot read %s: %s' % (path, error)) from error
    for values in (data, overrides if overrides is not None else {}):
        if not isinstance(values, dict):
            raise ValueError('Export config must be a table of options')
        unknown = (values.keys() - DEFAULTS.keys() - POLICY_DEFAULTS.keys() - set(POLICY_OPTIONS)
                   - {'joint_limits', 'convex_decomposition', 'convex_hull', 'mesh_physics'})
        if unknown:
            raise ValueError('Unknown export options: ' + ', '.join(sorted(map(str, unknown))))
        _export_policies(dict(values))
        config.update((key, value) for key, value in values.items()
                      if key not in ('joint_limits', 'convex_decomposition', 'convex_hull', 'mesh_physics'))
        if 'mesh_physics' in values:
            config['mesh_physics'] = _mesh_physics(values['mesh_physics'])
        if 'convex_hull' in values:
            options = values['convex_hull']
            if not isinstance(options, Mapping):
                raise ValueError('convex_hull must be a table of limits and source_policy')
            unknown = options.keys() - HULL_DEFAULTS.keys() - {'source_policy'}
            if unknown:
                raise ValueError('Unknown convex_hull options: ' + ', '.join(sorted(map(str, unknown))))
            hull.update((key, value) for key, value in options.items() if key != 'source_policy')
            if 'source_policy' in options:
                hull_source_policy = options['source_policy']
        if 'convex_decomposition' in values:
            options = values['convex_decomposition']
            if not isinstance(options, Mapping):
                raise ValueError('convex_decomposition must be a table of integer controls')
            unknown = options.keys() - COLLISION_DEFAULTS.keys()
            if unknown:
                raise ValueError('Unknown convex_decomposition options: ' + ', '.join(sorted(map(str, unknown))))
            collision.update(options)
        if 'joint_limits' in values:
            for name, fields in _joint_limits(values['joint_limits']).items():
                config.setdefault('joint_limits', {}).setdefault(name, {}).update(fields)
    for key, choices in (('mesh_refinement', ('low', 'medium', 'high')),
                         ('collision_method', ('visual', 'primitive_box', 'convex_decomposition', 'convex_hull')),
                         ('generalized_joints', ('reject', 'decompose')),
                         ('ball_chart', ('reject', 'export-local-rx-ry-rz')),
                         ('motion_links', ('reject', 'mimic')),
                         ('visual_materials', ('legacy', 'strict', 'diffuse-subset')),
                         ('visual_meshes', ('combined', 'per_body'))):
        if config.get(key, POLICY_DEFAULTS.get(key)) not in choices:
            raise ValueError('%s must be one of %s' % (key, ', '.join(choices)))
    if config.get('visual_meshes', 'combined') == 'per_body' and config.get('visual_materials', 'legacy') == 'legacy':
        raise ValueError('visual_meshes=per_body requires explicit visual_materials=strict or diffuse-subset')
    for key in JOINT_LIMIT_FIELDS:
        config[key] = _positive_number(config[key], key)
    if not isinstance(config['root_occurrence'], str):
        raise ValueError('root_occurrence must be an exact full occurrence path or design_root')
    for key, value in collision.items():
        if isinstance(value, bool) or not isinstance(value, int) or not 1 <= value <= COLLISION_MAXIMUMS[key]:
            raise ValueError('convex_decomposition.%s must be an integer in [1, %d]' % (key, COLLISION_MAXIMUMS[key]))
    if config['collision_method'] == 'convex_decomposition' or collision != COLLISION_DEFAULTS:
        config['convex_decomposition'] = collision
    for key, value in hull.items():
        if isinstance(value, bool) or not isinstance(value, int) or not 1 <= value <= HULL_MAXIMUMS[key]:
            raise ValueError('convex_hull.%s must be an integer in [1, %d]' % (key, HULL_MAXIMUMS[key]))
    if hull_source_policy not in HULL_SOURCE_POLICIES:
        raise ValueError('convex_hull.source_policy must be one of ' + ', '.join(HULL_SOURCE_POLICIES))
    if hull_source_policy != HULL_SOURCE_POLICIES[0]:
        hull['source_policy'] = hull_source_policy
    if config['collision_method'] == 'convex_hull' or hull != HULL_DEFAULTS:
        config['convex_hull'] = hull
    _export_policies(config)
    return config


def mesh_refinement_value(config, adsk_fusion):
    return getattr(adsk_fusion.MeshRefinementSettings,
                   'MeshRefinement' + config['mesh_refinement'].capitalize())
