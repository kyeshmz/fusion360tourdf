import math
from pathlib import Path

CONFIG_FILENAME = 'xacro_export.toml'
DEFAULTS = {
    'mesh_refinement': 'low',
    'collision_method': 'visual',
    'effort': 100.0,
    'velocity': 100.0,
    'root_occurrence': '',
}


def load_config(script_dir=None, overrides=None):
    config = dict(DEFAULTS)
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
        unknown = values.keys() - DEFAULTS.keys()
        if unknown:
            raise ValueError('Unknown export options: ' + ', '.join(sorted(map(str, unknown))))
        config.update(values)
    for key, choices in (('mesh_refinement', ('low', 'medium', 'high')),
                         ('collision_method', ('visual', 'primitive_box'))):
        if config[key] not in choices:
            raise ValueError('%s must be one of %s' % (key, ', '.join(choices)))
    for key in ('effort', 'velocity'):
        value = config[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError('%s must be a finite positive number' % key)
        config[key] = float(value)
    if not isinstance(config['root_occurrence'], str):
        raise ValueError('root_occurrence must be an exact full occurrence path or design_root')
    return config


def mesh_refinement_value(config, adsk_fusion):
    return getattr(adsk_fusion.MeshRefinementSettings,
                   'MeshRefinement' + config['mesh_refinement'].capitalize())
