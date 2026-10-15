"""Export configuration (stdlib TOML, no third-party deps).

Mirrors the idea of Adriaeik's ``xacro_export.toml`` and cadop's YAML
config, reduced to keys this exporter actually honors. Fusion 360 ships
Python 3.12+, so :mod:`tomllib` is available; offline tests run on 3.11+.
"""

try:
    import tomllib
except ImportError:  # pragma: no cover - Python < 3.11 fallback
    tomllib = None

CONFIG_FILENAME = 'xacro_export.toml'

DEFAULTS = {
    # '1' | '2' | 'both': dheena2k2/newtonjeri/runtimerobotics showed ROS2-only
    # splits the user base; emit both by default from one export.
    'ros_version': 'both',
    # Fusion STL mesh refinement: 'low' | 'medium' | 'high'.
    'mesh_refinement': 'low',
    # 'visual' reuses the visual mesh (legacy behavior); 'primitive_box'
    # fits an oriented box from the Fusion bounding box for cheap physics
    # (Adriaeik's primitive-collision idea, cadop's combined-collision idea).
    'collision_method': 'visual',
    'effort': 100.0,
    'velocity': 100.0,
    # Legacy fusion2urdf duplicated components into new occs (destructive:
    # renames originals to old_component). Default False = WYSIWYG direct
    # export without touching the design (cadop/newtonjeri direction).
    'use_legacy_copy': False,
    'export_transmissions_ros1': True,
    'export_ros2_control': True,
}

_VALID = {
    'ros_version': ('1', '2', 'both'),
    'mesh_refinement': ('low', 'medium', 'high'),
    'collision_method': ('visual', 'primitive_box'),
}


def load_config(script_dir=None, overrides=None):
    """Load config with defaults; never raise on missing/bad files."""
    config = dict(DEFAULTS)
    if script_dir:
        try:
            import os
            path = os.path.join(script_dir, CONFIG_FILENAME)
            with open(path, 'rb') as fh:
                data = tomllib.load(fh) if tomllib is not None else {}
            if isinstance(data, dict):
                for key, value in data.items():
                    if key in DEFAULTS:
                        config[key] = value
        except Exception:
            pass
    if overrides:
        for key, value in overrides.items():
            if key in DEFAULTS:
                config[key] = value
    for key, allowed in _VALID.items():
        if config.get(key) not in allowed:
            config[key] = DEFAULTS[key]
    for key in ('effort', 'velocity'):
        try:
            config[key] = float(config[key])
        except Exception:
            config[key] = DEFAULTS[key]
    for key in ('use_legacy_copy', 'export_transmissions_ros1', 'export_ros2_control'):
        config[key] = bool(config.get(key))
    return config


def mesh_refinement_value(config, adsk_fusion=None):
    """Map config refinement to a Fusion enum value (or None when unknown)."""
    name = (config or {}).get('mesh_refinement', 'low')
    mapping = {'low': 'MeshRefinementLow', 'medium': 'MeshRefinementMedium',
               'high': 'MeshRefinementHigh'}
    attr = mapping.get(name, 'MeshRefinementLow')
    try:
        settings = getattr(adsk_fusion, 'MeshRefinementSettings', None)
        if settings is not None and hasattr(settings, attr):
            return getattr(settings, attr)
    except Exception:
        pass
    return None


def wants_ros1(config):
    """True when ROS1 package files should be emitted."""
    return (config or {}).get('ros_version', 'both') in ('1', 'both')


def wants_ros2(config):
    """True when ROS2 package files should be emitted."""
    return (config or {}).get('ros_version', 'both') in ('2', 'both')
