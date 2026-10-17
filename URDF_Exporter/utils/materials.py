"""Extract one RGBA per link without importing adsk.

Native Color uses byte RGB and opacity. Untyped compatibility colors use
normalized opacity/alpha and either normalized RGB or byte RGB as a group.
A usable primary color takes precedence. Without it, unsupported legacy
texture/multiple-value properties or invalid colors return None for writer
silver. Texture patterns and distinct face/body materials are not exported.
"""

import inspect
import math


_COLOR_PROPERTY_IDS = ('opaque_albedo', 'metal_f0', 'transparent_color', 'generic_diffuse')
_UNREADABLE = object()


def _read(source, attribute, unreadable=None):
    try:
        value = getattr(source, attribute)
        return value() if callable(value) else value
    except AttributeError:
        return None if inspect.getattr_static(source, attribute, None) is None else unreadable
    except Exception:
        return unreadable


def appearance_to_rgba(appearance):
    """Read Appearance.color or a supported appearanceProperties ColorProperty."""
    if appearance is None:
        return None
    rgba = _rgba_from_color_like(_read(appearance, 'color'), native=True)
    if rgba is not None:
        return rgba
    properties = _read(appearance, 'appearanceProperties')
    if properties is not None:
        for identifier in _COLOR_PROPERTY_IDS:
            try:
                prop = properties.itemById(identifier)
            except Exception:
                continue
            if prop is None:
                continue
            if (_read(prop, 'objectType') != 'adsk::core::ColorProperty'
                    or _read(prop, 'hasConnectedTexture') is not False
                    or _read(prop, 'hasMultipleValues') is not False):
                return None
            return _rgba_from_color_like(_read(prop, 'value'), native=True)
        return None
    if _read(appearance, 'objectType') == 'adsk::core::Appearance':
        return None
    for attr in ('diffuse', 'color', 'baseColor'):
        rgba = _rgba_from_color_like(_read(appearance, attr))
        if rgba is not None:
            return rgba
    return _rgba_from_color_like(appearance)


def _rgba_from_color_like(color, native=False):
    if color is None:
        return None
    kind = _read(color, 'objectType')
    if native or kind is not None:
        if kind != 'adsk::core::Color':
            return None
        native = True
    try:
        channels = []
        for attr in ('red', 'green', 'blue'):
            value = getattr(color, attr)
            channels.append(float(value() if callable(value) else value))
        opacity = getattr(color, 'opacity', None)
        if opacity is None and not native:
            opacity = getattr(color, 'alpha', 1.0)
        opacity = float(opacity() if callable(opacity) else opacity)
    except Exception:
        return None
    rgb_scale = 255.0 if native or max(channels) > 1.0 else 1.0
    alpha_scale = 255.0 if native else 1.0
    if (any(not math.isfinite(c) or not 0.0 <= c <= rgb_scale for c in channels)
            or not math.isfinite(opacity) or not 0.0 <= opacity <= alpha_scale):
        return None
    return tuple(c / rgb_scale for c in channels) + (opacity / alpha_scale,)


def occurrence_rgba(occurrence, bodies=()):
    """Select owner override, canonical body appearance, then material fallback.

    A present unsupported or unreadable appearance stops the search with None.
    Bodies must be the selected direct bodies, in their canonical order.
    """
    if occurrence is None:
        return None
    sources = (occurrence,) + tuple(bodies)
    candidates = [(source, 'appearance') for source in sources]
    candidates.extend((source, attr) for source in sources
                      for attr in ('material', 'physicalMaterial'))
    component = _read(occurrence, 'component')
    candidates.extend((component, attr)
                      for attr in ('appearance', 'material', 'physicalMaterial'))
    for source, attr in candidates:
        candidate = _read(source, attr, unreadable=_UNREADABLE)
        if candidate is None:
            continue
        if attr != 'appearance':
            appearance = _read(candidate, 'appearance', unreadable=_UNREADABLE)
            if appearance is not None:
                candidate = appearance
        return appearance_to_rgba(candidate)
    return None


def rgba_to_urdf_string(rgba):
    """Format (r,g,b,a) for a URDF color attribute."""
    try:
        return ' '.join(str(float(c)) for c in list(rgba)[:4])
    except Exception:
        return '0.7 0.7 0.7 1.0'
