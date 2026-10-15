"""Fusion appearance/material helpers (best-effort, never raise).

Upstream fusion2urdf hardcodes every link to a global ``silver`` material.
newtonjeri/fusion2urdf-master added color detection; Adriaeik preserves
material/color per link. This module extracts RGBA defensively: the Fusion
API shape varies across versions, so every hop is guarded.
"""


def _float01(value):
    try:
        f = float(value)
    except Exception:
        return None
    if f < 0.0 or f > 255.0:
        # Could already be 0..1 or garbage; accept 0..1, clamp the rest.
        if 0.0 <= f <= 1.0:
            return f
        return max(0.0, min(1.0, f))
    # Heuristic: values > 1 are 0..255 channel values.
    if f > 1.0:
        return f / 255.0
    return f


def appearance_to_rgba(appearance):
    """Convert a Fusion appearance-ish object to (r, g, b, a) or None."""
    if appearance is None:
        return None
    # Common shape: appearance.diffuse.{red,green,blue,opacity}
    for attr in ('diffuse', 'color', 'baseColor'):
        try:
            candidate = getattr(appearance, attr, None)
            if candidate is None:
                continue
            if callable(candidate):
                candidate = candidate()
            rgba = _rgba_from_color_like(candidate)
            if rgba is not None:
                return rgba
        except Exception:
            continue
    return _rgba_from_color_like(appearance)


def _rgba_from_color_like(color):
    if color is None:
        return None
    channels = []
    for attr in ('red', 'green', 'blue'):
        try:
            value = getattr(color, attr, None)
            if value is None:
                continue
            if callable(value):
                value = value()
            channels.append(_float01(value))
        except Exception:
            return None
    if len(channels) != 3:
        return None
    if any(c is None for c in channels):
        return None
    opacity = 1.0
    for attr in ('opacity', 'alpha'):
        try:
            value = getattr(color, attr, None)
            if value is None:
                continue
            if callable(value):
                value = value()
            f = float(value)
            if 0.0 <= f <= 1.0:
                opacity = f
            elif 0.0 <= f <= 100.0:
                opacity = f / 100.0
            break
        except Exception:
            continue
    r, g, b = channels
    return (round(r, 3), round(g, 3), round(b, 3), round(opacity, 3))


def occurrence_rgba(occurrence):
    """Best-effort RGBA for an occurrence, or None to use the fallback."""
    if occurrence is None:
        return None
    sources = [occurrence]
    try:
        component = getattr(occurrence, 'component', None)
        if component is not None:
            sources.append(component)
    except Exception:
        pass
    for source in sources:
        for attr in ('appearance', 'material', 'physicalMaterial'):
            try:
                candidate = getattr(source, attr, None)
                if candidate is None:
                    continue
                if callable(candidate):
                    candidate = candidate()
                rgba = appearance_to_rgba(candidate)
                if rgba is not None:
                    return rgba
            except Exception:
                continue
    return None


def material_name_for(link_name):
    """Per-link material name (XML-safe)."""
    from .utils import sanitize_name
    return sanitize_name(link_name) + '_material'


def rgba_to_urdf_string(rgba):
    """Format (r,g,b,a) for a URDF color attribute."""
    try:
        return ' '.join(str(float(c)) for c in list(rgba)[:4])
    except Exception:
        return '0.7 0.7 0.7 1.0'
