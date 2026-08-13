"""Pure helpers for mapping joystick axes to ROS message values."""


def mapped_axis_value(source_value, mapping):
    """Apply direction-specific scaling while preserving legacy ``scale`` configs."""
    fallback_scale = mapping.get("scale", 1.0)
    if source_value >= 0.0:
        scale = mapping.get("scale_positive", fallback_scale)
    else:
        scale = mapping.get("scale_negative", fallback_scale)
    return source_value * scale + mapping.get("offset", 0.0)
