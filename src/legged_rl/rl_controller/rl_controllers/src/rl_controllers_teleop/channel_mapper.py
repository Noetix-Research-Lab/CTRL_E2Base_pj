"""Convert raw receiver channels to a virtual joystick state."""


DEFAULT_AXIS_CONFIG = [
    {"channel": 0, "minimum": 282, "center": 1002, "maximum": 1722},
    {"channel": 1, "minimum": 282, "center": 1002, "maximum": 1722, "invert": True},
    {"channel": 2, "minimum": 282, "center": 1002, "maximum": 1722},
    {"channel": 3, "minimum": 282, "center": 1002, "maximum": 1722},
]


def _clamp(value, lower=-1.0, upper=1.0):
    return max(lower, min(upper, value))


class ChannelMapper:
    """Apply YAML-defined axis calibration and virtual-button mappings."""

    def __init__(self, button_mappings=None, axis_config=None, minimum_buttons=14):
        self.button_mappings = button_mappings if isinstance(button_mappings, dict) else {}
        self.axis_config = axis_config if isinstance(axis_config, list) else DEFAULT_AXIS_CONFIG
        button_ids = []
        for raw_id in self.button_mappings:
            try:
                button_ids.append(int(raw_id))
            except (TypeError, ValueError):
                continue
        self.button_count = max([minimum_buttons - 1] + button_ids) + 1

    def map(self, channels):
        return {
            "axes": [self._map_axis(channels, spec) for spec in self.axis_config],
            "button": self._map_buttons(channels),
        }

    @staticmethod
    def _map_axis(channels, spec):
        channel = int(spec.get("channel", 0))
        if channel < 0 or channel >= len(channels):
            return 0.0
        minimum = float(spec.get("minimum", 282.0))
        center = float(spec.get("center", 1002.0))
        maximum = float(spec.get("maximum", 1722.0))
        deadzone = max(0.0, float(spec.get("deadzone", 0.0)))
        value = float(channels[channel])
        if abs(value - center) <= deadzone:
            result = 0.0
        elif value >= center:
            span = maximum - center
            result = 0.0 if span == 0 else (value - center) / span
        else:
            span = center - minimum
            result = 0.0 if span == 0 else (value - center) / span
        if spec.get("invert", False):
            result = -result
        return _clamp(result)

    def _map_buttons(self, channels):
        buttons = [0] * self.button_count
        for raw_id, mapping in self.button_mappings.items():
            if not isinstance(mapping, dict):
                continue
            try:
                button_id = int(raw_id)
                channel = int(mapping["channel"])
            except (KeyError, TypeError, ValueError):
                continue
            if button_id < 0 or button_id >= len(buttons) or channel < 0 or channel >= len(channels):
                continue
            value = channels[channel]
            condition = mapping.get("condition", "eq")
            if condition == "eq":
                target = mapping.get("value")
                tolerance = float(mapping.get("tolerance", 0))
                pressed = target is not None and abs(value - float(target)) <= tolerance
            elif condition == "high":
                pressed = value > float(mapping.get("threshold_high", 1200))
            elif condition == "low":
                pressed = value < float(mapping.get("threshold_low", 800))
            elif condition == "mid":
                pressed = float(mapping.get("threshold_mid_low", 800)) <= value <= float(
                    mapping.get("threshold_mid_high", 1200)
                )
            elif condition == "range":
                pressed = float(mapping.get("range_low", 0)) <= value <= float(
                    mapping.get("range_high", 2047)
                )
            else:
                pressed = False
            if pressed:
                buttons[button_id] = 1
        return buttons
