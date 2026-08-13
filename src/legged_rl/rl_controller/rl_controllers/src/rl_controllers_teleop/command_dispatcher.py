"""Pure command scheduling for continuous and edge-triggered teleop commands."""


class CommandDispatcher:
    """Select commands to run without depending on ROS message types."""

    def __init__(self, commands):
        self.commands = commands
        self._initialized = False
        self._matched = {}
        self._last_buttons = []

    @staticmethod
    def matches(command, buttons):
        required = command.get("buttons", command.get("deadman_buttons", []))
        if any(button < 0 or button >= len(buttons) or buttons[button] != 1 for button in required):
            return False
        if command.get("allow_multiple_commands", True):
            return sum(buttons) >= len(required)
        return sum(buttons) == len(required)

    @staticmethod
    def is_continuous(command):
        return (
            command.get("type") == "topic"
            and "axis_mappings" in command
            and "message_value" not in command
        )

    def synchronize(self, buttons):
        """Record the current controls without emitting discrete command edges."""
        self._matched = {
            name: self.matches(command, buttons) for name, command in self.commands.items()
        }
        self._last_buttons = list(buttons)
        self._initialized = True

    def commands_to_run(self, buttons):
        """Return command names; empty-button commands run every call, others on rising edge."""
        if not self._initialized:
            self.synchronize(buttons)
            return []

        changed = {
            index
            for index in range(max(len(buttons), len(self._last_buttons)))
            if (buttons[index] if index < len(buttons) else 0)
            != (self._last_buttons[index] if index < len(self._last_buttons) else 0)
        }
        selected = []
        for name, command in self.commands.items():
            required = command.get("buttons", command.get("deadman_buttons", []))
            matched = self.matches(command, buttons)
            if self.is_continuous(command):
                if matched:
                    selected.append(name)
                self._matched[name] = matched
                continue
            # Discrete commands require an explicit button chord.
            if not required:
                continue
            if not changed.intersection(required):
                continue
            if matched and not self._matched.get(name, False):
                selected.append(name)
            self._matched[name] = matched
        self._last_buttons = list(buttons)
        return selected
