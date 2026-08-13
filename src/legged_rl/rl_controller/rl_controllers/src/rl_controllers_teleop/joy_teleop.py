"""Joy-to-topic teleoperation with direction-specific axis scaling."""

import importlib

import rospy
from sensor_msgs.msg import Joy

from .axis_mapping import mapped_axis_value
from .command_dispatcher import CommandDispatcher


class JoyTeleopError(RuntimeError):
    pass


class JoyTeleopNode:
    def __init__(self):
        teleop_config = rospy.get_param("teleop", None)
        if not isinstance(teleop_config, dict):
            raise JoyTeleopError("missing or invalid 'teleop' configuration")

        self.commands = self._normalize_commands(teleop_config)
        self.dispatcher = CommandDispatcher(self.commands)
        self.publishers = {}
        self.message_types = {}

        for name, command in self.commands.items():
            if command.get("type") != "topic":
                raise JoyTeleopError(
                    "Joy command '{}' has unsupported type '{}'".format(
                        name, command.get("type")
                    )
                )
            message_type = self._message_type(command["message_type"])
            self.publishers[command["topic_name"]] = rospy.Publisher(
                command["topic_name"], message_type, queue_size=1
            )

        self.subscriber = rospy.Subscriber("joy", Joy, self._joy_callback, queue_size=1)
        rospy.loginfo("Joy teleop started with direction-specific axis scaling")

    @staticmethod
    def _normalize_commands(config):
        commands = {}
        for name, raw_command in config.items():
            command = dict(raw_command)
            command["buttons"] = list(command.get("deadman_buttons", []))
            commands[name] = command
        return commands

    def _joy_callback(self, joy):
        state = {"axes": joy.axes, "button": joy.buttons}
        selected = self.dispatcher.commands_to_run(joy.buttons)
        for name in selected:
            self._publish_topic(self.commands[name], state)

        for name, command in self.commands.items():
            if self.dispatcher.is_continuous(command) and name not in selected:
                self._publish_topic(command, self._zero_state(state))

    @staticmethod
    def _zero_state(state):
        return {
            "axes": [0.0] * len(state["axes"]),
            "button": [0] * len(state["button"]),
        }

    def _publish_topic(self, command, state):
        message = self._message_type(command["message_type"])()
        if "message_value" in command:
            for value in command["message_value"]:
                self._set_member(message, value["target"], value["value"])
        else:
            for mapping in command.get("axis_mappings", []):
                if "axis" in mapping:
                    source = state["axes"]
                    index = mapping["axis"]
                elif "button" in mapping:
                    source = state["button"]
                    index = mapping["button"]
                else:
                    continue
                if index < 0 or index >= len(source):
                    rospy.logerr_throttle(1.0, "Joy input index %s is out of range", index)
                    continue
                value = mapped_axis_value(source[index], mapping)
                self._set_member(message, mapping["target"], value)
        if hasattr(message, "header"):
            message.header.stamp = rospy.Time.now()
        self.publishers[command["topic_name"]].publish(message)

    @staticmethod
    def _set_member(message, member, value):
        target = message
        path = member.split(".")
        for component in path[:-1]:
            target = getattr(target, component)
        setattr(target, path[-1], value)

    def _message_type(self, type_name):
        if type_name not in self.message_types:
            try:
                package, message = type_name.split("/")
                module = importlib.import_module(package + ".msg")
                self.message_types[type_name] = getattr(module, message)
            except (ValueError, ImportError, AttributeError) as exc:
                raise JoyTeleopError(
                    "cannot load message type '{}': {}".format(type_name, exc)
                )
        return self.message_types[type_name]


def main():
    rospy.init_node("joy_teleop")
    try:
        JoyTeleopNode()
        rospy.spin()
    except JoyTeleopError as exc:
        rospy.logfatal("Joy teleop startup failed: %s", exc)
        raise SystemExit(1)
