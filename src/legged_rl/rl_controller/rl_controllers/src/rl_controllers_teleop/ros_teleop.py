"""ROS integration for the modular SBUS teleoperation stack."""

import importlib
import threading

import actionlib
import genpy.message
import rospy
import rosservice
import rostopic
from rosservice import ROSServiceException

from .axis_mapping import mapped_axis_value
from .channel_mapper import ChannelMapper
from .command_dispatcher import CommandDispatcher
from .sbus_driver import SbusDriver


class TeleopError(RuntimeError):
    pass


class AsyncServiceProxy:
    def __init__(self, name, service_class, persistent):
        try:
            rospy.wait_for_service(name, timeout=2.0)
        except rospy.ROSException:
            raise TeleopError("service {} is unavailable".format(name))
        self._proxy = rospy.ServiceProxy(name, service_class, persistent)
        self._thread = None

    def __call__(self, request):
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(0.01)
            if self._thread.is_alive():
                return False
        self._thread = threading.Thread(target=self._proxy, args=[request])
        self._thread.daemon = True
        self._thread.start()
        return True


class SbusTeleopNode:
    def __init__(self):
        teleop_config = rospy.get_param("teleop", None)
        if not isinstance(teleop_config, dict):
            raise TeleopError("missing or invalid 'teleop' configuration")

        self.commands = self._normalize_commands(teleop_config)
        self.dispatcher = CommandDispatcher(self.commands)
        self.publishers = {}
        self.action_clients = {}
        self.service_clients = {}
        self.message_types = {}
        self.service_types = {}
        self.offline_actions = set()
        self.offline_services = set()
        self._input_initialized = False
        self._input_valid = False

        sbus_config = rospy.get_param("sbus", {})
        serial_config = dict(sbus_config.get("serial", {}))
        # Preserve the historical top-level sbus_port override.
        serial_config["port"] = rospy.get_param("~sbus_port", rospy.get_param("sbus_port", serial_config.get("port", "/dev/ttyUSB0")))
        mappings = rospy.get_param("~channel_mappings", rospy.get_param("channel_mappings", {}))
        axes = sbus_config.get("axes")
        mapper = ChannelMapper(mappings, axes)
        self.driver = SbusDriver(serial_config, mapper, self._log, rospy.is_shutdown)

        for name, command in self.commands.items():
            if command["type"] == "topic":
                self._register_topic(name, command)
            elif command["type"] == "action":
                self._register_action(name, command)
            elif command["type"] == "service":
                self._register_service(name, command)
            else:
                raise TeleopError("unknown command type '{}' for '{}'".format(command["type"], name))

        # Complete all state initialization before either worker can call into this object.
        self.driver.start()
        self.timer = rospy.Timer(rospy.Duration(0.02), self._tick)
        self.action_timer = rospy.Timer(rospy.Duration(2.0), self._update_actions)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Modular SBUS teleop started with %d virtual buttons", mapper.button_count)

    @staticmethod
    def _log(level, message):
        functions = {
            "info": rospy.loginfo,
            "warn": rospy.logwarn,
            "error": rospy.logerr,
            "warn_throttle": lambda text: rospy.logwarn_throttle(1.0, text),
            "error_throttle": lambda text: rospy.logerr_throttle(1.0, text),
        }
        functions.get(level, rospy.loginfo)(message)

    @staticmethod
    def _normalize_commands(config):
        commands = {}
        for name, raw_command in config.items():
            command = dict(raw_command)
            command_type = command.get("type")
            if command_type == "topic":
                command["buttons"] = list(command.get("deadman_buttons", []))
            elif command_type == "action":
                command.setdefault("buttons", [])
                command.setdefault("action_goal", {})
            elif command_type == "service":
                command.setdefault("buttons", [])
                command.setdefault("service_request", {})
            commands[name] = command
        return commands

    def _tick(self, _event):
        state = self.driver.snapshot()
        if not state["signal_valid"]:
            # Stop continuous motion immediately, but do not feed synthetic zero
            # buttons into the edge detector. Otherwise every held switch becomes
            # a new rising edge when the receiver recovers.
            for name, command in self.commands.items():
                if self.dispatcher.is_continuous(command):
                    self._publish_topic(command, self._zero_state(state))
            self._input_valid = False
            return
        if not self._input_initialized:
            self.dispatcher.synchronize(state["button"])
            self._input_initialized = True
            self._input_valid = True
            rospy.loginfo("First valid SBUS frame recorded; discrete commands are now armed")
            return
        if not self._input_valid:
            self.dispatcher.synchronize(state["button"])
            self._input_valid = True
            rospy.loginfo("SBUS signal recovered; switch positions resynchronized")
            return
        selected = self.dispatcher.commands_to_run(state["button"])
        for name in selected:
            try:
                self._run_command(name, state)
            except Exception as exc:
                rospy.logerr_throttle(1.0, "Teleop command '%s' failed: %s", name, exc)
        # A released deadman must actively overwrite the last non-zero velocity.
        for name, command in self.commands.items():
            if self.dispatcher.is_continuous(command) and name not in selected:
                self._publish_topic(command, self._zero_state(state))

    @staticmethod
    def _zero_state(state):
        return {
            "axes": [0.0] * len(state["axes"]),
            "button": [0] * len(state["button"]),
            "has_received_frame": state.get("has_received_frame", False),
            "signal_valid": state.get("signal_valid", False),
        }

    def _register_topic(self, name, command):
        try:
            message_type = self._message_type(command["message_type"])
            self.publishers[command["topic_name"]] = rospy.Publisher(
                command["topic_name"], message_type, queue_size=1
            )
        except (KeyError, TeleopError) as exc:
            raise TeleopError("cannot register topic command '{}': {}".format(name, exc))

    def _register_action(self, name, command):
        action_name = command["action_name"]
        try:
            self.action_clients[action_name] = actionlib.SimpleActionClient(
                action_name, self._message_type(self._action_type(action_name))
            )
            self.offline_actions.discard(action_name)
        except TeleopError:
            self.offline_actions.add(action_name)

    def _register_service(self, name, command):
        service_name = command["service_name"]
        try:
            self.service_clients[service_name] = AsyncServiceProxy(
                service_name,
                self._service_type(service_name),
                command.get("service_persistent", False),
            )
            self.offline_services.discard(service_name)
        except TeleopError:
            self.offline_services.add(service_name)

    def _run_command(self, name, state):
        command = self.commands[name]
        if command["type"] == "topic":
            self._publish_topic(command, state)
        elif command["type"] == "action":
            self._run_action(name, command)
        elif command["type"] == "service":
            self._run_service(name, command)

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
                    rospy.logerr_throttle(1.0, "Teleop input index %s is out of range", index)
                    continue
                value = mapped_axis_value(source[index], mapping)
                self._set_member(message, mapping["target"], value)
        if hasattr(message, "header"):
            message.header.stamp = rospy.Time.now()
        self.publishers[command["topic_name"]].publish(message)

    def _run_action(self, name, command):
        action_name = command["action_name"]
        if action_name in self.offline_actions:
            self._register_action(name, command)
            return
        goal_type = self._action_type(action_name)[:-6] + "Goal"
        goal = self._message_type(goal_type)()
        genpy.message.fill_message_args(goal, [command["action_goal"]])
        self.action_clients[action_name].send_goal(goal)

    def _run_service(self, name, command):
        service_name = command["service_name"]
        if service_name in self.offline_services:
            self._register_service(name, command)
            return
        request = self._service_type(service_name)._request_class()
        genpy.message.fill_message_args(request, [command["service_request"]])
        if not self.service_clients[service_name](request):
            rospy.loginfo("Previous request for command '%s' is still running", name)

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
                raise TeleopError("cannot load message type '{}': {}".format(type_name, exc))
        return self.message_types[type_name]

    @staticmethod
    def _action_type(action_name):
        result = rostopic._get_topic_type(rospy.resolve_name(action_name) + "/goal")[0]
        if result is None:
            raise TeleopError("cannot find action '{}'".format(action_name))
        return result[:-4]

    def _service_type(self, service_name):
        if service_name not in self.service_types:
            try:
                service_type = rosservice.get_service_class_by_name(service_name)
            except ROSServiceException as exc:
                raise TeleopError("cannot load service '{}': {}".format(service_name, exc))
            if service_type is None:
                raise TeleopError("cannot load service '{}'".format(service_name))
            self.service_types[service_name] = service_type
        return self.service_types[service_name]

    def _update_actions(self, _event=None):
        for name, command in self.commands.items():
            if command["type"] == "action" and command["action_name"] in self.offline_actions:
                self._register_action(name, command)

    def shutdown(self):
        self.driver.close()


def main():
    rospy.init_node("sbus_teleop")
    try:
        SbusTeleopNode()
        rospy.spin()
    except TeleopError as exc:
        rospy.logfatal("SBUS teleop startup failed: %s", exc)
