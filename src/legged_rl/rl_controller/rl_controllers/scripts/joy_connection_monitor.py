#!/usr/bin/env python3
"""Publish USB joystick connectivity from device presence plus /joy heartbeat."""

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

from rl_controllers_teleop.joy_connection import JoyConnectionDetector


class JoyConnectionMonitor:
    def __init__(self):
        device = rospy.get_param("~device", "/dev/input/js0")
        heartbeat_timeout = float(rospy.get_param("~heartbeat_timeout", 0.3))
        check_rate = float(rospy.get_param("~check_rate", 20.0))
        if check_rate <= 0.0:
            raise ValueError("check_rate must be positive")

        self.detector = JoyConnectionDetector(device, heartbeat_timeout)
        self.publisher = rospy.Publisher(
            "/joystick_connected", Bool, queue_size=1, latch=True
        )
        self.previous = None
        self.subscriber = rospy.Subscriber("/joy", Joy, self._joy_callback, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0 / check_rate), self._check)

    def _joy_callback(self, _message):
        self.detector.record_heartbeat()

    def _check(self, _event):
        connected = self.detector.connected()
        self.publisher.publish(Bool(data=connected))
        if connected != self.previous:
            if connected:
                rospy.loginfo("USB joystick connected and /joy heartbeat is healthy")
            else:
                rospy.logerr("USB joystick unavailable or /joy heartbeat timed out")
            self.previous = connected


if __name__ == "__main__":
    rospy.init_node("joy_connection_monitor")
    JoyConnectionMonitor()
    rospy.spin()
