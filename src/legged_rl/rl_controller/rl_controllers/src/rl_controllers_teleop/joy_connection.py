"""USB joystick physical-presence and message-heartbeat checks."""

import os
import stat
import time


def joystick_device_exists(path):
    """Return true only when path resolves to a character device."""
    try:
        return stat.S_ISCHR(os.stat(path).st_mode)
    except OSError:
        return False


class JoyConnectionDetector:
    def __init__(self, device, heartbeat_timeout, device_checker=joystick_device_exists,
                 clock=time.monotonic):
        if heartbeat_timeout <= 0.0:
            raise ValueError("heartbeat_timeout must be positive")
        self.device = device
        self.heartbeat_timeout = float(heartbeat_timeout)
        self.device_checker = device_checker
        self.clock = clock
        self.last_heartbeat = None

    def record_heartbeat(self):
        self.last_heartbeat = self.clock()

    def connected(self):
        now = self.clock()
        heartbeat_fresh = (
            self.last_heartbeat is not None
            and now >= self.last_heartbeat
            and now - self.last_heartbeat <= self.heartbeat_timeout
        )
        return self.device_checker(self.device) and heartbeat_fresh
