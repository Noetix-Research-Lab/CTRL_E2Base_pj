"""Threaded SBUS receiver composed from transport, decoder, and mapper modules."""

import threading
import time

import serial

from .sbus_decoder import SBUS_FRAME_LEN, SBUS_HEADER, SbusFrameError, decode_sbus_frame
from .serial_transport import SerialTransport


class SbusDriver:
    def __init__(self, serial_config, mapper, logger, shutdown_requested):
        self.transport = SerialTransport(serial_config, logger)
        self.mapper = mapper
        self.logger = logger
        self.shutdown_requested = shutdown_requested
        self.signal_timeout = float(serial_config.get("signal_timeout", 0.2))
        self.reconnect_delay = float(serial_config.get("reconnect_delay", 1.0))
        self._state = {"axes": [0.0] * len(mapper.axis_config), "button": [0] * mapper.button_count}
        self._state_lock = threading.Lock()
        self._running = False
        self._thread = None
        self._last_valid_frame = 0.0
        self._safe = True
        self._has_received_frame = False
        self._signal_valid = False

    def start(self):
        # Missing hardware at process start is not fatal; the worker keeps probing for hot-plug.
        self.transport.connect()
        self._running = True
        self._thread = threading.Thread(target=self._run, name="sbus_reader")
        self._thread.daemon = True
        self._thread.start()

    def snapshot(self):
        with self._state_lock:
            return {
                "axes": list(self._state["axes"]),
                "button": list(self._state["button"]),
                "has_received_frame": self._has_received_frame,
                "signal_valid": self._signal_valid,
            }

    def _set_safe(self, reason):
        with self._state_lock:
            changed = any(abs(value) > 1e-6 for value in self._state["axes"]) or any(self._state["button"])
            was_valid = self._signal_valid
            self._state = {
                "axes": [0.0] * len(self.mapper.axis_config),
                "button": [0] * self.mapper.button_count,
            }
            self._signal_valid = False
        if changed or was_valid or not self._safe:
            self.logger("warn_throttle", "SBUS safe state: {}".format(reason))
        self._safe = True

    def _check_timeout(self):
        if self._last_valid_frame and time.monotonic() - self._last_valid_frame > self.signal_timeout:
            self._set_safe("no valid frame for {:.2f}s".format(time.monotonic() - self._last_valid_frame))

    def _accept_frame(self, frame):
        try:
            channels = decode_sbus_frame(frame)
        except SbusFrameError as exc:
            self._set_safe(str(exc))
            return
        state = self.mapper.map(channels)
        with self._state_lock:
            self._state = state
            self._has_received_frame = True
            self._signal_valid = True
        self._last_valid_frame = time.monotonic()
        self._safe = False

    def _run(self):
        frame = bytearray()
        last_reconnect = 0.0
        while self._running and not self.shutdown_requested():
            try:
                if not self.transport.connected:
                    self._set_safe("receiver disconnected")
                    now = time.monotonic()
                    if now - last_reconnect >= self.reconnect_delay:
                        last_reconnect = now
                        if not self.transport.connect():
                            self.logger("warn_throttle", "SBUS receiver unavailable; retrying")
                    time.sleep(0.05)
                    continue
                value = self.transport.read_byte()
                if value is None:
                    self._check_timeout()
                    continue
                if not frame and value != SBUS_HEADER:
                    self._check_timeout()
                    continue
                frame.append(value)
                if len(frame) == SBUS_FRAME_LEN:
                    self._accept_frame(frame)
                    frame = bytearray()
            except (serial.SerialException, OSError) as exc:
                self.logger("warn_throttle", "SBUS serial error: {}".format(exc))
                self.transport.close()
                frame = bytearray()
                self._check_timeout()
            except Exception as exc:
                self.logger("error_throttle", "SBUS reader error: {}".format(exc))
                frame = bytearray()
                self._check_timeout()
                time.sleep(0.05)

    def close(self):
        self._running = False
        self.transport.close()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
