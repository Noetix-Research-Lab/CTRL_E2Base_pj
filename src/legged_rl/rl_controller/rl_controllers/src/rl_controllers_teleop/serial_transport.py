"""Serial transport and hot-plug discovery for SBUS receivers."""

import glob
import os

import serial
import serial.tools.list_ports


PARITIES = {
    "none": serial.PARITY_NONE,
    "even": serial.PARITY_EVEN,
    "odd": serial.PARITY_ODD,
}


class SerialTransport:
    def __init__(self, config, logger):
        self.config = dict(config or {})
        self.logger = logger
        self.preferred_port = self.config.get("port", "/dev/ttyUSB0")
        self.port = None
        self.serial = None

    def _candidates(self):
        candidates = []
        if self.preferred_port and self.preferred_port != "auto":
            candidates.append(self.preferred_port)
        for path in ["/dev/ch340", "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyUSB3"]:
            if path not in candidates:
                candidates.append(path)
        try:
            for info in serial.tools.list_ports.comports():
                if info.device not in candidates and (
                    "1a86:7523" in info.hwid.lower() or "ch340" in info.description.lower()
                ):
                    candidates.append(info.device)
        except Exception as exc:
            self.logger("warn", "USB serial scan failed: {}".format(exc))
        for path in sorted(glob.glob("/dev/ttyUSB*")):
            if path not in candidates:
                candidates.append(path)
        return candidates

    def connect(self):
        self.close()
        for port in self._candidates():
            if not os.path.exists(port):
                continue
            try:
                self.serial = serial.Serial(
                    port=port,
                    baudrate=int(self.config.get("baudrate", 100000)),
                    bytesize=serial.EIGHTBITS,
                    parity=PARITIES.get(str(self.config.get("parity", "even")).lower(), serial.PARITY_EVEN),
                    stopbits=float(self.config.get("stopbits", 2)),
                    timeout=float(self.config.get("timeout", 0.01)),
                    write_timeout=0,
                )
                self.serial.reset_input_buffer()
                self.port = port
                self.logger(
                    "info",
                    "SBUS serial opened: {} @ {} 8{}{}".format(
                        port,
                        self.config.get("baudrate", 100000),
                        str(self.config.get("parity", "even"))[0].upper(),
                        self.config.get("stopbits", 2),
                    ),
                )
                return True
            except (serial.SerialException, OSError, ValueError) as exc:
                self.logger("warn", "Cannot open {}: {}".format(port, exc))
                self.close()
        return False

    @property
    def connected(self):
        return self.serial is not None and self.serial.is_open and self.port and os.path.exists(self.port)

    def read_byte(self):
        data = self.serial.read(1)
        return data[0] if data else None

    def close(self):
        if self.serial is not None:
            try:
                self.serial.close()
            except (serial.SerialException, OSError):
                pass
        self.serial = None
