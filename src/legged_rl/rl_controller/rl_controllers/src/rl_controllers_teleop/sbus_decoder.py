"""Pure SBUS frame decoding, independent from ROS and serial I/O."""

SBUS_FRAME_LEN = 25
SBUS_HEADER = 0x0F
SBUS_FOOTERS = (0x00, 0x04, 0x14, 0x24, 0x34)
SBUS_FLAG_FRAME_LOST = 1 << 2
SBUS_FLAG_FAILSAFE = 1 << 3


class SbusFrameError(ValueError):
    """Raised when an SBUS frame is malformed or unsafe to consume."""


def decode_sbus_frame(frame):
    """Validate a 25-byte SBUS frame and return its 16 analog channels."""
    if len(frame) != SBUS_FRAME_LEN:
        raise SbusFrameError("expected 25 bytes, got {}".format(len(frame)))
    if frame[0] != SBUS_HEADER:
        raise SbusFrameError("invalid header 0x{:02X}".format(frame[0]))
    if frame[24] not in SBUS_FOOTERS:
        raise SbusFrameError("invalid footer 0x{:02X}".format(frame[24]))

    flags = frame[23]
    if flags & SBUS_FLAG_FAILSAFE:
        raise SbusFrameError("receiver failsafe flag set")
    if flags & SBUS_FLAG_FRAME_LOST:
        raise SbusFrameError("receiver frame-lost flag set")

    b = frame
    return [
        (b[1] | b[2] << 8) & 0x07FF,
        (b[2] >> 3 | b[3] << 5) & 0x07FF,
        (b[3] >> 6 | b[4] << 2 | b[5] << 10) & 0x07FF,
        (b[5] >> 1 | b[6] << 7) & 0x07FF,
        (b[6] >> 4 | b[7] << 4) & 0x07FF,
        (b[7] >> 7 | b[8] << 1 | b[9] << 9) & 0x07FF,
        (b[9] >> 2 | b[10] << 6) & 0x07FF,
        (b[10] >> 5 | b[11] << 3) & 0x07FF,
        (b[12] | b[13] << 8) & 0x07FF,
        (b[13] >> 3 | b[14] << 5) & 0x07FF,
        (b[14] >> 6 | b[15] << 2 | b[16] << 10) & 0x07FF,
        (b[16] >> 1 | b[17] << 7) & 0x07FF,
        (b[17] >> 4 | b[18] << 4) & 0x07FF,
        (b[18] >> 7 | b[19] << 1 | b[20] << 9) & 0x07FF,
        (b[20] >> 2 | b[21] << 6) & 0x07FF,
        (b[21] >> 5 | b[22] << 3) & 0x07FF,
    ]
