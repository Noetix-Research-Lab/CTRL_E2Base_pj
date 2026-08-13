#!/usr/bin/env python3

import unittest

from rl_controllers_teleop.channel_mapper import ChannelMapper
from rl_controllers_teleop.axis_mapping import mapped_axis_value
from rl_controllers_teleop.command_dispatcher import CommandDispatcher
from rl_controllers_teleop.sbus_decoder import SbusFrameError, decode_sbus_frame
from rl_controllers_teleop.joy_connection import JoyConnectionDetector


def encode_frame(channels, flags=0, footer=0):
    payload = 0
    for index, value in enumerate(channels):
        payload |= (int(value) & 0x07FF) << (11 * index)
    frame = bytearray([0x0F])
    frame.extend(payload.to_bytes(22, byteorder="little"))
    frame.extend([flags, footer])
    return frame


class SbusDecoderTest(unittest.TestCase):
    def test_decodes_all_sixteen_channels(self):
        channels = [282, 1002, 1722, 0, 2047, 353, 1024, 1695] * 2
        self.assertEqual(decode_sbus_frame(encode_frame(channels)), channels)

    def test_rejects_failsafe_and_lost_frames(self):
        with self.assertRaises(SbusFrameError):
            decode_sbus_frame(encode_frame([1024] * 16, flags=1 << 3))
        with self.assertRaises(SbusFrameError):
            decode_sbus_frame(encode_frame([1024] * 16, flags=1 << 2))


class ChannelMapperTest(unittest.TestCase):
    def test_supports_legacy_calibration_and_eq_buttons(self):
        mapper = ChannelMapper(
            {"4": {"channel": 5, "condition": "eq", "value": 1722}},
            [
                {"channel": 0, "minimum": 282, "center": 1002, "maximum": 1722},
                {"channel": 1, "minimum": 282, "center": 1002, "maximum": 1722, "invert": True},
            ],
        )
        state = mapper.map([1722, 282, 1024, 1024, 1024, 1722] + [1024] * 10)
        self.assertEqual(state["axes"], [1.0, 1.0])
        self.assertEqual(state["button"][4], 1)

    def test_supports_wfly_threshold_buttons(self):
        mapper = ChannelMapper(
            {
                "17": {"channel": 10, "condition": "low", "threshold_low": 700},
                "18": {
                    "channel": 10,
                    "condition": "mid",
                    "threshold_mid_low": 950,
                    "threshold_mid_high": 1100,
                },
                "19": {"channel": 10, "condition": "high", "threshold_high": 1400},
            },
            [{"channel": 0, "minimum": 321, "center": 992, "maximum": 1663}],
        )
        channels = [992] * 16
        channels[10] = 1695
        state = mapper.map(channels)
        self.assertEqual(state["axes"], [0.0])
        self.assertEqual(state["button"][17:20], [0, 0, 1])


class AxisMappingTest(unittest.TestCase):
    def test_supports_independent_positive_and_negative_scales(self):
        mapping = {"scale_positive": 3.0, "scale_negative": 1.0}
        self.assertEqual(mapped_axis_value(1.0, mapping), 3.0)
        self.assertEqual(mapped_axis_value(-1.0, mapping), -1.0)
        self.assertEqual(mapped_axis_value(0.5, mapping), 1.5)
        self.assertEqual(mapped_axis_value(-0.5, mapping), -0.5)

    def test_legacy_scale_and_offset_remain_supported(self):
        mapping = {"scale": 2.0, "offset": 0.25}
        self.assertEqual(mapped_axis_value(0.5, mapping), 1.25)
        self.assertEqual(mapped_axis_value(-0.5, mapping), -0.75)


class CommandDispatcherTest(unittest.TestCase):
    def setUp(self):
        self.dispatcher = CommandDispatcher(
            {
                "walk": {"type": "topic", "buttons": [], "axis_mappings": []},
                "stand": {"type": "topic", "buttons": [2], "message_value": []},
            }
        )

    def test_initial_switch_position_does_not_trigger(self):
        self.assertEqual(self.dispatcher.commands_to_run([0, 0, 1]), [])

    def test_continuous_command_runs_every_tick(self):
        self.dispatcher.commands_to_run([0, 0, 0])
        self.assertEqual(self.dispatcher.commands_to_run([0, 0, 0]), ["walk"])
        self.assertEqual(self.dispatcher.commands_to_run([0, 0, 0]), ["walk"])

    def test_discrete_command_runs_only_on_rising_edge(self):
        self.dispatcher.commands_to_run([0, 0, 0])
        self.assertEqual(self.dispatcher.commands_to_run([0, 0, 1]), ["walk", "stand"])
        self.assertEqual(self.dispatcher.commands_to_run([0, 0, 1]), ["walk"])


    def test_continuous_deadman_is_selected_while_held(self):
        dispatcher = CommandDispatcher(
            {"walk": {"type": "topic", "buttons": [1], "axis_mappings": []}}
        )
        dispatcher.commands_to_run([0, 0])
        self.assertEqual(dispatcher.commands_to_run([0, 1]), ["walk"])
        self.assertEqual(dispatcher.commands_to_run([0, 1]), ["walk"])
        self.assertEqual(dispatcher.commands_to_run([0, 0]), [])

    def test_discrete_command_without_buttons_is_not_repeated(self):
        dispatcher = CommandDispatcher(
            {"unsafe": {"type": "topic", "buttons": [], "message_value": []}}
        )
        dispatcher.commands_to_run([])
        self.assertEqual(dispatcher.commands_to_run([]), [])

    def test_synchronize_after_recovery_does_not_emit_switch_edges(self):
        self.dispatcher.commands_to_run([0, 0, 0])
        self.dispatcher.synchronize([0, 0, 1])
        self.assertEqual(self.dispatcher.commands_to_run([0, 0, 1]), ["walk"])
        self.assertEqual(self.dispatcher.commands_to_run([0, 0, 0]), ["walk"])
        self.assertEqual(self.dispatcher.commands_to_run([0, 0, 1]), ["walk", "stand"])


class JoyConnectionDetectorTest(unittest.TestCase):
    def test_requires_device_and_fresh_heartbeat(self):
        now = [10.0]
        device_present = [True]
        detector = JoyConnectionDetector(
            "/dev/input/js0",
            0.3,
            device_checker=lambda _path: device_present[0],
            clock=lambda: now[0],
        )

        self.assertFalse(detector.connected())
        detector.record_heartbeat()
        self.assertTrue(detector.connected())

        device_present[0] = False
        self.assertFalse(detector.connected())
        device_present[0] = True
        now[0] += 0.31
        self.assertFalse(detector.connected())


if __name__ == "__main__":
    unittest.main()
