import json
import unittest
from unittest.mock import MagicMock, patch

import main


def mode(pulse, **values):
    pulse.receive({"type": "DriverStation", "data": values})


def enable(pulse):
    mode(pulse, **{">enabled": True, ">autonomous": False,
                   ">test": False, ">estop": False})


class PulseTests(unittest.TestCase):
    def test_waits_for_complete_mode_and_runs_once(self):
        pulse = main.JoystickPulse()
        self.assertIsNone(pulse.next_input(0))
        mode(pulse, **{">enabled": True})
        self.assertIsNone(pulse.next_input(1))
        mode(pulse, **{">autonomous": False, ">test": False, ">estop": False})
        self.assertEqual(pulse.next_input(10), 0.5)
        self.assertEqual(pulse.next_input(11.999), 0.5)
        self.assertEqual(pulse.next_input(12), 0.0)
        mode(pulse, **{">enabled": False})
        enable(pulse)
        self.assertIsNone(pulse.next_input(20))

    def test_leaving_teleop_ends_without_replay(self):
        for change in ({">enabled": False}, {">autonomous": True},
                       {">test": True}, {">estop": True}):
            with self.subTest(change=change):
                pulse = main.JoystickPulse()
                enable(pulse)
                self.assertEqual(pulse.next_input(0), 0.5)
                mode(pulse, **change)
                self.assertEqual(pulse.next_input(0.5), 0.0)
                enable(pulse)
                self.assertIsNone(pulse.next_input(1))

    def test_wire_messages_only_control_joystick_and_notify(self):
        websocket = MagicMock()
        main.send_joystick(websocket, 0.5)
        joystick, notification = [json.loads(c.args[0])
                                  for c in websocket.send.call_args_list]
        self.assertEqual(joystick["device"], "0")
        self.assertEqual(joystick["data"][">axes"], [0, 0.5, 0, 0, 0, 0])
        self.assertEqual(joystick["data"][">buttons"], [False] * 10)
        self.assertEqual(joystick["data"][">povs"], [-1])
        self.assertEqual(notification, {"type": "DriverStation", "device": "",
                                       "data": {">new_data": True}})

    @patch("main.ntcore.NetworkTableInstance")
    @patch("main.connect")
    def test_websocket_failure_cleans_up(self, connect, nt):
        connect.side_effect = OSError("connection refused")
        self.assertEqual(main.main(), 1)
        nt.create.return_value.stopClient.assert_called_once()
        nt.destroy.assert_called_once()

    @patch("main.ntcore.NetworkTableInstance")
    @patch("main.connect")
    @patch("main.run_client", side_effect=KeyboardInterrupt)
    def test_interrupt_sends_neutral_and_closes(self, run, connect, nt):
        self.assertEqual(main.main(), 0)
        websocket = connect.return_value
        message = json.loads(websocket.send.call_args_list[0].args[0])
        self.assertEqual(message["data"][">axes"], [0] * 6)
        websocket.close.assert_called_once()
        nt.destroy.assert_called_once()


if __name__ == "__main__":
    unittest.main()
