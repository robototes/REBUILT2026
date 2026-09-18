"""Run one simulated joystick pulse and report the robot's estimated position."""

import json
import time

import ntcore
from wpimath.geometry import Pose2d
from websockets.exceptions import WebSocketException
from websockets.sync.client import connect

SERVER_HOST = "127.0.0.1"
SERVER_PORT = 5810
POSE_TOPIC = "/DriveState/Pose"
REPORT_INTERVAL = 1.0
CONNECTION_TIMEOUT = 5.0
POLL_INTERVAL = 0.1
WEBSOCKET_URL = "ws://127.0.0.1:3300/wpilibws"
JOYSTICK_INTERVAL = 0.02
FORWARD_INPUT = -0.5
PULSE_DURATION = 2.0


class JoystickPulse:
    """One pulse per process, with mode updates merged from HALSim messages."""

    def __init__(self):
        self.driver_station = {}
        self.started_at = None
        self.complete = False

    def receive(self, message):
        if message.get("type") == "DriverStation":
            self.driver_station.update(message.get("data", {}))

    def next_input(self, now):
        if self.complete:
            return None
        ds = self.driver_station
        teleop = (
            ds.get(">enabled") is True
            and ds.get(">autonomous") is False
            and ds.get(">test") is False
            and ds.get(">estop") is False
        )
        if self.started_at is None:
            if not teleop:
                return None
            self.started_at = now
            print("Moving joystick halfway forward for two seconds.", flush=True)
        if not teleop or now - self.started_at >= PULSE_DURATION:
            self.complete = True
            print("Joystick neutral; pulse complete.", flush=True)
            return 0.0
        return FORWARD_INPUT


def send_joystick(websocket, forward):
    websocket.send(json.dumps({
        "type": "Joystick",
        "device": "0",
        "data": {
            ">axes": [0.0, forward, 0.0, 0.0, 0.0, 0.0],
            ">buttons": [False] * 10,
            ">povs": [-1],
        },
    }))
    websocket.send(json.dumps({
        "type": "DriverStation", "device": "", "data": {">new_data": True},
    }))


def run_client(instance, subscriber, websocket):
    pulse = JoystickPulse()
    next_tick = time.monotonic()
    next_report = next_tick + REPORT_INTERVAL
    print("Waiting for manually enabled teleop.", flush=True)
    while True:
        if not instance.isConnected():
            print("NetworkTables connection lost.", flush=True)
            return 1

        now = time.monotonic()
        if now >= next_tick:
            forward = pulse.next_input(now)
            if forward is not None:
                send_joystick(websocket, forward)
            next_tick = now + JOYSTICK_INTERVAL

        if now >= next_report:
            sample = subscriber.getAtomic()
            if sample.time == 0:
                print(f"Waiting for robot pose on {POSE_TOPIC}.", flush=True)
            else:
                pose = sample.value
                print(
                    f"Estimated position: x={pose.X():.3f} m, y={pose.Y():.3f} m",
                    flush=True,
                )
            next_report = now + REPORT_INTERVAL

        try:
            message = websocket.recv(timeout=max(0.0, next_tick - time.monotonic()))
        except TimeoutError:
            continue
        pulse.receive(json.loads(message))
        # Apply mode changes immediately instead of waiting for the next tick.
        if pulse.started_at is not None and not pulse.complete:
            forward = pulse.next_input(time.monotonic())
            if forward == 0.0:
                send_joystick(websocket, forward)


def main() -> int:
    instance = ntcore.NetworkTableInstance.create()
    subscriber = None
    websocket = None
    try:
        subscriber = instance.getStructTopic(POSE_TOPIC, Pose2d).subscribe(Pose2d())
        instance.setServer(SERVER_HOST, SERVER_PORT)
        instance.startClient4("simcontroller")

        deadline = time.monotonic() + CONNECTION_TIMEOUT
        while not instance.isConnected():
            if time.monotonic() >= deadline:
                print(
                    f"No NetworkTables server available at {SERVER_HOST}:{SERVER_PORT}.",
                    flush=True,
                )
                return 1
            time.sleep(POLL_INTERVAL)

        websocket = connect(
            WEBSOCKET_URL, open_timeout=CONNECTION_TIMEOUT, close_timeout=1,
            proxy=None,
        )
        return run_client(instance, subscriber, websocket)
    except (OSError, TimeoutError, WebSocketException, ValueError) as error:
        print(f"HALSim WebSocket error at {WEBSOCKET_URL}: {error}", flush=True)
        return 1
    except KeyboardInterrupt:
        return 0
    finally:
        if websocket is not None:
            try:
                send_joystick(websocket, 0.0)
            except (OSError, TimeoutError, WebSocketException):
                pass  # A disconnected socket cannot deliver the neutral message.
            finally:
                websocket.close()
        if subscriber is not None:
            subscriber.close()
        instance.stopClient()
        ntcore.NetworkTableInstance.destroy(instance)


if __name__ == "__main__":
    raise SystemExit(main())
