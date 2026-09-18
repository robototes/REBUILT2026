"""Print the robot's estimated position from the local NetworkTables server."""

import time

import ntcore
from wpimath.geometry import Pose2d

SERVER_HOST = "127.0.0.1"
SERVER_PORT = 5810
POSE_TOPIC = "/DriveState/Pose"
REPORT_INTERVAL = 1.0
CONNECTION_TIMEOUT = 5.0
POLL_INTERVAL = 0.1


def main() -> int:
    instance = ntcore.NetworkTableInstance.create()
    subscriber = None
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

        next_report = time.monotonic() + REPORT_INTERVAL
        while True:
            if not instance.isConnected():
                print("NetworkTables connection lost.", flush=True)
                return 1

            if time.monotonic() >= next_report:
                sample = subscriber.getAtomic()
                if sample.time == 0:
                    print(f"Waiting for robot pose on {POSE_TOPIC}.", flush=True)
                else:
                    pose = sample.value
                    print(
                        f"Estimated position: x={pose.X():.3f} m, y={pose.Y():.3f} m",
                        flush=True,
                    )
                next_report += REPORT_INTERVAL
            time.sleep(POLL_INTERVAL)
    except KeyboardInterrupt:
        return 0
    finally:
        if subscriber is not None:
            subscriber.close()
        instance.stopClient()
        ntcore.NetworkTableInstance.destroy(instance)


if __name__ == "__main__":
    raise SystemExit(main())
