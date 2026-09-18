# Simulation joystick and position client

From the repository root, run:

```sh
cd scripts/simcontroller
uv run main.py
```

uv installs the locked dependencies automatically. Python 3.14 or newer is required.

Restart the Java simulation after updating `build.gradle` so that the HALSim
WebSocket server extension is loaded. The script also connects to
`ws://127.0.0.1:3300/wpilibws`. Enable teleop manually in the simulation GUI.
Once both connections are ready and enabled teleop is confirmed, the script
pushes controller port 0's left Y axis to -0.5 for two seconds, then returns all
controls to neutral. This pulse happens only once per script run. Disabling or
leaving teleop early ends the pulse without resuming it. Position reporting
continues afterward. If teleop is already enabled, the pulse starts upon connection
and mode confirmation. The script never enables the robot or changes its mode.

Do not map a GUI joystick to port 0 while using this script. Do not connect an
external Driver Station via the DS socket: WPILib ignores WebSocket joystick
input while that connection is active. Driving remains field-centric, as in the
Java robot code.

The script connects as an NT4 client to `127.0.0.1:5810` and subscribes to
`/DriveState/Pose`, the estimated `Pose2d` published by the Java robot's
`DriveStateNtLogger`. It prints x/y in meters every second:

```text
Estimated position: x=1.234 m, y=5.678 m
```

If the server does not connect within five seconds, the script prints an error
and exits with status 1. Losing an established connection also prints an error
and exits with status 1. While connected without a received pose, it prints a
waiting message every second. Ctrl+C stops the client cleanly.

WebSocket connection attempts time out after five seconds. WebSocket errors or
disconnects also exit with status 1. On shutdown, the script attempts to send
neutral input; a broken connection prevents guaranteed delivery of that message.

Run the automated checks with `uv run python -m unittest discover -s tests`.

The server address, topic, and timing settings are constants in `main.py`.
