# Simulation position client

From the repository root, run:

```sh
cd scripts/simcontroller
uv run main.py
```

uv installs the locked dependencies automatically. Python 3.14 or newer is required.

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

The server address, topic, and timing settings are constants in `main.py`.
