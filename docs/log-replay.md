# Replaying a WPILib Log in Simulation

This guide walks you through replaying a real match log (a `.wpilog` file) on
your laptop using the robot simulator. While replay is running, the simulator
pretends to be a real robot: it feeds the recorded Limelight data and
drivetrain state from the log into the same NetworkTables that the vision
subsystem normally reads from. That means you can test changes to vision or
localization code against real match data, and get the **same** result every
time you run it.

You don't need a physical robot, a Limelight, or the field. Just your laptop,
the robot code, and a log file.

---

## What you need before you start

1. **The robot project already set up on your laptop.** If you can run
   `./gradlew simulateJava` and the sim GUI opens, you're good.
2. **A `.wpilog` file** to replay. WPILib writes these automatically whenever
   the robot runs (real or simulated). You can either:
   - Copy a `.wpilog` from a teammate or grab one off the roboRIO with a USB
     stick, or
   - Generate one yourself by running the robot or simulator for a bit — WPILib
     creates the log file on its own.

   Drop the file anywhere on your laptop and remember the path. A common spot
   is a local `logs/` folder in the project root (that folder is gitignored,
   so files you put there stay on your machine).
3. **A terminal** open at the project root (the folder that contains
   `build.gradle` and `gradlew`).

> **Tip:** If you've never opened the sim GUI before, try running
> `./gradlew simulateJava` once *without* replay first, just to make sure
> simulation works on your machine.

---

## Step 1: Pick a log file

Open the [logs/](../logs/) folder and pick a `.wpilog` file. Copy its path
relative to the project root. For example:

```
logs/FRC_20260402_185809.wpilog
```

---

## Step 2: Start simulation with replay enabled

In your terminal at the project root, run:

**Windows (PowerShell):**

```powershell
.\gradlew simulateJava "-PlogreplayFile=logs/FRC_20260402_185809.wpilog"
```

**macOS / Linux:**

```bash
./gradlew simulateJava -PlogreplayFile=logs/FRC_20260402_185809.wpilog
```

Replace the log path with the file you picked in Step 1.

That `-PlogreplayFile=...` is the only thing that turns replay on. With no
`-PlogreplayFile`, the robot simulates normally and replay is completely off.

### What you should see in the terminal

After the build finishes, the simulator starts and you should see lines like:

```
[LogReplay] Log replay enabled with file: logs/FRC_20260402_185809.wpilog
[LogReplay] Loading log file: logs/FRC_20260402_185809.wpilog
[LogReplay] Indexed 12345 replayable data records.
[LogReplay] Breakdown by source:
[LogReplay]   limelight-a: 4567 records
[LogReplay]   limelight-b: 4321 records
[LogReplay]   DriveState: 3456 records
[LogReplay] Time span: 135.0 seconds
+==============================================================+
|  LOG REPLAY MODE ACTIVE                                      |
|  Simulator time is paused before first data point.           |
|  Use the sim GUI timing controls to resume/step.             |
+==============================================================+
```

If you see those lines, replay is loaded and waiting for you to press play.

> **If you see `[LogReplay] No entries to replay.`**, the log file doesn't
> contain Limelight or DriveState data. Pick a different log.
>
> **If you see `[LogReplay] Log file is not valid`**, the path is wrong or
> the file is corrupted. Double-check the path you typed.

---

## Step 3: Start playback in the sim GUI

The simulator deliberately starts with time **paused** so you have a moment to
arrange windows before data starts flowing.

1. In the sim GUI window, find the **Timing** panel (usually near the top).
2. Click **Resume** (the play button) to start the clock.
3. You can also click **Step** to advance one tick at a time, which is great
   for debugging.

As soon as time resumes, log entries are published into NetworkTables at the
same pace they were recorded. Your `VisionSubsystem` and pose estimator will
react exactly as if a real Limelight were sending data.

When playback finishes you'll see:

```
[LogReplay] Replay complete. 12345 records published.
```

The simulator keeps running after that — close the sim GUI window or press
`Ctrl+C` in the terminal when you're done.

---

## Step 4: Watch what's happening

Two easy ways to see the replayed data:

- **In the sim GUI**, open the **NetworkTables** panel and expand
  `limelight-a`, `limelight-b`, `limelight-c`, and `DriveState`. The values
  update as the log plays.
- **In AdvantageScope** (recommended for visualizing pose), connect to the
  simulator (`localhost`). The robot pose, vision targets, and module states
  all show up as if it were a live match.

---

## Inspecting a log without running the robot

If you just want to see what's inside a `.wpilog` (which entries exist, how
many records, etc.), run:

```powershell
.\gradlew logDiagnostic "-PlogreplayFile=logs/FRC_20260402_185809.wpilog"
```

This prints a table of every entry in the log, plus a focused view of just
the Limelight and DriveState entries. Useful when you're not sure if a log
has the data you need.

---

## Tips & troubleshooting

- **Use the same log every time** for a clean A/B test of vision code. The
  log replays identically on every run, so any differences you see in robot
  behavior come from your code changes — not from random sim noise.
- **Paths with spaces** need quotes around the whole `-P` argument:
  `"-PlogreplayFile=C:/My Logs/match1.wpilog"`.
- **Replay only covers Limelight and DriveState data.** Other things (motor
  feedback, controller input, etc.) are *not* replayed — they come from the
  normal simulator. That's intentional: replay is meant to test vision and
  localization, not to re-drive the whole match.
- **You can pause anytime** by clicking Pause in the sim GUI Timing panel.
  Replay pauses with the clock, so you can stop and look at the current
  state, then resume.
- **Replay is off by default.** If you forget the `-PlogreplayFile=...` flag,
  the simulator runs normally with the synthetic vision sim instead. That's
  the right behavior — replay should be an opt-in.

---

## How it works (short version, for the curious)

1. Gradle sees `-PlogreplayFile=...` and sets an environment variable
   `LOGREPLAY_FILE` for the simulator process.
2. When `Robot` starts up in simulation, it checks that variable and creates
   a [LogReplayManager](../src/main/java/frc/robot/util/simulation/LogReplayManager.java)
   that reads the log file.
3. In `simulationInit()`, the manager pauses the sim clock and arms a
   [ReplaySwerveDriveState](../src/main/java/frc/robot/util/simulation/ReplaySwerveDriveState.java)
   that takes over the drivetrain's pose estimator.
4. Every robot loop, `updateReplay()` publishes any log entries whose
   timestamps have caught up to the current FPGA time, into the same
   `limelight-a` / `limelight-b` / `limelight-c` / `DriveState` NetworkTables
   keys the rest of the robot code already listens to.

That's it — the rest of the robot code doesn't need to know replay is
happening.
