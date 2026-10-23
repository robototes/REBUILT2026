# Main-loop timing

Open the AdvantageKit `.wpilog` in AdvantageScope, or connect live using the
existing NT4 publisher. Under `RealOutputs/LoopTiming` (or `ReplayOutputs` during
replay), all durations have an `MS` suffix and are in milliseconds:

| Field | Measures |
| --- | --- |
| `TotalMS`, `Overrun`, `OverrunCount` | One `loopFunc()` iteration, including timing collection/publication; overrun means longer than the configured robot period (20 ms). |
| `RobotEpochs/*` | WPILib's mode init/periodic, robotPeriodic, SmartDashboard, LiveWindow, Shuffleboard, and simulation timing slices. |
| `SchedulerEpochs/*` | Each registered subsystem's periodic, button polling, and top-level command initialize/execute/end timing slices. |
| `RobotPeriodic/*` | This robot's vision simulation/update, autonomous field dashboard, complete scheduler invocation, drive state, launcher NetworkTables, GC logging, and DataLog resume work. |
| `CollectionMS`, `PublishMS` | Additional timing collection and sample-publication overhead on the main thread. |
| `WatchdogEpochsAvailable` | Whether the WPILib watchdog integration is working. |

Graph `TotalMS` alongside `RobotPeriodic/SchedulerMS`, the largest subsystem and
command epochs, and the dashboard epochs to locate spikes. Values for previously
seen but inactive slices are explicitly zeroed each loop, including when the
scheduler is disabled. The first observation of a label creates its cached key;
keep command/subsystem names stable and give expensive top-level commands
distinct names.

Existing telemetry settings are unchanged: `Robot` still disables LiveWindow
telemetry. Its epoch measures the update call's actual cost, even when that call
only checks whether telemetry is disabled.

## Cost and interpretation

The collector reuses the timings WPILib 2026.2.1 already measures, using read-only
access to its private watchdog/tracer fields. It does not fork the scheduler,
wrap commands/subsystems, serialize protobufs, print per-loop text, or send a
command tree. Reflected fields and map references are resolved once at startup;
one cached boolean field is read each scheduler capture to avoid stale samples
when the scheduler is disabled. Log keys are cached by label. Numeric samples
are recorded each loop through the existing AdvantageKit logger, preserving
single-loop spikes rather than averaging them away.

Private fields are version-sensitive. If access or the epoch format becomes
incompatible after a WPILib upgrade, a Driver Station warning and
`WatchdogEpochsAvailable=false` explicitly identify the loss of epoch detail.
The explicit robotPeriodic slices, total, and overrun counter remain active.
Run `.\gradlew.bat test --tests frc.robot.util.LoopTimingTest` after an upgrade.

Watchdog epochs are **elapsed timing slices, not isolated CPU measurements**:

- A slice includes everything since the previous WPILib epoch, including
  scheduler bookkeeping and command logging callbacks. `isFinished()` time is
  included in the following slice, not reported separately.
- Subsystem `periodic()` includes `simulationPeriodic()` in simulation.
- Commands inside compositions are included in the top-level group's timing.
  Repeated identical epoch names in the same loop retain only the last slice,
  as in WPILib's watchdog output. Initializations/cancellations outside
  `CommandScheduler.run()` are not preserved as separate scheduler epochs.
- Parent timings include child work: do not add `robotPeriodic()` to its
  scheduler/subsystem/command slices, or add button polling to initialization
  slices that occur during polling.
- These are per-loop durations, not command scheduled lifetimes or per-run
  cumulative processing times. No command graph or unique run IDs are generated.

The FPGA clock is used directly, not AdvantageKit's frozen per-loop/replay
timestamp. Replay timings measure the replay computer, not the original robot.
`TotalMS` includes Driver Station refresh and the framework's local NetworkTables
flush, but excludes AdvantageKit's work outside `loopFunc()`. Compare it with the
existing `LoggedRobot/FullCycleMS`, `LoggedRobot/LogPeriodicMS`, `Logger/*MS`,
`Logger/QueuedCycles`, and `LoggedRobot/GCTimeMS` for the rest of the main-loop
cost. Receiver-thread/disk/network work is not a synchronous logging slice.

Check `CollectionMS` and `PublishMS` on the roboRIO to quantify observer overhead;
desktop timing is not a roboRIO performance guarantee. They exclude the few
final summary writes and subsequent AdvantageKit serialization/receiver work;
`LoggedRobot/LogPeriodicMS` helps expose that downstream cost. The startup/key
creation cycles are not representative of steady-state overhead.
