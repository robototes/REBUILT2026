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
| `Thread/WallMS`, `Thread/CpuMS`, `Thread/OffCpuMS` | Active-loop elapsed time, main-thread CPU time, and nonnegative wall minus CPU time. |
| `Thread/UserMS`, `Thread/SystemMS` | Main-thread user-mode CPU time and nonnegative total CPU minus user time. |
| `Thread/CpuTimeAvailable`, `Thread/UserTimeAvailable` | Whether the corresponding CPU counters were valid for this loop. |
| `GC/Collections`, `GC/CollectionTimeMS`, `GC/ActivityDetected` | JVM-wide collection-count/time deltas and whether either delta detected activity during the active loop. |
| `GC/CollectionCountAvailable`, `GC/CollectionTimeAvailable` | Whether all cached collectors supplied valid, monotonic counters for that measurement. |

Graph `TotalMS` alongside `RobotPeriodic/SchedulerMS`, the largest subsystem and
command epochs, and the dashboard epochs to locate spikes. Values for previously
seen but inactive slices are explicitly zeroed each loop, including when the
scheduler is disabled. The first observation of a label creates its cached key;
keep command/subsystem names stable and give expensive top-level commands
distinct names.

Existing telemetry settings are unchanged: `Robot` still disables LiveWindow
telemetry. Its epoch measures the update call's actual cost, even when that call
only checks whether telemetry is disabled.

## CPU versus off-CPU and GC

Graph `Thread/WallMS`, `Thread/CpuMS`, `Thread/OffCpuMS`, and
`GC/CollectionTimeMS` together. High wall **and** CPU suggests expensive computation;
high wall with low CPU suggests the main thread was not running. Off-CPU is
**not an I/O-wait measurement**: it also includes lock contention, sleeping,
stop-the-world GC, and OS scheduling/preemption. CPU used by other threads,
including receiver, odometry, and GC threads, is not main-thread CPU.
`SystemMS` is kernel-mode CPU execution, not time blocked in an I/O operation.
User/system counters may have coarser resolution than their nanosecond units
suggest; derived differences are clamped to zero to handle sampling skew.

The `ThreadMXBean` and collector beans are cached at startup. Current-thread CPU
accounting is enabled only if supported and permitted; no contention monitoring,
thread enumeration, stack sampling, new GC listeners, or background threads are
added. Unsupported CPU accounting emits one Driver Station warning and keeps
wall/GC diagnostics active. Unavailable CPU/GC numeric values are **-1**, not
zero, with separate availability flags. User-time support may be unavailable
even when total CPU accounting works.

Each active loop gets a fresh baseline, so the robot's intentional wait between
20 ms iterations is excluded. CPU and `System.nanoTime()` wall time are bracketed
inside `beginLoop()`/`endLoop()`; this interval includes the framework and epoch
collection, but excludes final metric publication and AdvantageKit's work
outside `loopFunc()`. It is slightly different from `TotalMS`. `System.nanoTime()`
is used for CPU comparison even in simulation/replay; advancing or pausing the
simulated FPGA clock does not advance this real elapsed clock. CPU and user-time
reads have small boundary skew rather than an atomic snapshot.

GC counters are polled before and after the loop, avoiding notification-delivery
latency. `ActivityDetected` is a **GC activity detector**, not an exact pause
duration. With Serial GC on the roboRIO, a nonzero collection-time delta is useful
evidence of a GC-related stall; concurrent collectors can perform work without
stopping the main thread, and aggregate collector times may overlap. The counters
are approximate milliseconds, so a collection may increment the count without
incrementing time. Boundary-spanning collections may be attributed to an adjacent
loop or missed by these deltas; collections entirely between loops are excluded.
Do not subtract GC time from `OffCpuMS` to infer I/O time. Existing
`LoggedRobot/GCTimeMS` and `LoggedRobot/GCCounts` cover the broader between-update
interval and can help correlate activity outside the measured loop.

These are whole-loop CPU measurements. Individual command/subsystem watchdog
epochs remain wall-only; their completed wall slices do not contain historical
thread CPU counters. Correlate a CPU/off-CPU spike with those slices rather than
treating them as per-command CPU attribution.

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
Run `.\gradlew.bat test --tests "frc.robot.util.Loop*Test"` after an upgrade.

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
