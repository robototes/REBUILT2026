---
name: frc-command-loop-best-practices
description: Architecture principles, WPILib command loop lifecycle constraints, concurrency boundaries, CAN bus practices, and GC mitigation for FRC WPILib robot projects on legacy roboRIO and SystemCore controllers.
version: 3.0.0
---

# FRC Command Loop Architecture & Performance Standards

This document establishes engineering principles, real-time loop constraints, command lifecycle patterns, memory/GC practices, and concurrency boundaries for WPILib robot software across both legacy (**NI roboRIO / roboRIO 2.0**) and next-generation (**SystemCore**) FIRST Robotics Competition controllers.

Each rule is tagged by target platform:
* **`[Universal]`**: Applies to both roboRIO and SystemCore.
* **`[roboRIO]`**: Specific to legacy roboRIO 1.0 / 2.0 hardware constraints.
* **`[SystemCore]`**: Specific to SystemCore hardware capabilities and multi-core architecture.

---

## 1. Controller Architectures & Execution Characteristics

Understanding target hardware constraints determines how concurrency models, CAN bus topologies, and memory practices should be structured.

| Attribute | Legacy roboRIO (1.0 / 2.0) | Next-Gen SystemCore |
| :--- | :--- | :--- |
| **SoC / Compute Engine** | Dual-Core ARM Cortex-A9 (667 MHz on 1.0 / 866 MHz on 2.0) | Quad-Core ARM Cortex-A76 @ 2.4 GHz (Raspberry Pi CM5) |
| **I/O Coprocessor** | Xilinx Zynq-7020 FPGA | Raspberry Pi RP2350 microcontroller |
| **System Memory** | 256 MB (1.0) / 512 MB (2.0) DDR3 | 2 GB / 4 GB (4 GB standard for FRC 2027) |
| **CAN Subsystem** | 1x Standard CAN 2.0B (1 Mbps shared) | 5x Native CAN FD buses (up to 5 Mbps) |
| **Thread Capacity** | Limited dual-core CPU; keep control loop single-threaded | 4 cores allow offloading heavy background work off the main loop |
| **GC Pause Sensitivity** | High: STW pauses on constrained RAM/CPU easily overrun 20 ms | Lower pause sensitivity due to faster 64-bit cores and 2–4 GB RAM |

---

## 2. Execution Model & Real-Time Constraints `[Universal]`

In WPILib Command-Based programming, the robot runtime operates on a cooperative scheduling loop driven by an `IterativeRobotBase` subclass—such as `LoggedRobot` (used by `Robot.java` in this project) or `TimedRobot`—executing by default at **20 ms (50 Hz)**.

### 2.1 The 20 ms Cycle & Overrun Mechanics
* **Default Loop Period (20 ms / 50 Hz):** Every 20 ms, the main thread runs Driver Station updates, `robotPeriodic()` (including `CommandScheduler.getInstance().run()`, subsystem `periodic()` calls, and active command `execute()` / `isFinished()`), mode-specific periodic methods, and telemetry updates.
* **Loop Overrun Warning:** `IterativeRobotBase` monitors each iteration with an internal `Watchdog` configured to `m_period` (`0.02` s by default). When a single iteration exceeds `m_period`, `printLoopOverrunMessage()` logs:
  ```
  Loop time of 0.02s overrun
  ```
  followed by `Watchdog` epoch timings when enabled.
* **Startup vs. Recurring Overruns:** A single overrun immediately after boot or first enable is typically caused by Java Just-In-Time (JIT) compilation and lazy class loading. *Recurring* overruns during steady-state operation indicate a blocking call or CPU bottleneck in the main loop.
* **Consequences of Recurring Overruns:**
  * Jitter and degraded stability in CPU-side feedback controllers (PID / ProfiledPID).
  * Increased latency between driver inputs and actuator response.
  * Delayed motor updates (risking WPILib `MotorSafety` timeouts if updates stall beyond the configured safety timeout).

### 2.2 Loop Budgeting Principles
* **Finish Well Within `m_period`:** Keep total main-loop execution (DS input handling, subsystem `periodic()`, `CommandScheduler.run()`, actuator writes, and logging) comfortably below the 20 ms period so OS scheduling, GC, and bus transit jitter do not cause overruns.
* **Profile Before Guessing:** When overruns occur, use WPILib's `Tracer` / `Watchdog` epoch output or a profiler (such as VisualVM) to identify the specific subsystem or command consuming time rather than optimizing blindly.

---

## 3. Platform-Specific Architectural Principles

### 3.1 Legacy roboRIO Principles `[roboRIO]`
1. **Keep Control Synchronous on the Main Thread `[roboRIO]`:** On the dual-core Cortex-A9, spawning extra threads for control or odometry math adds context-switching overhead and contention. Keep control and odometry on the main 20 ms loop unless a specific vendor library (e.g., CTRE's `OdometryThread`) manages it safely.
2. **Manage Single-Bus CAN 2.0B Utilization `[roboRIO]`:** With one shared 1 Mbps CAN 2.0B bus, keep total bus utilization **below 90%** (CTRE Phoenix 6's documented threshold to avoid indeterminate behavior, noting that the Phoenix diagnostic server adds a 0–5% baseline). Reduce update rates on non-essential status frames and telemetry signals to preserve headroom for control frames.
3. **Minimize Steady-State Allocations `[roboRIO]`:** With 256–512 MB of RAM and a slower dual-core CPU, frequent short-lived heap allocations trigger Stop-The-World (STW) GC pauses that cause loop overruns.

### 3.2 SystemCore Concurrency & Bus Principles `[SystemCore]`
SystemCore's quad-core Cortex-A76 provides substantial compute headroom, but uncoordinated multi-threading still causes race conditions, priority inversion, and non-deterministic state.
Don't hand-roll threads (new Thread, executors); library-managed threads are fine; check with a mentor to break it.

1. **The Main WPILib Loop Remains Single-Threaded `[Universal]`:**
   * **Principle:** Never invoke `CommandScheduler` methods, schedule/cancel commands, or mutate subsystem actuator state from multiple parallel threads. WPILib's `CommandScheduler` is **not thread-safe**—command lifecycle events (`initialize()`, `execute()`, `end()`), subsystem requirement arbitration, and deterministic log replay (e.g., AdvantageKit) assume single-threaded execution on the main loop.
2. **Offload Only Independent Work `[SystemCore]`:**
   * **Principle:** Use background threads only for work that does not touch `CommandScheduler` or write directly to actuators—for example, high-frequency sensor/odometry ingestion (such as vendor odometry threads), on-core vision/AprilTag pipelines, heavy dynamic trajectory generation, or asynchronous disk logging.
3. **Avoid Lock Contention on the Main Loop `[Universal]`:**
   * **Principle:** The main loop must never block waiting on a lock (`synchronized`, `ReentrantLock`) held by a slower background worker. Transfer data between threads using non-blocking patterns such as `AtomicReference<T>` snapshots, double buffering, or lock-free queues.
4. **Use Bounded Executors and Appropriate Priorities `[SystemCore]`:**
   * **Principle:** Keep heavy background workers (vision, path generation) at normal or lower priority than the main control loop, and use bounded, named thread pools rather than unbounded pools (`Executors.newCachedThreadPool()`).
5. **Clean Thread Teardown `[Universal]`:**
   * **Principle:** Ensure user-spawned threads and executors respond to interruption and shut down cleanly on robot/program teardown so lingering threads do not leak memory or hold resources across restarts.
6. **Partition Traffic Across CAN FD Buses `[SystemCore]`:**
   * **Principle:** SystemCore provides **5x native CAN FD buses** (up to 5 Mbps). Isolate high-rate drivetrain/swerve devices onto dedicated CAN FD buses separate from peripheral mechanisms to reduce bus congestion and simplify wiring.

---

## 4. Loop Execution Restrictions `[Universal]`

Avoid the following blocking or high-latency operations inside `periodic()`, `initialize()`, `execute()`, and `isFinished()` on all platforms:

1. **Thread Sleeping & Spin-Waits:** Never call `Thread.sleep()`, `Timer.delay()`, or busy-wait loops waiting for a sensor or condition.
2. **Synchronous File & Disk I/O:** Never read or write files (`FileWriter`, `Scanner`, synchronous JSON/CSV parsing) in the cyclic loop. Use WPILib's `DataLogManager` or background threads.
3. **Blocking Network Calls:** Never make synchronous HTTP requests, socket reads, or remote RPCs on the main thread.
4. **Blocking Locks Shared with Workers:** Never acquire locks in the main loop that may be held by a slow background thread.
5. **Runtime Hardware Configuration Calls:** Never call blocking configuration methods inside the cyclic loop:
   * **CTRE Phoenix 6:** `device.getConfigurator().apply(...)` blocks waiting for device acknowledgment (up to `DefaultTimeoutSeconds`).
   * **REVLib:** `spark.configure(config, resetMode, PersistMode.kPersistParameters)` blocks CAN communication while writing to flash memory.
   * Perform all hardware configuration during initialization (subsystem constructors or startup).
6. **Unnecessary Telemetry Flushes:** Avoid calling `NetworkTableInstance.getDefault().flush()` every cycle unless specifically needed for latency-critical synchronization; `TimedRobot` already handles periodic local updates, and excessive `flush()` calls increase network traffic.

---

## 5. Command Lifecycle Best Practices `[Universal]`

```
  +--------------+
  | initialize() |  --> Run once when scheduled (non-blocking setup)
  +--------------+
         |
         v
  +--------------+
  |  execute()   |  --> Run every 20 ms loop iteration
  +--------------+
         |
         v
  +--------------+
  | isFinished() |  --> Pure condition query; no actuator writes
  +--------------+
         |
         +-------------------+
         | (true or cancel)  |
         v                   v
  +---------------------------------+
  |     end(boolean interrupted)    |  --> Neutralize actuators / safe handoff
  +---------------------------------+
```

### 5.1 `initialize()`
* **Role:** One-time preparation when the command is scheduled (reset timers, capture starting setpoints, reset profiles/estimators).
* **Principle:** Must return immediately. Never loop waiting for a mechanism or sensor inside `initialize()`.

### 5.2 `execute()`
* **Role:** Periodic control logic (evaluating a profile step, updating motor setpoints/voltages).
* **Principle:** Keep logic fast and non-blocking. Prefer offloading high-frequency PID/feedforward loops to motor controller hardware where appropriate.

### 5.3 `isFinished()`
* **Role:** Evaluates whether the command has reached its exit condition.
* **Principle:** Keep `isFinished()` a pure query—do not write to actuators or mutate external subsystem state inside it.

### 5.4 `end(boolean interrupted)`
* **Role:** Safe teardown when the command completes naturally or is interrupted/cancelled.
* **Safety Principle:** Always neutralize actuators (e.g., set voltage/output to `0`, or hand off control to a safe default command) so mechanisms do not keep running after a command ends.

---

## 6. Allocation & Garbage Collection (GC) Mitigation

WPILib documentation warns that creating a high volume of short-lived objects in periodic loops fills the heap and triggers Stop-The-World GC pauses that cause loop overruns—especially on roboRIO.

### 6.1 Common Allocation Traps in Hot Paths
* **Dynamic String Concatenation (`[roboRIO]` critical, `[SystemCore]` recommended):**
  ```java
  // AVOID: Allocates StringBuilder and String objects every cycle
  SmartDashboard.putString("Status", "Current: " + currentDraw + " A, Temp: " + temp);

  // PREFER: Publish primitives directly to cached NetworkTable publishers
  currentEntry.setDouble(currentDraw);
  tempEntry.setDouble(temp);
  ```
* **Stream Pipelines and Capturing Lambdas in Tight Loops (`[roboRIO]`):**
  ```java
  // AVOID: Allocates a Stream pipeline and Spliterator every 20 ms
  subsystems.stream().filter(Subsystem::needsUpdate).forEach(Subsystem::update);

  // PREFER: Simple indexed for-loop
  for (int i = 0; i < subsystemList.size(); i++) {
      subsystemList.get(i).update();
  }
  ```
  *Note on WPILib Lambdas:* Per WPILib's Command-Based documentation, inline commands using lambdas and method references (`Commands.run(() -> ...)`, `Commands.runOnce(subsystem::action, subsystem)`) are the **recommended idiomatic pattern** when constructed once during initialization (`configureBindings()` or subsystem factory methods). Non-capturing lambdas are cached by the JVM (`invokedynamic`) and do not allocate per call; only avoid *creating new `Stream` pipelines, capturing lambdas, or command objects* repeatedly inside `periodic()` or `execute()`.
* **Auto-Boxing & Enhanced-For Over Temporary Collections:** Prefer primitive arrays (`double[]`) over boxed collections (`ArrayList<Double>`). Note that enhanced-for loops over `List`s allocate an `Iterator` unless eliminated by JIT escape analysis.

### 6.2 WPILib-Specific Allocation Notes
* **Immutable Geometry Types (`[Universal]`):** WPILib's `Pose2d`, `Translation2d`, `Rotation2d`, and `Transform2d` are **immutable** (`final` fields). You cannot mutate their internal fields in place. Use pre-allocated constants (e.g., `Rotation2d.kZero`, `Pose2d.kZero`) where applicable and avoid gratuitous intermediate geometry chains in tight loops.
* **Command Factories in Periodic (`[Universal]`):** Never construct commands via factories (e.g., `Commands.runOnce(...)`, inline `SequentialCommandGroup`s) inside `periodic()` or `execute()`. Build and bind commands once during initialization (`configureBindings()`).

---

## 7. Subsystem & Hardware Optimization

### 7.1 CAN Bus Optimization
* **roboRIO CAN 2.0B (`[roboRIO]`):** Total bus bandwidth is 1 Mbps. Per CTRE Phoenix 6 documentation, keep total CAN bus utilization **below 90%** (accounting for the 0–5% diagnostic server baseline). Slow down non-essential status signals so control frames are never delayed.
* **SystemCore CAN FD (`[SystemCore]`):** SystemCore provides **5x native CAN FD buses** (up to 5 Mbps). Partition high-rate drivetrain actuators onto dedicated buses separate from peripheral mechanisms.
* **Non-Blocking Status Signal Reads (`[Universal]`):**
  * In CTRE Phoenix 6, device getters (e.g., `talonFX.getPosition()`) implicitly refresh by default.
  * When caching `StatusSignal` objects, use non-blocking `signal.refresh()` or bulk `BaseStatusSignal.refreshAll(...)` (or `BaseStatusSignal.waitForAll(0, ...)`). Avoid calling `waitForUpdate(timeout)` with a non-zero timeout in the 20 ms loop, as it blocks the calling thread until a frame arrives or the timeout expires.

### 7.2 Control Distribution `[Universal]`
* **Onboard Motor Controller Closed Loops (Preferred):** Run high-frequency PID, feedforward, and motion profiling (e.g., Motion Magic, Spark ClosedLoop) directly on the motor controller hardware (`TalonFX`, `SparkMax`/`SparkFlex`) to offload the robot CPU and avoid 20 ms loop latency.
* **Controller CPU Closed Loops:** When running closed-loop control on the robot CPU (roboRIO or SystemCore), instantiate `PIDController` / `ProfiledPIDController` / `SimpleMotorFeedforward` once in the subsystem constructor and reuse them.

---

## 8. Implementation Architecture Example

The following example demonstrates lock-free cross-thread snapshot transfer (`[Universal]`), pre-allocated `SwerveModulePosition` containers, one-time hardware configuration, and primitive NetworkTables publishing:

```java
package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.concurrent.atomic.AtomicReference;

public class DriveSubsystem extends SubsystemBase {
    private static final int MODULE_COUNT = 4;

    // Immutable snapshot for lock-free cross-thread transfer from a background sensor thread
    public static class OdometrySnapshot {
        public final double[] distances = new double[MODULE_COUNT];
        public final double[] angles = new double[MODULE_COUNT];
        public final double timestamp;

        public OdometrySnapshot(double timestamp) {
            this.timestamp = timestamp;
        }
    }

    // Non-blocking atomic reference for background thread -> main loop handoff
    private final AtomicReference<OdometrySnapshot> latestSnapshot =
        new AtomicReference<>(new OdometrySnapshot(0.0));

    // Pre-allocated mutable SwerveModulePosition containers
    private final SwerveModulePosition[] cachedPositions = new SwerveModulePosition[MODULE_COUNT];

    // Pre-allocated NetworkTables publisher (direct primitive writes, no string concatenation)
    private final DoublePublisher speedPublisher = NetworkTableInstance.getDefault()
        .getTable("Drive")
        .getDoubleTopic("VelocityMPS")
        .publish();

    public DriveSubsystem() {
        for (int i = 0; i < MODULE_COUNT; i++) {
            cachedPositions[i] = new SwerveModulePosition(0.0, Rotation2d.kZero);
        }
        configureHardware();
    }

    private void configureHardware() {
        // Apply blocking motor/sensor configurations once during initialization
    }

    // Invoked by a background sensor/odometry thread (e.g., on SystemCore)
    public void updateAsynchronousSensors(double timestamp, double[] dist, double[] rads) {
        OdometrySnapshot next = new OdometrySnapshot(timestamp);
        System.arraycopy(dist, 0, next.distances, 0, MODULE_COUNT);
        System.arraycopy(rads, 0, next.angles, 0, MODULE_COUNT);
        latestSnapshot.set(next); // Non-blocking atomic publish
    }

    @Override
    public void periodic() {
        // Non-blocking snapshot consumption on the main thread
        OdometrySnapshot snapshot = latestSnapshot.get();

        // Update pre-allocated SwerveModulePosition fields
        for (int i = 0; i < MODULE_COUNT; i++) {
            cachedPositions[i].distanceMeters = snapshot.distances[i];
            cachedPositions[i].angle = Rotation2d.fromRadians(snapshot.angles[i]);
        }

        // Direct primitive telemetry publication (no string formatting allocations)
        speedPublisher.set(getForwardVelocity());
    }

    public double getForwardVelocity() {
        return 0.0;
    }
}
```

---

## 9. Diagnostics & Profiling Checklist

Before deploying code to competition robots, verify system health:

* [ ] **`[Universal]` No Recurring Overruns:** After initial startup/enable JIT warmup, Driver Station logs show no recurring `Loop time of 0.02s overrun` warnings during normal operation.
* [ ] **`[Universal]` No Blocking Calls in the Loop:** No `Thread.sleep()`, `Timer.delay()`, synchronous disk/network I/O, or blocking hardware config calls (`apply()`, `configure(..., kPersistParameters)`) inside `periodic()`, `initialize()`, `execute()`, or `isFinished()`.
* [ ] **`[Universal]` Safe Command Teardown & Requirements:** Commands declare `addRequirements(...)` to prevent conflicting actuator writes and neutralize actuators in `end(boolean interrupted)`.
* [ ] **`[roboRIO]` Low Steady-State Allocation Rate:** VisualVM (or heap profiling) shows minimal steady-state allocation churn, avoiding string concatenation, streams, and command factories in periodic paths.
* [ ] **`[roboRIO]` CAN 2.0B Bus Health:** Total CAN bus utilization stays **below 90%** (per CTRE Phoenix 6 guidelines), with non-essential telemetry throttled.
* [ ] **`[SystemCore]` Thread Boundaries & Bus Partitioning:**
  * `CommandScheduler.run()` and actuator writes run exclusively on the main thread.
  * Background workers use bounded executors, avoid blocking locks with the main thread, and shut down cleanly on teardown.
  * High-rate actuators are partitioned across SystemCore's CAN FD buses.
