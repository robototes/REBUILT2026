---
name: frc-command-loop-best-practices
description: Complete architecture standards, loop timing budgets, lifecycle constraints, zero-allocation practices, multi-threading boundaries, and GC mitigation for FRC WPILib robot projects on both legacy roboRIO and System Core controllers.
version: 2.1.0
---

# FRC Command Loop Architecture & Performance Standards

This document establishes the engineering guidelines, real-time timing budgets, command lifecycle patterns, JVM performance constraints, and multi-threading execution rules for robot software development across both legacy (NI roboRIO / roboRIO 2.0) and next-generation (**System Core**) FIRST Robotics Competition control hardware.

---

## 1. Controller Architectures & Execution Characteristics

Understanding target hardware constraints determines how software budgets, concurrency models, and memory architectures must be structured.

| Attribute | Legacy roboRIO (1.0 / 2.0) | Next-Gen System Core |
| :--- | :--- | :--- |
| **SoC / Compute Engine** | Dual-Core ARM Cortex-A9 (667–866 MHz) | Quad-Core ARM Cortex-A76 (Raspberry Pi CM5) |
| **I/O Coprocessor** | Xilinx Zynq FPGA | Raspberry Pi RP2350 microcontroller |
| **System Memory** | 256 MB (1.0) / 512 MB (2.0) DDR3 | 4 GB – 8 GB LPDDR4X |
| **CAN Subsystem** | 1x Standard CAN 2.0B (1 Mbps shared) | Multiple Native CAN FD buses (up to 5 Mbps) |
| **Thread Capacity** | Single-core pinned hot path, strict contention | Multi-threaded offloading supported across 4 full cores |
| **GC Pause Sensitivity** | Extreme: 15–80 ms Stop-The-World (STW) pauses | Lower pause times via modern 64-bit JVMs (e.g., ZGC/Shenandoah) |

---

## 2. Execution Model & Real-Time Constraints

In WPILib Command-Based programming, the robot runtime operates on a cooperative scheduling loop driven by `TimedRobot` executing by default at **20 ms (50 Hz)**.

### 2.1 The 20 ms Cycle & Overrun Mechanics

* **Default Loop Period (20 ms / 50 Hz):** Every 20 ms, the main thread runs Driver Station packet ingestion, subsystem `periodic()` calls, active command lifecycle steps (`execute()`, `isFinished()`), and telemetry dispatch.
* **TimedRobot Overrun Warning:** `TimedRobot` prints a loop overrun warning (`Loop time of 0.020000s overrun to 0.02xxxxs!`) as soon as any single iteration exceeds the configured loop period ($20\text{ ms}$ by default).
* **Severe Overruns ($\ge 50\text{ ms}$):** If loop execution takes $\ge 50\text{ ms}$, more than two entire control cycles are dropped, triggering Driver Station packet drops, brownout safety states, and CAN buffer saturation.
* **Consequences of Overruns:**
  * Jitter and degraded stability in feedback controllers (PIDF / ProfiledPID).
  * Latency between driver stick inputs and motor response.
  * CAN buffer saturation, dropped heartbeat frames, and motor safety watchdog cutouts.

### 2.2 Unified Loop Timing Budget (20 ms Total Frame)

Regardless of controller, execution times are budgeted with a target total runtime under **12.0 ms**, maintaining a mandatory $\ge 8.0\text{ ms}$ **safety headroom margin** for OS thread scheduling, GC cycles, and bus transit jitter.

| Phase | Budget Target | Scope & Responsibilities |
| :--- | :--- | :--- |
| **DS & CAN Ingestion** | `< 2.5 ms` | Read DS joystick inputs, update cached CAN status signals. |
| **Subsystem `periodic()`** | `< 3.5 ms` | Total across all subsystems (sensors, estimators, odometry). |
| **`CommandScheduler.run()`** | `< 3.5 ms` | Command execution logic, trigger evaluation, finishes. |
| **Actuator Outputs & Writes** | `< 1.5 ms` | Setting motor voltages/duty cycles over CAN/PWM. |
| **Telemetry & Data Logging** | `< 1.0 ms` | Enqueueing binary log records, NetworkTables updates. |
| **Target Total Active Execution** | **$< 12.0\text{ ms}$** | **Total active time across all phases.** |
| **Safety Headroom Margin** | **$\ge 8.0\text{ ms}$** | **Reserved buffer for OS scheduling jitter and CAN bursts.** |

* **Single-Command Target:** An individual command's `execute()` method must target $< 0.5\text{ ms}$.
* **Per-Routine Diagnostic Threshold:** If any individual subsystem `periodic()` or command `execute()` routine exceeds $2.5\text{ ms}$, it must be flagged for profiling and optimization.
* **Overrun Defect Threshold:** If total loop execution exceeds $16.0\text{ ms}$ (approaching the $20\text{ ms}$ limit), it is treated as a critical performance defect.

---

## 3. Platform-Specific Architectural Rules

### 3.1 Hard Rules for Legacy roboRIO

1. **Zero Multi-Threading for Control Math:** The roboRIO Cortex-A9 cannot handle high context-switching overhead. Keep all control and odometry operations synchronous on the main thread; do not spawn unmanaged threads.
2. **Strict Single-Bus CAN Throttling:** With only one standard CAN 2.0B network, overall bus utilization must remain strictly below **70%**. Keep all non-essential telemetry frames at 100–250 ms.
3. **Extreme Zero-Allocation Mandate:** Memory pressure directly induces GC Stop-The-World (STW) pauses that paralyze the main loop for up to 80 ms.

### 3.2 Rules for System Core Multi-Threading & Concurrency

The quad-core architecture of System Core provides substantial compute, but uncoordinated multi-threading introduces race conditions, priority inversion, and non-deterministic state. The following rules govern concurrent execution on System Core:

#### Rule 1: The Main WPILib Loop Remains Strictly Single-Threaded
* **Constraint:** Never execute subsystem logic, command scheduling, or actuator setpoint generation across multiple parallel worker threads. The WPILib `CommandScheduler` is fundamentally **not thread-safe**.
* **Rationale:** WPILib command lifecycle events (`initialize()`, `execute()`, `end()`), subsystem requirement arbitration, and default command resolution assume single-threaded execution. Spreading commands across threads breaks state machine guarantees and corrupts AdvantageKit deterministic log replays.

#### Rule 2: Background Thread Confinement (Worker Thread Boundaries)
Multi-threading on System Core is strictly limited to four isolated functional domains:
1. **Asynchronous Sensor Ingestion:** High-frequency CAN status reads (e.g., reading odometry/gyro updates via hardware-level threads or high-rate notifiers at 100 Hz–250 Hz).
2. **On-Core Vision Processing:** Local AprilTag detection, target tracking, and neural network inferencing pipelines.
3. **Heavy Trajectory & Path Generation:** Offline or mid-match path generation (e.g., dynamic Choreo/PathPlanner trajectories) that exceed 5 ms of compute.
4. **Data Logging I/O:** Disk writes handled through asynchronous queues.

#### Rule 3: Zero Mutex Contention on the Hot Path
* **Constraint:** Worker threads must **never** hold locks or `synchronized` blocks that the main loop has to wait on. Mutex locks on the main thread cause priority inversion and immediate frame drops.
* **Non-Blocking Transfer Mechanisms:**
  * **Double Buffering:** Worker threads write to a secondary back-buffer; the main thread swaps references atomically at the start of `periodic()`.
  * **Atomic References & Snapshots:** Use `AtomicReference<T>` or lock-free Single-Producer Single-Consumer (SPSC) ring buffers.
  * **Copy-on-Read Snapshots:** Subsystem `periodic()` reads the latest stamped sensor or vision measurement in sub-microsecond time without blocking on the worker thread's internal calculations.

#### Rule 4: Explicit Thread Priorities and Core Pinning
* **Constraint:** The main robot control loop thread must run at higher OS priority than asynchronous background tasks.
* **Guideline:** Heavy computational workers (vision pipelines, path generators) must run at default or lower thread priority (e.g., `Thread.NORM_PRIORITY - 1` or via OS `nice` levels). Avoid unmanaged thread pools (`Executors.newCachedThreadPool()`). Use fixed-size, named executors sized to System Core’s core budget to prevent CPU cache thrashing on Core 0.

#### Rule 5: Lifecycle Cleanup and Safe Teardown
* **Constraint:** All user-spawned threads and executors must respond to interruption and shut down cleanly during robot code restart or disabled transitions.
* **Rationale:** Lingering worker threads consume CPU cycles, keep uncollected objects pinned across code restarts, and cause out-of-memory errors during rapid code redeployments in the pit.

#### Rule 6: CAN FD Bus Partitioning
* Take advantage of multiple native CAN FD buses. Isolate high-frequency swerve drive actuation to dedicated CAN FD lines while routing peripheral actuators and telemetry nodes to secondary buses.

---

## 4. Hard Execution Restrictions (Universal)

The following operations are **strictly forbidden** inside `periodic()`, `initialize()`, `execute()`, and `isFinished()` across all platforms:

1. **Thread Sleeping & Active Waits:** Never call `Thread.sleep()`, `Timer.delay()`, or spin-wait on conditions.
2. **Synchronous File & Disk I/O:** Never read or write local files (`FileWriter`, `Scanner`, raw JSON/CSV parsing) in the loop. Use WPILib's asynchronous `DataLogManager` or background worker threads.
3. **Blocking Network Calls:** Never make synchronous HTTP calls, socket reads, or remote RPCs.
4. **Lock-Heavy Synchronization:** Never acquire blocking locks shared with high-latency background workers. Use lock-free atomics (`AtomicReference`, lock-free ring buffers) for passing data between threads.
5. **Runtime Hardware Configuration Changes:** Never invoke mutating configuration commands (e.g., `motor.config*()`, `motor.setInverted()`, `motor.setIdleMode()`, or CAN PID gain updates) inside the cyclic loop. These calls block on CAN acknowledgment frames. Perform all configurations during initialization.
6. **Synchronous Telemetry Flushes:** Never call `NetworkTableInstance.getDefault().flush()`. Allow NetworkTables' dedicated background thread to handle network dispatch.

---

## 5. Command Lifecycle Best Practices

Each phase of the WPILib `Command` lifecycle serves a specific role and carries strict execution expectations:

```
  +--------------+
  | initialize() |  --> Run once when scheduled (Target: < 0.2 ms)
  +--------------+
         |
         v
  +--------------+
  |  execute()   |  --> Run every 20 ms loop iteration (Target: < 0.5 ms)
  +--------------+
         |
         v
  +--------------+
  | isFinished() |  --> Pure condition query; no mutations (Target: < 0.05 ms)
  +--------------+
         |
         +-------------------+
         | (true or cancel)  |
         v                   v
  +---------------------------------+
  |     end(boolean interrupted)    |  --> Actuator neutralization (Target: < 0.2 ms)
  +---------------------------------+
```

### 5.1 `initialize()`
* **Role:** One-time preparation when the command is scheduled. Reset timers, capture starting setpoints, zero profiles, or reset state estimators.
* **Cost Target:** `< 0.2 ms`.
* **Anti-Pattern:** Looping until a mechanism reaches a sensor trigger. Initialization must exit immediately.

### 5.2 `execute()`
* **Role:** Periodic execution logic (e.g., computing a profile step, writing target setpoints or voltages to controllers).
* **Cost Target:** `< 0.5 ms` per individual command execution.
* **Diagnostic Flag:** Any execution exceeding $2.5\text{ ms}$ indicates a defect (e.g., blocking call, unoptimized math).
* **Best Practice:** Keep logic purely mathematical. Offload high-frequency PID loops to motor controller hardware wherever possible.

### 5.3 `isFinished()`
* **Role:** Evaluates whether the command has achieved its exit condition.
* **Cost Target:** `< 0.05 ms`.
* **Rule:** Must be pure and idempotent. Do not write to actuators or mutate external subsystem state inside this method.

### 5.4 `end(boolean interrupted)`
* **Role:** Safe teardown when the command completes naturally or is cancelled/interrupted.
* **Safety Mandate:** Always neutralize actuators (e.g., set voltage to $0\text{ V}$, engage brake mode, or hand off control to a safe default command).
* **Cost Target:** `< 0.2 ms`.

---

## 6. Zero-Allocation & Garbage Collection (GC) Mitigation

Regardless of the target controller, real-time deterministic software must avoid continuous heap churn in the hot path.

### 6.1 Prohibited Java Patterns in the Hot Path
* **No Object Instantiation (`new`) in Loops:** Pre-allocate all math objects, arrays, and intermediate data structures during subsystem construction and reuse them.
* **No Java Streams or Inline Lambdas:**
  ```java
  // INCORRECT: Allocates Stream, Pipeline, and closure instances every 20ms
  subsystems.stream().filter(Subsystem::needsUpdate).forEach(Subsystem::update);

  // CORRECT: Traditional indexed for-loop with zero heap allocations
  for (int i = 0; i < subsystemList.size(); i++) {
      subsystemList.get(i).update();
  }
  ```
* **No Dynamic String Concatenation or Formatting:**
  ```java
  // INCORRECT: Allocates StringBuilder, Formatters, and String objects every cycle
  SmartDashboard.putString("Status", "Current: " + currentDraw + " A, Temp: " + temp);

  // CORRECT: Publish primitive types directly to cached NetworkTable entries
  currentEntry.setDouble(currentDraw);
  tempEntry.setDouble(temp);
  ```
* **No Auto-Boxing and Wrapper Collections:** Avoid collections of boxed primitives (e.g., `ArrayList<Double>`). Use fixed-size primitive arrays (`double[]`) or pre-allocated circular buffers.

### 6.2 WPILib-Specific Allocation Traps
* **Geometry and Math Instantiation:** Avoid calling factory methods or constructors that instantiate `Pose2d`, `Translation2d`, `Rotation2d`, or `Transform2d` every 20 ms. Pre-allocate these objects and update internal fields or pass scalar values.
* **Dynamic Command Factories in Periodic:** Avoid calling command factories (e.g., `Commands.runOnce(...)` or inline sequences) inside `periodic()` or `execute()`. Bind commands once in `configureBindings()` or evaluate triggers using state machines.

---

## 7. Subsystem & Hardware Optimization

### 7.1 CAN Bus Optimization: CAN 2.0B vs. CAN FD
* **roboRIO (Standard CAN 2.0B):** Total bus capacity is 1 Mbps. Bus utilization above **70%** triggers packet loss. Throttle telemetry to 100–250 ms and prioritize motor control frames.
* **System Core (CAN FD):** Up to 5 Mbps per bus with native multi-bus support. Distribute high-rate actuators (e.g., swerve drive azimuth and steer) across Bus 0 and Bus 1, keeping peripheral mechanisms on Bus 2.
* **Non-Blocking Status Queries:** Use asynchronous signal polling (such as CTRE Phoenix 6 `StatusSignal.refresh()` / `waitForUpdate(0)`) rather than blocking synchronous getters on both platforms.

### 7.2 Control Distribution: Onboard vs. Controller CPU
* **Onboard Motor Controller Closed Loops (Preferred):** Execute high-frequency feedback loops (1 kHz PID, feedforward, Motion Magic / Smart Motion) directly on the motor controller hardware (Talon FX, Spark Max/Flex).
* **Controller Closed Loops:** When running control loops on the controller CPU (roboRIO or System Core), use pre-allocated controller structures (`SimpleMotorFeedforward`, `ProfiledPIDController`) without intermediate object creation.

---

## 8. Implementation Architecture Example

The following example demonstrates pre-allocated buffers, lock-free thread offloading for background measurements, zero allocations in the cyclic path, and direct primitive telemetry publishing:

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

    // Immutable snapshot class for lock-free cross-thread data transfer
    public static class OdometrySnapshot {
        public final double[] distances = new double[MODULE_COUNT];
        public final double[] angles = new double[MODULE_COUNT];
        public final double timestamp;

        public OdometrySnapshot(double timestamp) {
            this.timestamp = timestamp;
        }
    }

    // Lock-free atomic reference for high-frequency worker thread updates
    private final AtomicReference<OdometrySnapshot> latestSnapshot = 
        new AtomicReference<>(new OdometrySnapshot(0.0));

    // Pre-allocated SwerveModulePosition and Rotation2d instances
    private final SwerveModulePosition[] cachedPositions = new SwerveModulePosition[MODULE_COUNT];

    // Pre-allocated NetworkTable publishers (direct primitive writes)
    private final DoublePublisher speedPublisher = NetworkTableInstance.getDefault()
        .getTable("Drive")
        .getDoubleTopic("VelocityMPS")
        .publish();

    public DriveSubsystem() {
        for (int i = 0; i < MODULE_COUNT; i++) {
            cachedPositions[i] = new SwerveModulePosition(0.0, new Rotation2d());
        }
        configureHardware();
    }

    private void configureHardware() {
        // Configure motor controllers and tune CAN frame periods here
    }

    // Invoked by high-frequency background worker thread on System Core
    public void updateAsynchronousSensors(double timestamp, double[] dist, double[] rads) {
        OdometrySnapshot next = new OdometrySnapshot(timestamp);
        System.arraycopy(dist, 0, next.distances, 0, MODULE_COUNT);
        System.arraycopy(rads, 0, next.angles, 0, MODULE_COUNT);
        latestSnapshot.set(next); // Atomic, non-blocking write
    }

    @Override
    public void periodic() {
        // Non-blocking snapshot consumption on the main thread (< 0.05 ms)
        OdometrySnapshot snapshot = latestSnapshot.get();

        // Update pre-allocated module positions in-place
        for (int i = 0; i < MODULE_COUNT; i++) {
            cachedPositions[i].distanceMeters = snapshot.distances[i];
            cachedPositions[i].angle = cachedPositions[i].angle.minus(cachedPositions[i].angle)
                .plus(Rotation2d.fromRadians(snapshot.angles[i]));
        }

        // Primitive telemetry publication (zero heap garbage generated)
        speedPublisher.set(getForwardVelocity());
    }

    public double getForwardVelocity() {
        return 0.0;
    }
}
```

---

## 9. Diagnostics & Profiling Checklist

Before deploying code to competition robots on either platform, verify system health using this checklist:

* [ ] **Zero Overrun Logs:** The Driver Station log shows zero recurring `Loop time of 0.020000s overrun` warnings during continuous match simulations.
* [ ] **Target Total Active Execution Time:** Steady-state robot execution completes in $< 12.0\text{ ms}$, guaranteeing at least $8.0\text{ ms}$ of safety margin per 20 ms frame.
* [ ] **Per-Routine Timing Compliance:** Individual command `execute()` passes maintain $< 0.5\text{ ms}$; no subsystem `periodic()` or command exceeds the $2.5\text{ ms}$ diagnostic threshold.
* [ ] **Zero Steady-State Heap Allocation:** VisualVM, JProfiler, or the embedded profiling agent shows memory allocation rates near $0\text{ B/sec}$ in disabled and teleoperated idle states.
* [ ] **No `new` Keywords in Hot Paths:** Verified that no `new` keywords or continuous factory allocations exist inside `periodic()`, `execute()`, or `isFinished()` methods across all subsystems and commands.
* [ ] **CAN Bus Health:** CAN bus utilization remains consistently below **70%** (for roboRIO CAN 2.0B) or operates across segregated CAN FD buses with low bus arbitration delays (for System Core).
* [ ] **Thread Boundaries & Contention (System Core):**
  * `CommandScheduler.run()` and all actuator writes reside exclusively on the main thread.
  * Background threads are confined strictly to sensor ingestion, vision, path generation, or asynchronous logging.
  * No `synchronized` blocks or mutexes are locked by the main thread.
  * All background executors use fixed pool sizes, appropriate priorities, and respond cleanly to interruption during code teardown.
* [ ] **Clean Command Requirements:** All active commands explicitly declare required subsystems via `addRequirements(...)` to prevent conflicting actuator writes in the same loop cycle.