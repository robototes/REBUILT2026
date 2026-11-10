---
name: frc-command-loop-best-practices
description: Complete architecture standards, loop timing budgets, lifecycle constraints, zero-allocation practices, and GC mitigation for FRC WPILib robot projects.
version: 1.2.0
---

# FRC Command Loop Architecture & Performance Standards

This document establishes the official engineering guidelines, real-time timing budgets, command lifecycle patterns, and JVM performance constraints for robot software development using WPILib Command-Based architecture.

## 1. Execution Model & Real-Time Constraints

In WPILib Command-Based programming, the robot runtime operates on a cooperative, single-threaded scheduling loop driven by `TimedRobot` executing by default at **20 ms (50 Hz)**.

### 1.1 The 20 ms Cycle & Overrun Mechanics

* **Default Loop Period (20 ms / 50 Hz):** Every 20 ms, the main thread runs Driver Station packet ingestion, subsystem `periodic()` calls, active command lifecycle steps (`execute()`, `isFinished()`), and telemetry dispatch.
* **TimedRobot Overrun Warning:** `TimedRobot` prints a loop overrun warning (`Loop time of 0.020000s overrun to 0.02xxxxs!`) as soon as any single iteration exceeds the configured loop period ($20\text{ ms}$ by default). 
* **Severe Overruns ($\ge 50\text{ ms}$):** If loop execution takes $\ge 50\text{ ms}$, more than two entire control cycles are dropped, triggering Driver Station packet drops and CAN buffer saturation.
* **Consequences of Overruns:**
  * Jitter and degraded stability in feedback controllers (PIDF / ProfiledPID).
  * Latency between driver stick inputs and motor response.
  * CAN buffer saturation, dropped heartbeat frames, and motor safety watchdog cutouts.

### 1.2 Unified Loop Timing Budget (20 ms Total Frame)

To guarantee deterministic behavior and prevent loop overruns, execution times are budgeted with a target total runtime under **12.0 ms**, maintaining a mandatory **$\ge 8.0\text{ ms}$ safety headroom margin** for OS thread scheduling, GC cycles, and CAN transit jitter.

| Phase | Budget Target | Scope & Responsibilities |
| :--- | :--- | :--- |
| **DS & CAN Ingestion** | `< 2.5 ms` | Read DS joystick inputs, update cached CAN status signals. |
| **Subsystem `periodic()`** | `< 3.5 ms` | Total across all subsystems (sensors, estimators, odometry). |
| **`CommandScheduler.run()`** | `< 3.5 ms` | Command execution logic, trigger evaluation, finishes. |
| **Actuator Outputs & Writes** | `< 1.5 ms` | Setting motor voltages/duty cycles over CAN/PWM. |
| **Telemetry & Data Logging** | `< 1.0 ms` | Enqueueing binary log records, NetworkTables updates. |
| **Target Total Active Execution**| **$< 12.0\text{ ms}$** | **Total active time across all phases.** |
| **Safety Headroom Margin** | **$\ge 8.0\text{ ms}$** | **Reserved buffer for OS scheduling jitter and CAN bursts.** |

* **Single-Command Target:** An individual command's `execute()` method must target **$< 0.5\text{ ms}$**.
* **Per-Routine Diagnostic Threshold:** If any individual subsystem `periodic()` or command `execute()` routine exceeds **$2.5\text{ ms}$**, it must be flagged for profiling and optimization.
* **Overrun Defect Threshold:** If total loop execution exceeds **$16.0\text{ ms}$** (approaching the $20\text{ ms}$ limit), it is treated as a critical performance defect.

---

## 2. Hard Execution Restrictions

Because the main robot loop is single-threaded on the roboRIO CPU, any blocking operation delays all subsystems, estimators, and safety loops. The following are strictly forbidden inside `periodic()`, `initialize()`, `execute()`, and `isFinished()`:

1. **Thread Sleeping & Active Waits:** Never call `Thread.sleep()`, `Timer.delay()`, or spin-wait on conditions.
2. **Synchronous File & Disk I/O:** Never read or write local files (`FileWriter`, `Scanner`, raw JSON/CSV parsing) in the loop. Use WPILib's asynchronous `DataLogManager` or background worker threads.
3. **Blocking Network Calls:** Never make synchronous HTTP calls, socket reads, or remote RPCs.
4. **On-Thread Heavy Computer Vision:** Never run OpenCV pipelines, contour searches, or `solvePnP` on the roboRIO main thread. Vision pipelines must execute on a coprocessor (Orange Pi, Limelight, PhotonVision) or on an isolated worker thread.
5. **Runtime Hardware Configuration Changes:** Never invoke mutating configuration commands (e.g., `motor.config*()`, `motor.setInverted()`, `motor.setIdleMode()`, or CAN PID gain updates) inside the cyclic loop. These calls block on CAN acknowledgment frames. Perform all configurations during initialization.
6. **Synchronous Telemetry Flushes:** Never call `NetworkTableInstance.getDefault().flush()`. Allow NetworkTables' dedicated background thread to handle network dispatch.

---

## 3. Command Lifecycle Best Practices

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

### 3.1 `initialize()`
* **Role:** One-time preparation when the command is scheduled. Reset timers, capture starting setpoints, zero profiles, or reset state estimators.
* **Cost Target:** `< 0.2 ms`.
* **Anti-Pattern:** Looping until a mechanism reaches a sensor trigger. Initialization must exit immediately.

### 3.2 `execute()`
* **Role:** Periodic execution logic (e.g., computing a profile step, writing target setpoints or voltages to controllers).
* **Cost Target:** `< 0.5 ms` per individual command execution.
* **Diagnostic Flag:** Any execution exceeding **$2.5\text{ ms}$** indicates a defect (e.g., blocking call, unoptimized math).
* **Best Practice:** Keep logic purely mathematical. Offload high-frequency PID loops to motor controller hardware wherever possible.

### 3.3 `isFinished()`
* **Role:** Evaluates whether the command has achieved its exit condition.
* **Cost Target:** `< 0.05 ms`.
* **Rule:** Must be pure and idempotent. Do not write to actuators or mutate external subsystem state inside this method.

### 3.4 `end(boolean interrupted)`
* **Role:** Safe teardown when the command completes naturally or is cancelled/interrupted.
* **Safety Mandate:** Always neutralize actuators (e.g., set voltage to $0\text{ V}$, engage brake mode, or hand off control to a safe default command).
* **Cost Target:** `< 0.2 ms`.

---

## 4. Zero-Allocation & Garbage Collection (GC) Mitigation

The roboRIO’s ARM Cortex-A9 processor has constrained memory bandwidth. Heap churn triggers the Java Virtual Machine’s garbage collector, resulting in Stop-The-World (STW) pauses ranging from **15 ms to 80 ms** that guarantee loop overruns and frame drops.

### 4.1 Prohibited Java Patterns in the Hot Path
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

### 4.2 WPILib-Specific Allocation Traps
* **Geometry and Math Instantiation:** Avoid calling factory methods or constructors that instantiate `Pose2d`, `Translation2d`, `Rotation2d`, or `Transform2d` every 20 ms. Pre-allocate these objects and update internal fields or pass scalar values.
* **Dynamic Command Factories in Periodic:** Avoid calling command factories (e.g., `Commands.runOnce(...)` or inline sequences) inside `periodic()` or `execute()`. Bind commands once in `configureBindings()` or evaluate triggers using state machines.

---

## 5. Subsystem & Hardware Optimization

### 5.1 CAN Bus Optimization & Status Frames
Bus utilization exceeding **70–80%** leads to frame drops, delayed sensor data, and scheduler latency.
* **Optimize CAN Status Frame Frequencies:**
  * Configure high-frequency updates (10–20 ms) *only* for signals essential to immediate loop control (e.g., primary encoder position/velocity).
  * Demote telemetry signals that are not actively used for control (e.g., motor temperatures, bus voltages, fault codes) to 100–250 ms.
* **Non-Blocking Status Queries:** Use vendor asynchronous signal polling (such as CTRE Phoenix 6 `StatusSignal.refresh()` / `waitForUpdate(0)`) rather than blocking synchronous getters.

### 5.2 Control Distribution: Onboard vs. RoboRIO
* **Recommended (Onboard Control):** Execute high-frequency feedback loops (1 kHz PID, feedforward, Motion Magic / Smart Motion) directly on the motor controller hardware (Talon FX, Spark Max/Flex).
* **RoboRIO Control:** When control must reside on the roboRIO (e.g., multi-sensor fusion, trajectory followers), use deterministic algorithms such as `SimpleMotorFeedforward` and `ProfiledPIDController` without intermediate object creation.

---

## 6. Implementation Architecture Example

The following example demonstrates pre-allocated buffers, zero allocations in the cyclic path, and direct primitive telemetry publishing:

```java
package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
    private static final int MODULE_COUNT = 4;

    // 1. Pre-allocate SwerveModulePosition and internal Rotation2d instances once
    private final SwerveModulePosition[] cachedPositions = new SwerveModulePosition[MODULE_COUNT];

    // 2. Pre-allocated primitive buffers to ingest hardware readings without allocations
    private final double[] cachedDistancesMeters = new double[MODULE_COUNT];
    private final double[] cachedAnglesRadians = new double[MODULE_COUNT];

    // 3. Pre-allocated NetworkTable publishers (direct primitive writes)
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
        // Set up CAN frame rates, motor inversions, and current limits here
    }

    @Override
    public void periodic() {
        // Ingest hardware measurements directly into primitive buffers
        readHardwareMeasurements(cachedDistancesMeters, cachedAnglesRadians);

        // Update pre-allocated module positions in-place
        for (int i = 0; i < MODULE_COUNT; i++) {
            cachedPositions[i].distanceMeters = cachedDistancesMeters[i];
            // Mutate existing pre-allocated Rotation2d without creating new objects
            cachedPositions[i].angle = cachedPositions[i].angle.minus(cachedPositions[i].angle)
                .plus(Rotation2d.fromRadians(cachedAnglesRadians[i])); // Note: WPILib 2024+ mutable angle setters or cached lookup tables preferred
        }

        // Primitive telemetry publication (zero heap garbage generated)
        speedPublisher.set(getForwardVelocity());
    }

    private void readHardwareMeasurements(double[] distancesOut, double[] anglesOut) {
        for (int i = 0; i < MODULE_COUNT; i++) {
            distancesOut[i] = readModuleDistanceMeters(i);
            anglesOut[i] = readModuleAngleRadians(i);
        }
    }

    private double readModuleDistanceMeters(int moduleIndex) {
        return 0.0; // Raw sensor reading
    }

    private double readModuleAngleRadians(int moduleIndex) {
        return 0.0; // Raw sensor reading in radians
    }

    public double getForwardVelocity() {
        return 0.0;
    }
}
```

---

## 7. Diagnostics & Profiling Checklist

Before deploying code to competition robots, verify system health using this checklist:

* [ ] **Zero Overrun Logs:** The Driver Station log shows zero recurring `Loop time of 0.020000s overrun` warnings during continuous match simulations.
* [ ] **Target Total Active Execution Time:** Steady-state robot execution completes in **$< 12.0\text{ ms}$**, guaranteeing at least **$8.0\text{ ms}$** of safety margin per 20 ms frame.
* [ ] **Per-Routine Timing Compliance:** Individual command `execute()` passes maintain **$< 0.5\text{ ms}$**; no subsystem `periodic()` or command exceeds the **$2.5\text{ ms}$** diagnostic threshold.
* [ ] **Zero Steady-State Heap Allocation:** VisualVM, JProfiler, or the roboRIO profiling agent shows memory allocation rates near $0\text{ B/sec}$ in disabled and teleoperated idle states.
* [ ] **No `new` Keywords in Hot Paths:** Verified that no `new` keywords or continuous factory allocations exist inside `periodic()`, `execute()`, or `isFinished()` methods across all subsystems and commands.
* [ ] **CAN Bus Health:** CAN bus utilization remains consistently below **70%** under peak actuation loads.
* [ ] **Clean Command Requirements:** All active commands explicitly declare required subsystems via `addRequirements(...)` to prevent conflicting actuator writes in the same loop cycle.