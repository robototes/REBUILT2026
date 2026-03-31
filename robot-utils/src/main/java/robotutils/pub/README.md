# How to integrate a robot project with robot-utils

This guide covers step-by-step integration for four robot-utils capabilities:
**Dashboard**, **GroundTruthSim**, **SimLimelightProducer**, and **FaultyDriveManager**.

It is based on how Mochi2026 uses each feature.  For each section the relevant
source files are cited so you can cross-reference real usage.

---

## Prerequisites — Gradle wiring

`robot-utils` is a Gradle subproject.  Before using any of the APIs below, the
consuming robot project must declare the dependency.

1. In `settings.gradle`, include the subproject:

   ```groovy
   include ':robot-utils'
   ```

2. In the root `build.gradle`, add the implementation dependency:

   ```groovy
   dependencies {
       implementation project(':robot-utils')
   }
   ```

3. Copy robot-utils/vendordeps/photonlib.json to vendordeps/photonlib.json.

---

## 1. Dashboard

The dashboard subsystem (`DashboardManagerInterface`) provides a central hub
for publishing telemetry — poses on `Field2d` objects, strings, doubles — via
registered providers.  Providers are registered automatically by factory
methods (see GroundTruthSim section); the robot project only needs to create
the manager and call `update()`.

**Call sites in Mochi2026:**
- `RobotContainer.java` — `createDashboardManager()`, passes manager to `createGroundTruthSim()`, `addCustomRenderer()` ×4
- `Robot.java` — calls `m_dashboardManager.update()` from `robotPeriodic()`

### Step 1 — Create `RobotUtilsFactory` and the dashboard manager

In `RobotContainer` (field declarations, before any subsystems):

```java
private final RobotUtilsFactory m_robotUtilsFactory = new RobotUtilsFactory();
public final DashboardManagerInterface m_dashboardManager =
    m_robotUtilsFactory.createDashboardManager();
```

The factory is the single entry point for all robot-utils objects.

### Step 2 — Add custom renderers (sim only, optional)

In `RobotContainer`, after both `groundTruthSim` and `simLimelightProducer` are
constructed (see Sections 2 and 3), register renderers so ground-truth and
odometry poses appear on the sim debug `Field2d`:

```java
if (RobotBase.isSimulation()) {
    Field2d simDebugField = simLimelightProducer.getSimDebugField();

    // Odometry estimate on the sim debug field
    m_dashboardManager.addCustomRenderer(
        new Field2dObjectRenderer(simDebugField, DashboardConstants.kEstimatedPoseItemName),
        DashboardConstants.kGroundTruthProviderName,
        DashboardConstants.kEstimatedPoseItemName);

    // Swerve module poses on the sim debug field (array renderer, 4 modules)
    m_dashboardManager.addCustomRenderer(
        new Field2dMultipleObjectRenderer(simDebugField, DashboardConstants.kEstimatedPoseModules, 4),
        DashboardConstants.kGroundTruthProviderName,
        DashboardConstants.kEstimatedPoseModules);
}
```

The constant strings (`DashboardConstants.kGroundTruthProviderName` etc.) must
match identically the names used inside `RobotUtilsFactory` when it registers
providers, so only use the constants — never hardcode strings.

### Step 3 — Call `update()` every robot cycle

In `Robot.robotPeriodic()`, after `CommandScheduler.getInstance().run()`:

```java
m_robotContainer.m_dashboardManager.update();
```

This iterates all registered providers and pushes fresh data to NetworkTables.

---

## 2. GroundTruthSim

`GroundTruthSimInterface` tracks where the simulated robot *actually* is,
independently of the drivetrain's pose estimator.  It integrates chassis
speeds at the drivetrain's high-frequency (250 Hz) sim rate and publishes the
result for vision simulation and debug display.

**Call sites in Mochi2026:**
- `RobotContainer.java` — construction, high-frequency callback registration,
  `addCustomRenderer` calls, pose-manipulation helpers, named-command registrations
- `Robot.java` — `groundTruthSim.simulationPeriodic()`,
  `groundTruthSim.getGroundTruthPose()` in `simulationPeriodic()`

### Step 1 — Create a pose-reset consumer

When `GroundTruthSim` cycles the robot reset position it needs to push the new
pose into the drivetrain and the vision system.  Define a method in
`RobotContainer`:

```java
private void resetRobotPose(Pose2d pose) {
    drivetrain.resetPose(pose);
    if (Robot.isSimulation()) {
        groundTruthSim.resetGroundTruthPoseForSim(pose);
        simLimelightProducer.resetSimPose(pose);  // see Section 3
    }
    // reset any other stateful vision filters here
}
```

### Step 2 — Construct `GroundTruthSimInterface` via the factory

Call `createGroundTruthSim` inside the sim-only construction path.  The
factory returns `null` when `RobotBase.isSimulation()` is false, so guard all
usages:

```java
GroundTruthSimInterface groundTruthSim = m_robotUtilsFactory.createGroundTruthSim(
    Optional.of(dashboardManager),   // pass empty() to skip dashboard integration
    drivetrain,                      // SwerveDrivetrain instance
    this::resetRobotPose);           // consumer called on each pose reset
```

The factory internally creates a `GroundTruthSimDashboardProvider`, calls
`provider.init()`, and registers it under
`DashboardConstants.kGroundTruthProviderName` — so the dashboard manager
automatically receives ground truth telemetry without any additional wiring.

### Step 3 — Register the high-frequency pose integration callback

The drivetrain runs its physics at ~250 Hz.  `updateGroundTruthPose()` must be
called at that same rate so ground truth stays in sync:

```java
drivetrain.setHighFreqSimCallback(groundTruthSim::updateGroundTruthPose);
```

Register this immediately after constructing `groundTruthSim`, before the
first `simulationPeriodic()`.

### Step 4 — Call `simulationPeriodic()` every sim cycle

In `Robot.simulationPeriodic()`:

```java
if (m_robotContainer.groundTruthSim != null) {
    m_robotContainer.groundTruthSim.simulationPeriodic();
}
```

`GroundTruthSimInterface.simulationPeriodic()` publishes telemetry (estimated
vs. ground truth pose, distance between them) at the standard 50 Hz loop rate.

### Step 5 — Retrieve ground truth pose for external use

After `simulationPeriodic()` you can read the current ground truth pose:

```java
Pose2d groundTruthPose = groundTruthSim.getGroundTruthPose();
// use it to update vision sim, show on debug Field2d, etc.
```

### Step 6 — Pose reset consistency

Whenever the robot pose is reset (driver station request, auto start, etc.)
call both:

```java
groundTruthSim.resetGroundTruthPoseForSim(newPose);
// AND reset the vision sim pose history (see Section 3)
simLimelightProducer.resetSimPose(newPose);
```

### Optional — Drift and reset helpers

```java
// Inject odometry drift (ground truth unchanged; pose estimator is shifted)
groundTruthSim.injectDriftToPoseEstimate(xFrontBack, yLeftRight, rotDegrees);

// Move the ground truth pose without touching the pose estimator
groundTruthSim.injectDriftToGroundTruth(xFrontBack, yLeftRight, rotDegrees);

// Cycle through a predefined list of field reset positions
groundTruthSim.cycleResetPosition(blueAlliancePose);
```

---

## 3. SimLimelightProducer

`SimLimelightProducerInterface` simulates Limelight cameras:  it runs a
PhotonVision AprilTag simulation internally and publishes the results as
Limelight NetworkTables data so the real vision pipeline code works
unmodified in simulation.

**Call sites in Mochi2026:**
- `RobotContainer.java` — `createSimLimelightProducer()`; `getSimDebugField()`
  for custom renderers; `resetSimPose()` in the pose-reset consumer
- `Robot.java` — `simLimelightProducer.periodic()` in `robotPeriodic()`;
  `simLimelightProducer.simulationPeriodic()` in `simulationPeriodic()`

### Step 1 — Provide a `CameraInfoList`

The producer needs to know each camera's name and robot-to-camera transform.
Build a `CameraInfoList` for your robot's cameras:

```java
CameraInfoList cameras = new CameraInfoList(List.of(
    new CameraInfo("limelight-front", robotToFrontCamTransform),
    new CameraInfo("limelight-back",  robotToBackCamTransform)
));
// Each CameraInfo contains: String cameraName, Transform3d robotToCam
```

### Step 2 — Create `SimLimelightProducerInterface` via the factory

The factory returns `null` when not in simulation:

```java
SimLimelightProducerInterface simLimelightProducer =
    m_robotUtilsFactory.createSimLimelightProducer(cameras);
// null on real robot — guard all downstream usage
```

### Step 3 — Call `periodic()` from `Robot.robotPeriodic()`

This must run **before** `CommandScheduler.run()` and before the vision
pipeline reads NetworkTables, so the simulated limelight data is fresh:

```java
if (Robot.isSimulation() && simLimelightProducer != null) {
    simLimelightProducer.periodic();
}
// then: vision pipeline periodic(), CommandScheduler.run(), etc.
```

### Step 4 — Call `simulationPeriodic(groundTruthPose)` from `Robot.simulationPeriodic()`

Pass the current ground truth pose so the sim knows where to "see" AprilTags:

```java
// In Robot.simulationPeriodic():
Pose2d groundTruthPose = m_robotContainer.groundTruthSim.getGroundTruthPose();
m_robotContainer.simLimelightProducer.simulationPeriodic(groundTruthPose);
```

This updates the PhotonVision sim with the robot's actual position, not the
potentially-drifted odometry pose.

### Step 5 — Reset pose history on robot reset

Whenever the robot pose is reset, clear the vision sim's pose history so stale
measurements don't corrupt the pose estimator:

```java
simLimelightProducer.resetSimPose(newPose);
```

Call this alongside `groundTruthSim.resetGroundTruthPoseForSim(newPose)` — see
the `resetRobotPose` consumer in Section 2.

### Step 6 — Use `DrivetrainVisionPoseInfo` in the vision pipeline

`SimLimelightProducerInterface.DrivetrainVisionPoseInfo` is the data transfer
record that bridges the vision pipeline to `drivetrain.addVisionMeasurement()`.
The real vision code (`SingleCamOdometry`) creates these records from Limelight
NT data; the sim produces the same NT data, so no special handling is needed.

If you write your own vision integration class, consume the record like this:

```java
// Create from vision measurement
var info = new SimLimelightProducerInterface.DrivetrainVisionPoseInfo(
    pose, timestamp, estimationStdDevs, tagCount);

// Feed to drivetrain
drivetrain.addVisionMeasurement(info.pose(), info.timestamp(), info.estimationStdDevs());
```

### Step 7 — (Optional) Debug field

Get a `Field2d` that shows the simulated camera view and detected tags:

```java
Field2d simDebugField = simLimelightProducer.getSimDebugField();
SmartDashboard.putData("SimDebugField", simDebugField);
```

Use this with `addCustomRenderer` (see Dashboard section) to overlay odometry
estimates and module poses on the same field widget.

## 4. FaultyDriveManager

`FaultyDriveManagerInterface` injects simulated hardware faults — pull-right,
rotation drift, camera miscalibration — for autonomous testing.  It depends on
both `GroundTruthSimInterface` and `SimLimelightProducerInterface` because
some faults affect drivetrain physics (handled by ground truth) and others
affect camera position (handled by the vision sim).

**Call sites in Mochi2026:**
- `RobotContainer.java` — `createFaultyDriveManager()`, named-command
  registrations (`faulty-pull-right`, `faulty-rotate-clockwise`,
  `faulty-camera-misplaced`, plus aliases `nudge-right` and `nudge-rotate`)
  in `registerNamedCommands()`;
  `resetAllAutoSimFaults()` in `getAutonomousCommand()`

### Step 1 — Build `GroundTruthSim` and `SimLimelightProducer` first

`FaultyDriveManager` requires both objects already instantiated (see Sections
2 and 3).  Create them before calling `createFaultyDriveManager`.

### Step 2 — Create `FaultyDriveManagerInterface` via the factory

```java
FaultyDriveManagerInterface faultyDriveManager = m_robotUtilsFactory.createFaultyDriveManager(
    groundTruthSim,            // GroundTruthSimInterface (must be non-null)
    simLimelightProducer);     // SimLimelightProducerInterface (must be non-null)
```

Both arguments must be non-null — this is sim-only code, guarded by the same
`if (Robot.isSimulation())` block used to construct the other two.

### Step 3 — Wire fault triggers to PathPlanner named commands (or buttons)

Register named commands so autonomous paths can trigger faults mid-run:

```java
// Only register inside if (Robot.isSimulation()) block
NamedCommands.registerCommand("faulty-pull-right", Commands.runOnce(() ->
    faultyDriveManager.enablePullRight(true)));
NamedCommands.registerCommand("nudge-right", Commands.runOnce(() ->
    faultyDriveManager.enablePullRight(true)));

NamedCommands.registerCommand("faulty-rotate-clockwise", Commands.runOnce(() ->
    faultyDriveManager.enableRotateClockwise(true)));
NamedCommands.registerCommand("nudge-rotate", Commands.runOnce(() ->
    faultyDriveManager.enableRotateClockwise(true)));

NamedCommands.registerCommand("faulty-camera-misplaced", Commands.runOnce(() ->
    faultyDriveManager.enableCameraMisplaced(new Transform3d(0, -1.0, 0, new Rotation3d()))));
```

Use `Commands.runOnce` (not `drivetrain.runOnce`) to avoid a subsystem
requirement conflict — PathPlanner's `FollowPathCommand` already holds the
drivetrain subsystem, so a second requirement would silently prevent the
command from executing.

### Step 4 — Reset all faults after autonomous ends

In `getAutonomousCommand()` (or `autonomousExit()`), append a reset so faults
do not leak into subsequent runs:

```java
return getSelectedAutoCommand()
    .andThen(Commands.runOnce(() -> faultyDriveManager.resetAllAutoSimFaults()));
```

### Fault reference

| Method | Effect |
|--------|--------|
| `enablePullRight(true)` | Adds a rightward translation bias to forward motion |
| `enableRotateClockwise(true)` | Adds a clockwise rotation bias during turns |
| `enableCameraMisplaced(offset)` | Shifts the sim camera origin without touching the pose estimator |
| `resetAllAutoSimFaults()` | Disables all of the above |

---

## Putting it together — construction order

The dependencies between the four components dictate the order of construction:

1. **`RobotUtilsFactory`** — create once, reuse everywhere
2. **`DashboardManagerInterface`** — needed by `createGroundTruthSim`; create
   before any subsystem
3. **`GroundTruthSimInterface`** — needs the drivetrain and the pose-reset
   consumer; returns `null` on a real robot
4. **`SimLimelightProducerInterface`** — needs `CameraInfoList`; returns `null`
   on a real robot
5. **`FaultyDriveManagerInterface`** — needs both GroundTruthSim and
   SimLimelightProducer; only create after both are confirmed non-null
6. **Custom renderers** (`addCustomRenderer`) — add after `SimLimelightProducerInterface`
   is constructed so `getSimDebugField()` is available

And the lifecycle order each robot cycle:

| When | Call |
|------|------|
| `robotPeriodic()` — first thing | `simLimelightProducer.periodic()` |
| `robotPeriodic()` — after CommandScheduler | `dashboardManager.update()` |
| `simulationPeriodic()` | `groundTruthSim.simulationPeriodic()` |
| `simulationPeriodic()` | `simLimelightProducer.simulationPeriodic(groundTruthPose)` |
| On any pose reset | `groundTruthSim.resetGroundTruthPoseForSim(pose)` + `simLimelightProducer.resetSimPose(pose)` |
| After auto ends | `faultyDriveManager.resetAllAutoSimFaults()` |

## Next steps

1. Show how to copy simgui.json styles for robot-pose, and how to get robot wheel icons to show-up correctly
2. Show how to add all the sim* fault Pathplanner paths, and add them as sim-only in autologic
3. Pov-left should cycle auto start pose, and pov-right should offset groundtruth bot
