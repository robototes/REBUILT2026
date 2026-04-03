# robot-utils integration guide

It covers:
- `DashboardManagerInterface`
- `GroundTruthSimInterface`
- `SimLimelightProducerInterface`
- `FaultyDriveManagerInterface`

The reference call sites in this repo are:
- `src/main/java/frc/robot/Subsystems.java`
- `src/main/java/frc/robot/Robot.java`
- `src/main/java/frc/robot/subsystems/auto/AutoLogic.java`
- `src/main/java/frc/robot/subsystems/auto/AutoBuilderConfig.java`
- `src/main/java/frc/robot/Controls.java`

## 1) Gradle wiring

1. In `settings.gradle`:

```groovy
include ':robot-utils'
```

2. In root `build.gradle`:

```groovy
dependencies {
    implementation project(':robot-utils')
}
```

3. Ensure PhotonVision vendordep exists in the main project:

```text
vendordeps/photonlib.json
```

## 2) Centralize all pose resets

This commit expects one shared reset function that updates drivetrain, ground truth,
and vision sim together.

```java
public void resetRobotPose(Pose2d pose) {
  if (drivebaseSubsystem == null) {
    return;
  }
  drivebaseSubsystem.resetPose(pose);
  if (groundTruthSim != null) {
    groundTruthSim.resetGroundTruthPoseForSim(pose);
  }
  if (simLimelightProducer != null) {
    simLimelightProducer.resetSimPose(pose);
  }
}
```

Use this reset path everywhere instead of calling `drivebase.resetPose(...)` directly:
- PathPlanner auto reset callback (`AutoBuilderConfig.buildAuto(..., subsystems::resetRobotPose, ...)`)
- controls/manual reset actions
- any simulation helper that resets pose

## 3) Subsystems construction order

Use this order (see `Subsystems.java`):

1. `RobotUtilsFactory`
2. `DashboardManagerInterface dashboardManager = createDashboardManager()`
3. `groundTruthSim = createGroundTruthSim(Optional.of(dashboardManager), drivebase, this::resetRobotPose)`
4. If non-null, immediately sync startup pose and high-frequency update callback:

```java
groundTruthSim.resetGroundTruthPoseForSim(drivebaseSubsystem.getState().Pose);
drivebaseSubsystem.setHighFreqSimCallback(groundTruthSim::updateGroundTruthPose);
```

5. `simLimelightProducer = createSimLimelightProducer(VisionSimCameraConstants.kSimCameras)`
6. If simulation and both are non-null:

```java
faultyDriveManager = createFaultyDriveManager(groundTruthSim, simLimelightProducer);
```

7. If simulation and sim producer exists, add renderers to sim debug field:

```java
Field2d simDebugField = simLimelightProducer.getSimDebugField();
dashboardManager.addCustomRenderer(
    new Field2dObjectRenderer(simDebugField, DashboardConstants.kEstimatedPoseItemName),
    DashboardConstants.kGroundTruthProviderName,
    DashboardConstants.kEstimatedPoseItemName);
dashboardManager.addCustomRenderer(
    new Field2dMultipleObjectRenderer(simDebugField, DashboardConstants.kEstimatedPoseModules, 4),
    DashboardConstants.kGroundTruthProviderName,
    DashboardConstants.kEstimatedPoseModules);
```

## 4) Robot lifecycle call order

`Robot.robotPeriodic()`:
1. In simulation, call `simLimelightProducer.periodic()` first.
2. Run vision update.
3. Run `CommandScheduler`.
4. Call `dashboardManager.update()`.

`Robot.simulationPeriodic()`:
1. `groundTruthSim.simulationPeriodic()` (currently a lightweight hook)
2. `simLimelightProducer.simulationPeriodic(groundTruthSim.getGroundTruthPose())`

Important detail: ground-truth integration is done in the high-frequency callback via
`updateGroundTruthPose()`. `simulationPeriodic()` is not where integration happens.

## 5) Fault command behavior (including what was easy to miss)

In `AutoLogic.registerCommands(...)`, these are the actual mappings:

- `faulty-pull-right` -> `faultyDriveManager.enablePullRight(true)`
- `faulty-rotate-clockwise` -> `faultyDriveManager.enableRotateClockwise(true)`
- `faulty-camera-misplaced` -> `faultyDriveManager.enableCameraMisplaced(new Transform3d(0, -1.0, 0, new Rotation3d()))`
- `nudge-right` -> one-shot `groundTruthSim.injectDriftToGroundTruth(0, Units.inchesToMeters(12), 0)`
- `nudge-rotate` -> one-shot `groundTruthSim.injectDriftToGroundTruth(0, 0, -45)`

`nudge-right` and `nudge-rotate` are not aliases of the continuous fault toggles.

Reset faults after autonomous:

```java
selectedAuto.andThen(Commands.runOnce(subsystems::resetAllAutoSimFaults));
```

Also reset in `autonomousExit()`.

## 6) simgui.json requirements for correct visuals

For expected simulation visuals (`simgui.json`):

- Keep object names aligned with dashboard constants:
  - `EstimatedRobot`
  - `EstimatedRobotModules`
  - `GroundTruthPose` (if displayed)
- Keep camera style hidden if you do not want camera wireframes in the field widget.
- Set image paths for module/tag icons (for example `swerve_module.png`, `tag-blue.png`, `tag-green.png`).

If these names do not match the published Field2d object names, the custom styles/images will not apply.

## 7) Quick API notes

- `createGroundTruthSim(...)` and `createSimLimelightProducer(...)` return `null` on real hardware.
- `SimLimelightProducer.getSimDebugField()` is simulation-only in practice; guard its use.
- `FaultyDriveManager` assumes non-null dependencies from simulation wiring.

## 8) Copy sim Pathplanner autos to show sample vision problems

copy pathplanner autos and paths sim* files.

## 9) Add simgui.json styling for robot poses

```json
      "/SmartDashboard/VisionSystemSim-main/Sim Field": {
        "EstimatedRobotModules": {
          "image": "/Users/ido/robotics/Robototes/REBUILT2026/swerve_module.png",
          "length": 0.25,
          "width": 0.25
        },
        "Robot": {
          "arrowColor": [
            1.0,
            1.0,
            1.0,
            255.0
          ],
          "arrowWeight": 1.0,
          "color": [
            1.0,
            1.0,
            1.0,
            255.0
          ],
          "weight": 1.0
        },
        "apriltag": {
          "image": "/Users/ido/robotics/Robototes/REBUILT2026/tag-green.png"
        },
        "bottom": 1914,
        "cameras": {
          "arrows": false,
          "selectable": false,
          "style": "Hidden"
        },
        "height": 8.069275856018066,
        "left": 245,
        "right": 3942,
        "top": 118,
        "visibleTargetPoses": {
          "image": "/Users/ido/robotics/Robototes/REBUILT2026/tag-blue.png"
        },
        "width": 16.54119300842285,
        "window": {
          "visible": true
        }
      }
```

## 10) Add joystick button to reset pose to start of auto, and add drift

```json
    if (RobotBase.isSimulation()) {
      // povRight - Offsets physical robot from robot estimate pose
      driverController
          .povRight()
          .onTrue(
              Commands.runOnce(
                  () -> {
                    if (s.groundTruthSim != null) {
                      // Random translation up to 0.5 m in a random direction, random rotation sign
                      double angle = Math.random() * 2 * Math.PI;
                      double xFrontBack = 0.5 * Math.cos(angle);
                      double yLeftRight = 0.5 * Math.sin(angle);
                      double dtheta = 15.0 * (Math.random() > 0.5 ? 1 : -1);
                      s.groundTruthSim.injectDriftToGroundTruth(xFrontBack, yLeftRight, dtheta);
                    }
                  }));

      // povLeft - Resets the robot pose to auto start location
      driverController
          .povLeft()
          .onTrue(
              Commands.runOnce(
                  () -> {
                    if (s.groundTruthSim != null) {
                      s.groundTruthSim.cycleResetPosition(blueHub);
                    }
                  }));
    }
```

## Next steps

4. Show vision poses
4.5. Copy the new vision color styles for the vision pose in simgui.json into this README
5. Fix joystick for sim
