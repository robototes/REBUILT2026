package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import frc.robot.util.AllianceUtils;
import frc.robot.util.BetterPoseEstimate;
import frc.robot.util.LLCamera;
import frc.robot.util.LimelightHelpers.RawFiducial;

public class VisionSubsystem extends SubsystemBase {
  // Limelight names must match your NT names

  private static final String LIMELIGHT_A = Hardware.LIMELIGHT_A;
  private static final String LIMELIGHT_B = Hardware.LIMELIGHT_B;
  // hub pose blue X: 4.625m, Y: 4.035m
  // hub pose red X: 11.915m, Y: 4.035m

  private final Field2d robotField;
  private final FieldObject2d rawVisionFieldObject;

  private BooleanSubscriber disableVision;
  private final LLCamera ACamera = new LLCamera(LIMELIGHT_A);
  private final LLCamera BCamera = new LLCamera(LIMELIGHT_B);

  private final StructPublisher<Pose3d> fieldPose3dEntry =
      NetworkTableInstance.getDefault()
          .getStructTopic("vision/fieldPose3d", Pose3d.struct)
          .publish();
  private final StructPublisher<Pose3d> rawFieldPose3dEntryA =
      NetworkTableInstance.getDefault()
          .getStructTopic("vision/rawFieldPose3dA", Pose3d.struct)
          .publish();
  private final StructPublisher<Pose3d> rawFieldPose3dEntryB =
      NetworkTableInstance.getDefault()
          .getStructTopic("vision/rawFieldPose3dB", Pose3d.struct)
          .publish();

  // state
  private double lastTimestampSeconds = 0;
  private Pose2d lastFieldPose = null;
  private double distance = 0;
  private double tagAmbiguity = 0;
  // meters
  private static final double HEIGHT_TOLERANCE = 0.15;
  private static final double DISTANCE_TOLERANCE = 1.5;
  // degrees
  private static final double ROTATION_TOLERANCE = 12;
  private CommandSwerveDrivetrain drivetrain;

  public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;

    robotField = new Field2d();
    SmartDashboard.putData(robotField);
    rawVisionFieldObject = robotField.getObject("RawVision");
    SmartDashboard.putNumber("Last timestamp", getLastTimestampSeconds());
    SmartDashboard.putNumber("Num targets", getNumTargets());
    SmartDashboard.putNumber("april tag distance meters", getDistanceToTarget());
    SmartDashboard.putNumber("time since last reading", getTimeSinceLastReading());
    SmartDashboard.putNumber("tag ambiguity", getTagAmbiguity());
    var nt = NetworkTableInstance.getDefault();
    disableVision = nt.getBooleanTopic("/vision/disablevision").subscribe(false);
  }

  public void update() {
    // DataLogManager.log("updating");
    RawFiducial[] rawFiducialsA = ACamera.getRawFiducials();
    RawFiducial[] rawFiducialsB = BCamera.getRawFiducials();
    // DataLogManager.log("got raw fiducials");
    if (rawFiducialsA != null) {
      if (rawFiducialsA.length != 1) {
        BetterPoseEstimate estimatemt1 = ACamera.getBetterPoseEstimate();
        for (RawFiducial rf : rawFiducialsA) {
          // DataLogManager.log("processing raw fiducials");
          processLimelight(estimatemt1, rawFieldPose3dEntryA, rf);
        }
      } else {
        BetterPoseEstimate estimatemt2 = ACamera.getPoseEstimateMegatag2();
        for (RawFiducial rf : rawFiducialsA) {
          // DataLogManager.log("processing raw fiducials");
          processLimelight(estimatemt2, rawFieldPose3dEntryA, rf);
        }
      }
    }
    if (rawFiducialsB != null) {
      if (rawFiducialsB.length != 1) {
        BetterPoseEstimate estimatemt1 = BCamera.getBetterPoseEstimate();
        for (RawFiducial rf : rawFiducialsB) {
          // DataLogManager.log("processing raw fiducials");
          processLimelight(estimatemt1, rawFieldPose3dEntryB, rf);
        }
      } else {
        BetterPoseEstimate estimatemt2 = BCamera.getPoseEstimateMegatag2();
        for (RawFiducial rf : rawFiducialsB) {
          // DataLogManager.log("processing raw fiducials");
          processLimelight(estimatemt2, rawFieldPose3dEntryB, rf);
        }
      }
    }
  }

  private void processLimelight(
      BetterPoseEstimate estimate, StructPublisher<Pose3d> rawFieldPoseEntry, RawFiducial rf) {
    if (getDisableVision()) {
      return;
    }

    if (estimate != null) {
      if (estimate.tagCount <= 0) {
        // DataLogManager.log("no tags");
        return;
      }

      double timestampSeconds = estimate.timestampSeconds;
      Pose3d fieldPose3d = estimate.pose3d;
      var tagPose = AllianceUtils.FIELD_LAYOUT.getTagPose(rf.id);
      if (tagPose.isEmpty()) {
        DriverStation.reportWarning(
            "Vision: Received pose for tag ID " + rf.id + " which is not in the field layout.",
            false);
        return;
      }
      this.distance =
          getDistanceToTargetViaPoseEstimation(fieldPose3d.toPose2d(), tagPose.get().toPose2d());
      this.tagAmbiguity = rf.ambiguity;
      boolean pose_bad = false;
      rawFieldPoseEntry.set(fieldPose3d);
      //   DataLogManager.log("got new data");

      if (!MathUtil.isNear(0, fieldPose3d.getZ(), HEIGHT_TOLERANCE)
          || !MathUtil.isNear(
              0, fieldPose3d.getRotation().getX(), Units.degreesToRadians(ROTATION_TOLERANCE))
          || !MathUtil.isNear(
              0, fieldPose3d.getRotation().getY(), Units.degreesToRadians(ROTATION_TOLERANCE))
          || lastFieldPose != null && lastFieldPose.equals(fieldPose3d.toPose2d())
          || lastFieldPose != null
              && getDistanceToTargetViaPoseEstimation(lastFieldPose, fieldPose3d.toPose2d())
                  > DISTANCE_TOLERANCE) {
        pose_bad = true;
        // DataLogManager.log(("pose bad");
      }

      if (!pose_bad) {
        // use this instead of .addVisionMeasurement() because the limelight hardware is good enough
        // to not need kalman filtering
        drivetrain.resetPose(fieldPose3d.toPose2d());
        robotField.setRobotPose(drivetrain.getState().Pose);
        // DataLogManager.log("put pose in");
      }
      if (timestampSeconds > lastTimestampSeconds) {
        if (!pose_bad) {
          fieldPose3dEntry.set(fieldPose3d);
          lastFieldPose = fieldPose3d.toPose2d();
          rawVisionFieldObject.setPose(lastFieldPose);
          //   DataLogManager.log("updated pose");
        }
        lastTimestampSeconds = timestampSeconds;
        // DataLogManager.log("updated time");
      }
    }
  }

  public int getNumTargets() {
    int A = ACamera.getNumTargets();
    int B = BCamera.getNumTargets();
    return A + B;
  }

  public double getLastTimestampSeconds() {
    return lastTimestampSeconds;
  }

  public double getTimeSinceLastReading() {
    return Timer.getFPGATimestamp() - lastTimestampSeconds;
  }

  public double getDistanceToTarget() {
    return (double) Math.round(distance * 1000) / 1000;
  }

  public double getDistanceToTargetViaPoseEstimation(Pose2d yourPose, Pose2d targetPose) {
    double distance =
        Math.hypot(targetPose.getX() - yourPose.getX(), targetPose.getY() - yourPose.getY());
    // 1 millimeter
    return (double) Math.round(distance * 1000) / 1000;
  }

  public double getTagAmbiguity() {
    return tagAmbiguity;
  }

  public boolean getDisableVision() {
    return disableVision.get(false);
  }

  public Pose2d getLastVisionPose2d() {
    return lastFieldPose;
  }
}
