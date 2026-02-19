package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
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

  private static final String LIMELIGHT_C = Hardware.LIMELIGHT_C;
  // hub pose blue X: 4.625m, Y: 4.035m
  // hub pose red X: 11.915m, Y: 4.035m
  public static final Transform3d COMP_BOT_LEFT_CAMERA =
      new Transform3d(
          0.114,
          0.368,
          0.235,
          new Rotation3d(0, Units.degreesToRadians(8), Units.degreesToRadians(90)));
  public static final Transform3d COMP_BOT_FRONT_CAMERA =
      new Transform3d(0.267, -0.051, 0.451, new Rotation3d(0, Units.degreesToRadians(15), 0));

  private final Field2d robotField;
  private final FieldObject2d rawVisionFieldObject;

  private BooleanSubscriber disableVision;
  private final LLCamera CCamera = new LLCamera(LIMELIGHT_C);

  private final StructPublisher<Pose3d> fieldPose3dEntry =
      NetworkTableInstance.getDefault()
          .getStructTopic("vision/fieldPose3d", Pose3d.struct)
          .publish();
  private final StructPublisher<Pose3d> rawFieldPose3dEntryB =
      NetworkTableInstance.getDefault()
          .getStructTopic("vision/rawFieldPose3dLeft", Pose3d.struct)
          .publish();
  public static final StructPublisher<Pose3d> betaBotLeftCameraViewEntry =
      NetworkTableInstance.getDefault()
          .getStructTopic("vision/betaBotLeftCameraView", Pose3d.struct)
          .publish();
  public static final StructPublisher<Pose3d> compBotLeftCameraViewEntry =
      NetworkTableInstance.getDefault()
          .getStructTopic("vision/compBotLeftCameraView", Pose3d.struct)
          .publish();
  public static final StructPublisher<Pose3d> compBotFrontCameraViewEntry =
      NetworkTableInstance.getDefault()
          .getStructTopic("vision/compBotFrontCameraView", Pose3d.struct)
          .publish();

  // state
  private double lastTimestampSeconds = 0;
  private Pose2d lastFieldPose = null;
  private double distance = 0;
  private double tagAmbiguity = 0;
  // meters
  private static final double HEIGHT_TOLERANCE = 0.15;
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
    RawFiducial[] rawFiducialsC = CCamera.getRawFiducials();
    // DataLogManager.log("got raw fiducials");
    if (rawFiducialsC != null) {
      if (rawFiducialsC.length != 1) {
        BetterPoseEstimate estimatemt1 = CCamera.getBetterPoseEstimate();
        for (RawFiducial rf : rawFiducialsC) {
          // DataLogManager.log("processing raw fiducials");
          processLimelight(estimatemt1, rawFieldPose3dEntryB, rf);
        }
      } else {
        BetterPoseEstimate estimatemt2 = CCamera.getPoseEstimateMegatag2();
        for (RawFiducial rf : rawFiducialsC) {
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
          || lastFieldPose != null && lastFieldPose.equals(fieldPose3d.toPose2d())) {
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
    int C = CCamera.getNumTargets();
    return C;
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

  public boolean isViewFinder() {
    return "viewfinder".equals(CCamera.getPipeline());
  }

  public Pose2d getLastVisionPose2d() {
    return lastFieldPose;
  }

  public void updateSim() {
    Pose2d robotPose2d = drivetrain.getState().Pose;
    Pose3d robotPose3d = new Pose3d(robotPose2d);
    compBotLeftCameraViewEntry.set(robotPose3d.transformBy(COMP_BOT_LEFT_CAMERA));
    compBotFrontCameraViewEntry.set(robotPose3d.transformBy(COMP_BOT_FRONT_CAMERA));
  }
}
