package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
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
import frc.robot.util.robotType.RobotType;

public class VisionSubsystem extends SubsystemBase {
  // Limelight names must match your NT names

  private static final String LIMELIGHT_A = Hardware.LIMELIGHT_A;
  private static final String LIMELIGHT_B = Hardware.LIMELIGHT_B;
  private static final String LIMELIGHT_C = Hardware.LIMELIGHT_C;
  public boolean limelightaOnline = false;
  public boolean limelightbOnline = false;
  public boolean limelightcOnline = false;

  // hub pose blue X: 4.625m, Y: 4.035m
  // hub pose red X: 11.915m, Y: 4.035m
  private static final Transform3d COMP_BOT_LEFT_CAMERA =
      new Transform3d(
          0.114,
          0.368,
          0.235,
          new Rotation3d(0, Units.degreesToRadians(8), Units.degreesToRadians(90)));
  private static final Transform3d COMP_BOT_FRONT_CAMERA =
      new Transform3d(0.267, -0.051, 0.451, new Rotation3d(0, Units.degreesToRadians(15), 0));

  private final Field2d robotField;
  private final FieldObject2d rawVisionFieldObject;

  private BooleanSubscriber disableVision;
  private final LLCamera ACamera = new LLCamera(LIMELIGHT_A);
  private final LLCamera BCamera = new LLCamera(LIMELIGHT_B);
  private final LLCamera CCamera = new LLCamera(LIMELIGHT_C);

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
  private final StructPublisher<Pose3d> rawFieldPose3dEntryC =
      NetworkTableInstance.getDefault()
          .getStructTopic("vision/rawFieldPose3dC", Pose3d.struct)
          .publish();
  private final StructPublisher<Pose3d> compBotLeftCameraViewEntry =
      NetworkTableInstance.getDefault()
          .getStructTopic("vision/compBotLeftCameraView", Pose3d.struct)
          .publish();
  private final StructPublisher<Pose3d> compBotFrontCameraViewEntry =
      NetworkTableInstance.getDefault()
          .getStructTopic("vision/compBotFrontCameraView", Pose3d.struct)
          .publish();

  // state
  private double lastTimestampSeconds = 0;
  private double timestampSeconds = 0;
  private Pose2d lastFieldPose = null;
  private double distance = 0;
  private double tagAmbiguity = 0;
  // meters
  private static final double HEIGHT_TOLERANCE = 0.15;
  private static final double DISTANCE_TOLERANCE = 1.0;
  // degrees
  private static final double ROTATION_TOLERANCE = 12;
  private CommandSwerveDrivetrain drivetrain;
  private Pose3d drivePose3d;
  // vision
  private Pose3d fieldPose3d;
  private boolean pose_bad = false;

  public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;

    robotField = new Field2d();
    SmartDashboard.putData(robotField);
    rawVisionFieldObject = robotField.getObject("RawVision");
    SmartDashboard.putNumber("/vision/Last timestamp", getLastTimestampSeconds());
    SmartDashboard.putNumber("vision/Num targets", getNumTargets());
    SmartDashboard.putNumber("vision/april tag distance meters", getDistanceToTarget());
    SmartDashboard.putNumber("vision/time since last reading", getTimeSinceLastReading());
    SmartDashboard.putNumber("vision/tag ambiguity", getTagAmbiguity());
    var nt = NetworkTableInstance.getDefault();
    disableVision = nt.getBooleanTopic("/vision/disablevision").subscribe(false);
  }

  public void update() {
    limelightaOnline = isLimeLightOnline(LIMELIGHT_A);
    limelightbOnline = isLimeLightOnline(LIMELIGHT_B);
    limelightcOnline = isLimeLightOnline(LIMELIGHT_C);
    if (!RobotType.isAlpha()) {
      processCamera(ACamera, limelightaOnline, rawFieldPose3dEntryA);
      processCamera(BCamera, limelightbOnline, rawFieldPose3dEntryB);
      updateCameraView(drivePose3d);
    }

    if (RobotType.isAlpha()) {
      processCamera(CCamera, limelightcOnline, rawFieldPose3dEntryC);
    }
  }

  private void processCamera(
      LLCamera camera, boolean cameraOnline, StructPublisher<Pose3d> rawFieldPose3dEntry) {
    if (cameraOnline) {
      RawFiducial[] rawFiducials = camera.getRawFiducials();
      if (rawFiducials != null) {
        if (rawFiducials.length != 1) {
          // Multi tag pose Estimation
          processLimelight(camera.getBetterPoseEstimate(), rawFieldPose3dEntry);
          for (RawFiducial rf : rawFiducials) {
            processTags(rf);
          }
        } else {
          // Single tag pose Estimation
          processLimelight(camera.getPoseEstimateMegatag2(), rawFieldPose3dEntry);
          for (RawFiducial rf : rawFiducials) {
            processTags(rf);
          }
        }
      }
    }
  }

  private void processLimelight(
      BetterPoseEstimate estimate, StructPublisher<Pose3d> rawFieldPoseEntry) {
    if (getDisableVision()) {
      return;
    }

    if (estimate != null) {
      if (estimate.tagCount <= 0) {
        return;
      }

      timestampSeconds = estimate.timestampSeconds;
      drivePose3d = new Pose3d(drivetrain.getState().Pose);
      fieldPose3d = estimate.pose3d;
      pose_bad = false;
      rawFieldPoseEntry.set(fieldPose3d);

      if (!MathUtil.isNear(0, fieldPose3d.getZ(), HEIGHT_TOLERANCE)
          || !MathUtil.isNear(
              0, fieldPose3d.getRotation().getX(), Units.degreesToRadians(ROTATION_TOLERANCE))
          || !MathUtil.isNear(
              0, fieldPose3d.getRotation().getY(), Units.degreesToRadians(ROTATION_TOLERANCE))
          || lastFieldPose != null && lastFieldPose.equals(fieldPose3d.toPose2d())
          || lastFieldPose != null
              && !(Math.abs(
                      getDistanceToTargetViaPoseEstimation(
                          drivePose3d.toPose2d(), fieldPose3d.toPose2d()))
                  < DISTANCE_TOLERANCE)) {
        pose_bad = true;
        // DataLogManager.log(("pose bad");
      }

      if (!pose_bad) {
        // use this instead of .addVisionMeasurement() because the limelight hardware is good enough
        // to not need kalman filtering
        // drivetrain.addVisionMeasurement(fieldPose3d.toPose2d(), timestampSeconds,
        // VecBuilder.fill(0.1, 0.1, 99999));
        drivetrain.resetTranslation(fieldPose3d.getTranslation().toTranslation2d());
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

  private void processTags(RawFiducial rf) {
    var tagPose = AllianceUtils.FIELD_LAYOUT.getTagPose(rf.id);
    if (tagPose.isEmpty()) {
      DriverStation.reportWarning(
          "Vision: Received pose for tag ID " + rf.id + " which is not in the field layout.",
          false);
      return;
    }
    this.distance = getDistanceToTargetViaPoseEstimation(lastFieldPose, tagPose.get().toPose2d());
    this.tagAmbiguity = rf.ambiguity;
  }

  public int getNumTargets() {
    if (!RobotType.isAlpha()) {
      int A = ACamera.getNumTargets();
      int B = BCamera.getNumTargets();
      return A + B;
    }
    if (RobotType.isAlpha()) {
      return CCamera.getNumTargets();
    }
    return 0;
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
    if (yourPose == null || targetPose == null) {
      return 0;
    }
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

  private void updateCameraView(Pose3d robotPose3d) {
    compBotLeftCameraViewEntry.set(robotPose3d.transformBy(COMP_BOT_LEFT_CAMERA));
    compBotFrontCameraViewEntry.set(robotPose3d.transformBy(COMP_BOT_FRONT_CAMERA));
  }

  public boolean isLimeLightOnline(String name) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
    if (table == null) {
      return false;
    }
    // tl = timestamp, tv = valid target (supossedly tv updates every ll frame)
    return table.getEntry("tv").getLastChange() > 0;
  }
}
