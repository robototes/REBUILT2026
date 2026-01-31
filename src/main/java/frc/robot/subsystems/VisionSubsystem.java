package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;
import frc.robot.util.AllianceUtils;
import frc.robot.util.BetterPoseEstimate;
import frc.robot.util.LLCamera;
import frc.robot.util.LimelightHelpers.RawFiducial;

public class VisionSubsystem extends SubsystemBase {
  // Limelight names must match your NT names

  private static final String LIMELIGHT_C = Hardware.LIMELIGHT_C;
  // hub pose blue X: 4.536m, Y: 4.053m
  // hub pose red X: 11.950m, Y: 4.105m,
  // Deviations
  private static final Vector<N3> STANDARD_DEVS =
      VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(20));
  private static final Vector<N3> DISTANCE_SC_STANDARD_DEVS =
      VecBuilder.fill(1, 1, Units.degreesToRadians(50));

  private final DrivebaseWrapper drivebaseWrapper;

  private final Field2d robotField;
  private final FieldObject2d rawVisionFieldObject;

  private boolean disableVision;
  private final LLCamera CCamera = new LLCamera(LIMELIGHT_C);

  private final StructPublisher<Pose3d> fieldPose3dEntry =
      NetworkTableInstance.getDefault()
          .getStructTopic("vision/fieldPose3d", Pose3d.struct)
          .publish();
  private final StructPublisher<Pose3d> rawFieldPose3dEntryB =
      NetworkTableInstance.getDefault()
          .getStructTopic("vision/rawFieldPose3dLeft", Pose3d.struct)
          .publish();

  // state
  private double lastTimestampSeconds = 0;
  private Pose2d lastFieldPose = new Pose2d(-1, -1, new Rotation2d());
  private double distance = 0;
  private double tagAmbiguity = 0;

  public VisionSubsystem(DrivebaseWrapper drivebaseWrapper) {
    this.drivebaseWrapper = drivebaseWrapper;

    robotField = new Field2d();
    SmartDashboard.putData(robotField);
    rawVisionFieldObject = robotField.getObject("RawVision");
    SmartDashboard.putNumber("Last timestamp", getLastTimestampSeconds());
    SmartDashboard.putNumber("Num targets", getNumTargets());
    SmartDashboard.putNumber("april tag distance meters", getDistanceToTarget());
    SmartDashboard.putNumber("time since last reading", getTimeSinceLastReading());
    SmartDashboard.putNumber("tag ambiguity", getTagAmbiguity());
    disableVision = SmartDashboard.getBoolean("Disable Vision", false);
  }

  public void update() {
    disableVision = SmartDashboard.getBoolean("Disable Vision", false);
    // System.out.println("updating");
    RawFiducial[] rawFiducialsB = CCamera.getRawFiducials();
    // System.out.println("got raw fiducials");
    if (rawFiducialsB != null) {
      for (RawFiducial rf : rawFiducialsB) {
        // System.out.println("processing raw fiducials");
        processLimelight(CCamera, rawFieldPose3dEntryB, rf);
      }
    }
  }

  private void processLimelight(
      LLCamera camera, StructPublisher<Pose3d> rawFieldPoseEntry, RawFiducial rf) {
    if (disableVision) {
      return;
    }

    // System.out.println("processed a raw fiducial");
    BetterPoseEstimate estimate = camera.getBetterPoseEstimate();
    // System.out.println("got a pose estimate");

    if (estimate != null) {
      if (estimate.tagCount <= 0) {
        // System.out.println("no tags");
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
      //   System.out.println("got new data");

      if (!MathUtil.isNear(0, fieldPose3d.getZ(), 0.10)
          || !MathUtil.isNear(0, fieldPose3d.getRotation().getX(), Units.degreesToRadians(8))
          || !MathUtil.isNear(0, fieldPose3d.getRotation().getY(), Units.degreesToRadians(8))) {
        pose_bad = true;
        // System.out.println("pose bad");
      }

      if (!pose_bad) {

        drivebaseWrapper.addVisionMeasurement(
            fieldPose3d.toPose2d(),
            timestampSeconds,
            // start with STANDARD_DEVS, and for every
            // meter of distance past 1 meter,
            // add a distance standard dev
            DISTANCE_SC_STANDARD_DEVS.times(Math.max(0, this.distance - 1)).plus(STANDARD_DEVS));
        robotField.setRobotPose(drivebaseWrapper.getEstimatedPosition());
        // System.out.println("put pose in");
      }
      if (timestampSeconds > lastTimestampSeconds) {
        if (!pose_bad) {
          fieldPose3dEntry.set(fieldPose3d);
          lastFieldPose = fieldPose3d.toPose2d();
          rawVisionFieldObject.setPose(lastFieldPose);
          //   System.out.println("updated pose");
        }
        lastTimestampSeconds = timestampSeconds;
        // System.out.println("updated time");
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
    // 0.1 millimeter
    return (double) Math.round(distance * 10000) / 10000;
  }

  public double getTagAmbiguity() {
    return tagAmbiguity;
  }

  public boolean getDisableVision() {
    return disableVision;
  }

  public boolean isViewFinder() {
    return "viewfinder".equals(CCamera.getPipeline());
  }
}
