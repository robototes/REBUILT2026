package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Controls;
import frc.robot.Hardware;
import frc.robot.Subsystems;
import frc.robot.util.BetterPoseEstimate;
import frc.robot.util.LLCamera;
import frc.robot.util.LimelightHelpers.RawFiducial;

public class VisionSubsystem extends SubsystemBase {
  // Limelight names must match your NT names

  private static final String LIMELIGHT_B = Hardware.LIMELIGHT_B;
  private static double limelightbTX;
  private final Subsystems subsystems = new Subsystems();
  // Deviations
  private static final Vector<N3> STANDARD_DEVS =
      VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(20));
  private static final Vector<N3> DISTANCE_SC_STANDARD_DEVS =
      VecBuilder.fill(1, 1, Units.degreesToRadians(50));
  private final SwerveRequest.FieldCentricFacingAngle autoOrient =
      new SwerveRequest.FieldCentricFacingAngle();

  // AprilTag field layout for 2025
  private static final AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  private final Controls controls = new Controls(subsystems);
  double lastBestDistance = 0;

  private final DrivebaseWrapper drivebaseWrapper;

  private final Field2d robotField;
  private final FieldObject2d rawVisionFieldObject;

  private final GenericSubscriber disableVision = null;
  private final LLCamera BCamera = new LLCamera(LIMELIGHT_B);

  private final StructPublisher<Pose3d> fieldPose3dEntry =
      NetworkTableInstance.getDefault()
          .getStructTopic("vision/fieldPose3d", Pose3d.struct)
          .publish();
  private final StructPublisher<Pose3d> rawFieldPose3dEntryLeft =
      NetworkTableInstance.getDefault()
          .getStructTopic("vision/rawFieldPose3dLeft", Pose3d.struct)
          .publish();
  private final StructPublisher<Pose3d> rawFieldPose3dEntryRight =
      NetworkTableInstance.getDefault()
          .getStructTopic("vision/rawFieldPose3dRight", Pose3d.struct)
          .publish();
  public static double offsetCorrection = 0;

  // state
  private double lastTimestampSeconds = 0;
  private Pose2d lastFieldPose = new Pose2d(-1, -1, new Rotation2d());
  private double distance = 0;
  private double tagAmbiguity = 0;
  private RawFiducial closestRawFiducial;

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
    SmartDashboard.putBoolean("Disable Vision", disableVision.get().getBoolean());
  }

  public void update() {
    RawFiducial[] rawFiducialsB = BCamera.getRawFiducials();
    if (rawFiducialsB != null) {
      for (RawFiducial rf : rawFiducialsB) {
        processLimelight(BCamera, rawFieldPose3dEntryLeft, rf);
      }
    }
  }

  private void processLimelight(
      LLCamera camera, StructPublisher<Pose3d> rawFieldPoseEntry, RawFiducial rf) {
    if (disableVision.getBoolean(false)) {
      return;
    }

    processFiducials(rf);
    BetterPoseEstimate estimate = camera.getBetterPoseEstimate();

    if (estimate != null) {
      if (estimate.tagCount <= 0) {
        return;
      }

      double timestampSeconds = estimate.timestampSeconds;
      Pose3d fieldPose3d = estimate.pose3d;
      boolean pose_bad = false;
      rawFieldPoseEntry.set(fieldPose3d);
      limelightbTX = camera.getTX();

      if (!MathUtil.isNear(0, fieldPose3d.getZ(), 0.10)
          || !MathUtil.isNear(0, fieldPose3d.getRotation().getX(), Units.degreesToRadians(8))
          || !MathUtil.isNear(0, fieldPose3d.getRotation().getY(), Units.degreesToRadians(8))) {
        pose_bad = true;
      }

      // // // filter invalid tags by alliance reef
      // if (rf.id >= 0 && BadAprilTagDetector(rf)) {
      // return;
      // }
      if (!pose_bad) {

        drivebaseWrapper.addVisionMeasurement(
            fieldPose3d.toPose2d(),
            timestampSeconds,
            // start with STANDARD_DEVS, and for every
            // meter of distance past 1 meter,
            DISTANCE_SC_STANDARD_DEVS.times(Math.max(0, this.distance - 1)).plus(STANDARD_DEVS));
        robotField.setRobotPose(drivebaseWrapper.getEstimatedPosition());
      }
      if (timestampSeconds > lastTimestampSeconds) {
        if (!pose_bad) {
          fieldPose3dEntry.set(fieldPose3d);
          lastFieldPose = fieldPose3d.toPose2d();
          rawVisionFieldObject.setPose(lastFieldPose);
        }
        lastTimestampSeconds = timestampSeconds;
      }
    }
  }

  public int getNumTargets() {
    int B = BCamera.getNumTargets();
    return B;
  }

  public double getTargetAutoOrientAngle() {

    double correctedDegree = limelightbTX + offsetCorrection;

    // Combine with current heading
    return lastFieldPose.getRotation().getDegrees() + correctedDegree;
  }

  // you have to reset by putting out new swerve request after this to start driving again
  private Command setTargetAngle() {
    return runOnce(
        () -> {
          subsystems.drivebaseSubsystem.setControl(
              autoOrient
                  .withVelocityX(controls.getDriveX())
                  .withVelocityY(controls.getDriveY())
                  .withTargetDirection(Rotation2d.fromDegrees(getTargetAutoOrientAngle())));
        });
  }

  public Command moveToRadian() {
    return setTargetAngle();
  }

  private void processFiducials(RawFiducial rf) {
    // distance to closest fiducial
    if (fieldLayout.getTagPose(rf.id).isPresent()) {
      this.distance = rf.distToRobot;
      this.tagAmbiguity = rf.ambiguity;
    }
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

  public double getTagAmbiguity() {
    return tagAmbiguity;
  }

  // private static boolean BadAprilTagDetector(LimelightHelpers.RawFiducial r) {
  //   boolean isRed = DriverStation.getAlliance().equals(Optional.of(DriverStation.Alliance.Red));
  //   boolean isBlue =
  // DriverStation.getAlliance().equals(Optional.of(DriverStation.Alliance.Blue));
  //     boolean isRedReef = 6 <= r.id && r.id <= 11;
  //     boolean isBlueReef = 17 <= r.id && r.id <= 22;
  //     boolean isValid = isBlueReef && !isRed || isRedReef && !isBlue;
  //     if (!isValid) {
  //       return true;
  //     }
  //   return false;
  // }
}
