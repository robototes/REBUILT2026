package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
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
import frc.robot.util.tuning.NtTunableDouble;

public class VisionSubsystem extends SubsystemBase {
  private static final String LIMELIGHT_A = Hardware.LIMELIGHT_A;
  private static final String LIMELIGHT_B = Hardware.LIMELIGHT_B;
  private static final String LIMELIGHT_C = Hardware.LIMELIGHT_C;
  public boolean limelightaOnline = false;
  public boolean limelightbOnline = false;
  public boolean limelightcOnline = false;
  private Matrix<N3, N1> stdDevs = null;
  private static NtTunableDouble A_XY_MT2 = new NtTunableDouble("/vision/A_XY_MT2", 0.07);
  private static NtTunableDouble A_XY_MT1 = new NtTunableDouble("/vision/A_XY_MT1", 0.09);
  private static NtTunableDouble P_XY = new NtTunableDouble("/vision/P_XY", 1.4);

  private static class VisionConstants {
    private static final double STD_DEVS_MT1_THETA = Math.PI / 60;

    // Ambiguity gating
    private static final double MAX_AMBIGUITY = 0.4;

    // Range gates
    private static final double MAX_DISTANCE_MT1 = 2.0;
    private static final double MAX_DISTANCE_MT2 = 5.0;

    // Velocity sanity (alpha bot only)
    private static final double MAX_XY_VELO_ALPHA = 2.0;
    private static final double MAX_TURN_VELO_ALPHA = 0.2;

    // Pose 3D sanity
    private static final double STALENESS_THRESHOLD = 1.0;
    private static final double HEIGHT_TOLERANCE = 0.15;
    private static final double ROTATION_TOLERANCE = 12.0;

    private static final double FIELD_X_MAX = AllianceUtils.FIELD_LAYOUT.getFieldLength();
    private static final double FIELD_Y_MAX = AllianceUtils.FIELD_LAYOUT.getFieldWidth();
    private static final double MAX_VISION_IMPLIED_SPEED = 6.0; // m/s
    private static final double SPREAD_REJECT = 0.50; // meters RMS
    private static final double SPREAD_INFLATE_START = 0.10; // meters RMS
    private static final double RESET_MAX_AMBIGUITY = 0.15;
    private static final int RESET_MIN_TAGS = 2;
    private static final double FIELD_MARGIN = 0.5;
  }

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

  private double lastTimestampSeconds = 0;
  private Pose2d lastFieldPose = null;
  private CommandSwerveDrivetrain drivetrain;

  private record VisionPoseTracking(
      SwerveDriveState swerveState, ChassisSpeeds swerveSpeeds, Pose3d drivePose3d) {}

  private VisionPoseTracking visionPoseTracking;

  public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;

    robotField = new Field2d();
    SmartDashboard.putData(robotField);
    rawVisionFieldObject = robotField.getObject("RawVision");
    SmartDashboard.putNumber("/vision/Last timestamp", getLastTimestampSeconds());
    SmartDashboard.putNumber("/vision/Num targets", getNumTargets());
    SmartDashboard.putNumber("/vision/time since last reading", getTimeSinceLastReading());
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
      updateCameraView(visionPoseTracking);
    }
    if (RobotType.isAlpha()) {
      processCamera(CCamera, limelightcOnline, rawFieldPose3dEntryC);
    }
  }

  private void processCamera(
      LLCamera camera, boolean cameraOnline, StructPublisher<Pose3d> rawFieldPose3dEntry) {
    if (!cameraOnline) return;
    RawFiducial[] rawFiducials = camera.getRawFiducials();
    if (rawFiducials == null) return;

    double avgAmbiguity = getAvgAmbiguity(rawFiducials);
    SwerveDriveState swerveDriveState = drivetrain.getState();
    BetterPoseEstimate mt1Estimate = camera.getBetterPoseEstimate();
    BetterPoseEstimate mt2Estimate = camera.getPoseEstimateMegatag2();
    visionPoseTracking =
        new VisionPoseTracking(
            swerveDriveState, swerveDriveState.Speeds, new Pose3d(swerveDriveState.Pose));

    processLimelight(
        mt1Estimate, rawFieldPose3dEntry, avgAmbiguity, visionPoseTracking, rawFiducials, camera);
    processLimelight(
        mt2Estimate, rawFieldPose3dEntry, avgAmbiguity, visionPoseTracking, rawFiducials, camera);
  }

  private void processLimelight(
      BetterPoseEstimate estimate,
      StructPublisher<Pose3d> rawFieldPoseEntry,
      double avgAmbiguity,
      VisionPoseTracking visionPoseTracking,
      RawFiducial[] rawFiducials,
      LLCamera camera) {

    if (estimate == null || estimate.tagCount <= 0) return;

    if (getDisableVision()) return;
    camera.setLastTimestampSeconds(estimate.timestampSeconds);

    rawFieldPoseEntry.set(estimate.pose3d);
    Pose2d visionPose2d = estimate.pose3d.toPose2d();

    if (!MathUtil.isNear(0, estimate.pose3d.getZ(), VisionConstants.HEIGHT_TOLERANCE)
        || !MathUtil.isNear(
            0,
            estimate.pose3d.getRotation().getX(),
            Units.degreesToRadians(VisionConstants.ROTATION_TOLERANCE))
        || !MathUtil.isNear(
            0,
            estimate.pose3d.getRotation().getY(),
            Units.degreesToRadians(VisionConstants.ROTATION_TOLERANCE))
        || (lastFieldPose != null && lastFieldPose.equals(visionPose2d))) {
      SmartDashboard.putString("/vision/rejectReason", "impossible-rotation or height");
      publishDiagnostics(estimate, visionPose2d);
      return;
    }

    if (RobotType.isAlpha()
        && (Math.abs(visionPoseTracking.swerveSpeeds.vxMetersPerSecond)
                > VisionConstants.MAX_XY_VELO_ALPHA
            || Math.abs(visionPoseTracking.swerveSpeeds.vyMetersPerSecond)
                > VisionConstants.MAX_XY_VELO_ALPHA
            || Math.abs(visionPoseTracking.swerveSpeeds.omegaRadiansPerSecond)
                > VisionConstants.MAX_TURN_VELO_ALPHA)) {
      SmartDashboard.putString("/vision/rejectReason", "alpha-max-speed");
      publishDiagnostics(estimate, visionPose2d);
      return;
    }

    if (!isPoseOnField(visionPose2d)) {
      SmartDashboard.putString("/vision/rejectReason", "off-field");
      publishDiagnostics(estimate, visionPose2d);
      return;
    }
    Pose2d lastvisionPose2d;
    if (estimate.isMegaTag2) {
      lastvisionPose2d = camera.getlastPoseMT2();
    } else {
      lastvisionPose2d = camera.getlastPoseMT1();
    }
    if (!isVelocityPlausible(
        visionPose2d,
        estimate.timestampSeconds,
        lastvisionPose2d,
        camera.getLastTimestampSeconds())) {
      SmartDashboard.putString("/vision/rejectReason", "velocity-implausible");
      publishDiagnostics(estimate, visionPose2d);
      return;
    }

    double spread = getMultiTagSpread(rawFiducials, estimate.pose3d, AllianceUtils.FIELD_LAYOUT);
    if (spread > VisionConstants.SPREAD_REJECT) {
      SmartDashboard.putString("/vision/rejectReason", "inter-tag-inconsistent");
      SmartDashboard.putNumber("/vision/tagSpread", spread);
      publishDiagnostics(estimate, visionPose2d);
      return;
    }

    double avgTagDist = estimate.avgTagDist;
    if (estimate.isMegaTag2) {
      stdDevs =
          getEstimationStdDevsLimelightMT2(
              avgTagDist, estimate.tagCount, avgAmbiguity, rawFiducials, camera.getName());
    } else {
      stdDevs =
          getEstimationStdDevsLimelightMT1(
              avgTagDist, estimate.tagCount, avgAmbiguity, rawFiducials, camera.getName());
    }

    if (spread > VisionConstants.SPREAD_INFLATE_START) {
      double spreadInflation = Math.pow(spread / VisionConstants.SPREAD_INFLATE_START, 2.0);
      stdDevs = stdDevs.times(spreadInflation);
    }

    maybeResetToVision(visionPose2d, avgAmbiguity, estimate.tagCount);

    drivetrain.addVisionMeasurement(
        visionPose2d, Utils.fpgaToCurrentTime(estimate.timestampSeconds), stdDevs);

    robotField.setRobotPose(drivetrain.getState().Pose);
    if (estimate.isMegaTag2) {
      camera.setlastPoseMT2(visionPose2d);
      camera.setLastTimestampSeconds(estimate.timestampSeconds);
    } else {
      camera.setlastPoseMT1(visionPose2d);
      camera.setLastTimestampSeconds(estimate.timestampSeconds);
    }

    if (estimate.timestampSeconds >= lastTimestampSeconds) {
      fieldPose3dEntry.set(estimate.pose3d);
      lastFieldPose = visionPose2d;

      rawVisionFieldObject.setPose(lastFieldPose);
      lastTimestampSeconds = estimate.timestampSeconds;
    }

    SmartDashboard.putString("/vision/rejectReason", "none");
    SmartDashboard.putNumber("/vision/tagSpread", spread);
    publishDiagnostics(estimate, visionPose2d);
  }

  /**
   * Returns true if the pose is within the field boundary plus margin. Uses field geometry as
   * ground truth — does not touch odometry.
   */
  private boolean isPoseOnField(Pose2d pose) {
    return pose.getX() >= -VisionConstants.FIELD_MARGIN
        && pose.getX() <= VisionConstants.FIELD_X_MAX
        && pose.getY() >= -VisionConstants.FIELD_MARGIN
        && pose.getY() <= VisionConstants.FIELD_Y_MAX;
  }

  private double getMultiTagSpread(
      RawFiducial[] fiducials, Pose3d reportedPose, AprilTagFieldLayout aprilTagFieldLayout) {
    if (aprilTagFieldLayout == null || fiducials == null || fiducials.length < 2) return 0.0;

    double sumSqErr = 0;
    int count = 0;
    for (RawFiducial rf : fiducials) {
      var tagPoseOpt = aprilTagFieldLayout.getTagPose(rf.id);
      if (tagPoseOpt.isEmpty()) {
        return VisionConstants.SPREAD_REJECT + 1.0;
      }
      Pose3d tagPose3d = tagPoseOpt.get();

      double distancePerPose =
          reportedPose
              .getTranslation()
              .getDistance(tagPose3d.getTranslation()); // what the pose says

      double err = distancePerPose - rf.distToRobot;
      sumSqErr += err * err;
      count++;
    }
    if (count == 0) return 0.0;
    return Math.sqrt(sumSqErr / count);
  }

  private void maybeResetToVision(Pose2d visionPose, double ambiguity, int tagCount) {
    Pose2d odomPose = drivetrain.getState().Pose;
    boolean odomOffField = !isPoseOnField(odomPose);
    boolean visionTrusted =
        ambiguity < VisionConstants.RESET_MAX_AMBIGUITY
            && tagCount >= VisionConstants.RESET_MIN_TAGS
            && isPoseOnField(visionPose);
    if (odomOffField && visionTrusted) {
      drivetrain.resetTranslation(visionPose.getTranslation());
      SmartDashboard.putString("/vision/rejectReason", "odometry-reset");
    }
  }

  private Matrix<N3, N1> getEstimationStdDevsLimelightMT1(
      double avgTagDist,
      int numOfTags,
      double avgAmbiguity,
      RawFiducial[] rawFiducials,
      String cameraName) {

    if (avgAmbiguity >= VisionConstants.MAX_AMBIGUITY) {
      return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    }
    if (numOfTags == 1 && avgTagDist > VisionConstants.MAX_DISTANCE_MT1) {
      return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    }

    double ambiguityInflation = 1.0 / Math.pow(1.0 - avgAmbiguity, 2.0);
    double harmonicSum = computeHarmonicSum(rawFiducials);
    if (harmonicSum <= 0) harmonicSum = 1.0 / (avgTagDist * avgTagDist + 1e-6);

    double xy =
        A_XY_MT1.get()
            * Math.pow(avgTagDist, P_XY.get())
            / Math.sqrt(harmonicSum)
            * ambiguityInflation;

    double theta = VisionConstants.STD_DEVS_MT1_THETA * ambiguityInflation;
    switch (cameraName) {
      case LIMELIGHT_A -> {
        SmartDashboard.putNumber("/vision/limelight-a Mt1 STD xy", xy);
        SmartDashboard.putNumber("/vision/limelight-a Mt1 STD theta", theta);
      }
      case LIMELIGHT_B -> {
        SmartDashboard.putNumber("/vision/limelight-b Mt1 STD xy", xy);
        SmartDashboard.putNumber("/vision/limelight-b Mt1 STD theta", theta);
      }
      default -> {
        SmartDashboard.putNumber("/vision/limelight-c Mt1 STD xy", xy);
        SmartDashboard.putNumber("/vision/limelight-c Mt1 STD theta", theta);
      }
    }
    return VecBuilder.fill(xy, xy, theta);
  }

  private Matrix<N3, N1> getEstimationStdDevsLimelightMT2(
      double avgTagDist,
      int numOfTags,
      double avgAmbiguity,
      RawFiducial[] rawFiducials,
      String cameraName) {

    if (avgAmbiguity >= VisionConstants.MAX_AMBIGUITY) {
      return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    }
    if (numOfTags == 1 && avgTagDist > VisionConstants.MAX_DISTANCE_MT2) {
      return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    }

    double ambiguityInflation = 1.0 / Math.pow(1.0 - avgAmbiguity, 2.0);
    double harmonicSum = computeHarmonicSum(rawFiducials);
    if (harmonicSum <= 0) harmonicSum = 1.0 / (avgTagDist * avgTagDist + 1e-6);

    double xy =
        A_XY_MT2.get()
            * Math.pow(avgTagDist, P_XY.get())
            / Math.sqrt(harmonicSum)
            * ambiguityInflation;

    switch (cameraName) {
      case LIMELIGHT_A -> {
        SmartDashboard.putNumber("/vision/limelight-a Mt2 STD xy", xy);
      }
      case LIMELIGHT_B -> {
        SmartDashboard.putNumber("/vision/limelight-b Mt2 STD xy", xy);
      }
      default -> {
        SmartDashboard.putNumber("/vision/limelight-c Mt2 STD xy", xy);
      }
    }

    return VecBuilder.fill(xy, xy, Double.MAX_VALUE);
  }

  private double computeHarmonicSum(RawFiducial[] rawFiducials) {
    if (rawFiducials == null || rawFiducials.length == 0) return 0.0;
    double sum = 0.0;
    for (RawFiducial rf : rawFiducials) {
      double dist = rf.distToRobot;
      if (dist > 0.01) sum += 1.0 / (dist * dist);
    }
    return sum;
  }

  private void publishDiagnostics(BetterPoseEstimate estimate, Pose2d visionPose2d) {
    if (estimate.timestampSeconds >= lastTimestampSeconds) {
      SmartDashboard.putNumber(
          "/vision/visionError", getVisionPoseError(visionPose2d, estimate.timestampSeconds));
      SmartDashboard.putNumber("/vision/Last timestamp", getLastTimestampSeconds());
      SmartDashboard.putNumber("/vision/Num targets", getNumTargets());
      SmartDashboard.putNumber("/vision/time since last reading", getTimeSinceLastReading());
    }
  }

  private boolean isVelocityPlausible(
      Pose2d newPose, double newTimestamp, Pose2d lastPose2d, double lastTimestampSeconds) {
    if (lastPose2d == null) return true;
    double dt = newTimestamp - lastTimestampSeconds;
    if (dt <= 0 || dt > 1.0) return true;
    double dist = newPose.getTranslation().getDistance(lastPose2d.getTranslation());
    return (dist / dt) < VisionConstants.MAX_VISION_IMPLIED_SPEED;
  }

  public int getNumTargets() {
    if (!RobotType.isAlpha()) {
      return ACamera.getNumTargets() + BCamera.getNumTargets();
    } else {
      return CCamera.getNumTargets();
    }
  }

  public double getLastTimestampSeconds() {
    return lastTimestampSeconds;
  }

  public double getTimeSinceLastReading() {
    return Timer.getFPGATimestamp() - lastTimestampSeconds;
  }

  public double getDistanceToTargetViaPoseEstimation(Pose2d yourPose, Pose2d targetPose) {
    if (yourPose == null || targetPose == null) return 0;
    double distance =
        Math.hypot(targetPose.getX() - yourPose.getX(), targetPose.getY() - yourPose.getY());
    return (double) Math.round(distance * 1000) / 1000;
  }

  public boolean getDisableVision() {
    return disableVision.get(false);
  }

  public Pose2d getLastVisionPose2d() {
    return lastFieldPose;
  }

  private void updateCameraView(VisionPoseTracking visionPoseTracking) {
    if (visionPoseTracking != null && visionPoseTracking.drivePose3d != null) {
      compBotLeftCameraViewEntry.set(
          visionPoseTracking.drivePose3d.transformBy(COMP_BOT_LEFT_CAMERA));
      compBotFrontCameraViewEntry.set(
          visionPoseTracking.drivePose3d.transformBy(COMP_BOT_FRONT_CAMERA));
    }
  }

  public boolean isLimeLightOnline(String name) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
    if (table == null) return false;
    long lastChange = table.getEntry("hb").getLastChange();
    if (lastChange == 0) return false;
    double lastChangeSecs = lastChange / 1_000_000.0;
    return (Timer.getFPGATimestamp() - lastChangeSecs) < VisionConstants.STALENESS_THRESHOLD;
  }

  public double getAvgAmbiguity(RawFiducial[] rfs) {
    if (rfs.length == 0) return 0;
    double sum = 0;
    for (RawFiducial rf : rfs) sum += rf.ambiguity;
    return sum / rfs.length;
  }

  private double getVisionPoseError(Pose2d visionPose2d, double timestampSeconds) {
    if (drivetrain == null) return 0;
    var historicPose = drivetrain.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));
    return historicPose
        .map(pose2d -> getDistanceToTargetViaPoseEstimation(pose2d, visionPose2d))
        .orElse(0.0);
  }
}
