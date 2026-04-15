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
import frc.robot.sim.ShowVisionOnField;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.util.AllianceUtils;
import frc.robot.util.BetterPoseEstimate;
import frc.robot.util.robotType.RobotType;
import frc.robot.util.tuning.NtTunableDouble;
import frc.robot.util.vision.LLCamera;
import frc.robot.util.vision.LimelightHelpers.RawFiducial;
import frc.robot.util.vision.VisionStdDevMap;
import frc.robot.util.vision.VisionStdDevs;

public class VisionSubsystem extends SubsystemBase {
  private static final String LIMELIGHT_A = Hardware.LIMELIGHT_A;
  private static final String LIMELIGHT_B = Hardware.LIMELIGHT_B;
  private static final String LIMELIGHT_C = Hardware.LIMELIGHT_C;

  public boolean limelightaOnline = false;
  public boolean limelightbOnline = false;
  public boolean limelightcOnline = false;

  private Matrix<N3, N1> stdDevs = null;

  // How much to reduce std devs when defense slip is detected.
  // <1.0 = trust vision more (0.5 = half the std dev = 4x the filter weight).
  // Tune this at practice with someone actively defending.
  private static NtTunableDouble DEFENSE_STD_DEV_SCALE =
      new NtTunableDouble("/vision/defenseStdDevScale", 0.5);

  private static class VisionConstants {

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

    // Field boundary
    private static final double FIELD_MARGIN = 0.5;
    private static final double FIELD_X_MAX =
        AllianceUtils.FIELD_LAYOUT.getFieldLength() + FIELD_MARGIN;
    private static final double FIELD_Y_MAX =
        AllianceUtils.FIELD_LAYOUT.getFieldWidth() + FIELD_MARGIN;

    // Normal vision-to-vision plausibility limit
    private static final double MAX_VISION_IMPLIED_SPEED = 7.0; // m/s

    // Under defense, robots can be shoved rapidly by contact.
    // The gate is widened but not eliminated — 12 m/s is still physically impossible.
    private static final double MAX_VISION_IMPLIED_SPEED_DEFENSE = 12.0; // m/s

    // Inter-tag consistency
    private static final double SPREAD_REJECT = 0.50; // meters RMS — hard reject
    private static final double SPREAD_INFLATE_START = 0.10; // meters RMS — begin inflation

    // Odometry reset thresholds
    private static final double RESET_MAX_AMBIGUITY = 0.15;
    private static final int RESET_MIN_TAGS = 2;

    // DEFENSE_SPIN_OMEGA: high rotational speed with low translation suggests
    // being spun by an opponent rather than self-powered turning.
    // DEFENSE_SPIN_MAX_TRANSLATION: upper bound on translation speed to
    // qualify the spin signal.
    // DEFENSE_DIVERGE_BASE: minimum vision-odometry gap (meters) before the
    // disagreement signal fires regardless of elapsed time.
    // DEFENSE_DRIFT_RATE: expected odometry drift per second under normal
    // wheel noise — disagreement above (rate × dt + base) is attributed to slip.
    private static final double DEFENSE_SPIN_OMEGA = 2.5; // rad/s
    private static final double DEFENSE_SPIN_MAX_TRANSLATION = 1.0; // m/s
    private static final double DEFENSE_DIVERGE_BASE = 0.30; // meters
    private static final double DEFENSE_DRIFT_RATE = 0.02; // meters per second
  }

  public static final Transform3d COMP_BOT_LEFT_CAMERA =
      new Transform3d(
          -0.076,
          0.311,
          0.274,
          new Rotation3d(0, Units.degreesToRadians(-8), Units.degreesToRadians(90)));
  public static final Transform3d COMP_BOT_FRONT_CAMERA =
      new Transform3d(0.267, -0.111, 0.471, new Rotation3d(0, Units.degreesToRadians(-15), 0));

  private final Field2d robotField;
  private final FieldObject2d rawVisionFieldObject;
  private BooleanSubscriber disableVision;
  private IntakePivot intakePivot;

  private final LLCamera ACamera = new LLCamera(LIMELIGHT_A);
  private final LLCamera BCamera = new LLCamera(LIMELIGHT_B);
  private final LLCamera CCamera = new LLCamera(LIMELIGHT_C);

  // Per-camera IDW interpolation maps — keyed on (avgTagDistance, maxAmbiguity)
  private final VisionStdDevMap aMap = new VisionStdDevMap();
  private final VisionStdDevMap bMap = new VisionStdDevMap();
  private final VisionStdDevMap cMap = new VisionStdDevMap();

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
  private ShowVisionOnField m_showVisionOnField;

  public VisionSubsystem(CommandSwerveDrivetrain drivetrain, IntakePivot intakePivot) {
    this.drivetrain = drivetrain;
    this.intakePivot = intakePivot;

    robotField = new Field2d();
    SmartDashboard.putData(robotField);
    rawVisionFieldObject = robotField.getObject("RawVision");

    SmartDashboard.putNumber("/vision/limelight-a_Last timestamp", 0);
    SmartDashboard.putNumber("/vision/limelight-b_Last timestamp", 0);
    SmartDashboard.putNumber("/vision/limelight-c_Last timestamp", 0);
    SmartDashboard.putNumber("/vision/limelight-a_Num targets", 0);
    SmartDashboard.putNumber("/vision/limelight-b_Num targets", 0);
    SmartDashboard.putNumber("/vision/limelight-c_Num targets", 0);
    SmartDashboard.putNumber("/vision/limelight-a_time since last reading", 0);
    SmartDashboard.putNumber("/vision/limelight-b_time since last reading", 0);
    SmartDashboard.putNumber("/vision/limelight-c_time since last reading", 0);

    var nt = NetworkTableInstance.getDefault();
    disableVision = nt.getBooleanTopic("/vision/disablevision").subscribe(false);

    // Camera B — columns: avgTagDist, maxAmbiguity, mt1x, mt1y, mt1theta, mt2x, mt2y
    bMap.addData(2.8, 0.2, 0.9, 1.1, 38, 0.51, 0.49);
    bMap.addData(0.4, 0.4, 0.65, 1.4, 123, 0.74, 0.77);
    bMap.addData(3.4, 0.3, 0.67, 1.54, 117, 0.9, 1.04);
    bMap.addData(3.0, 0.2, 0.85, 1.86, 94, 0.78, 1.57);
    bMap.addData(3.7, 0.5, 0.95, 1.47, 40, 0.77, 1.2);

    // Camera A — columns: avgTagDist, maxAmbiguity, mt1x, mt1y, mt1theta, mt2x, mt2y
    aMap.addData(4.4, 0.2, 0.47, 1.3, 35, 0.2, 1.27);
    aMap.addData(3.9, 0.4, 4.22, 0.96, 141, 4.0, 0.66);
    aMap.addData(2.6, 0.3, 3.2, 0.99, 101, 2.65, 0.57);
    aMap.addData(3.6, 0.2, 0.8, 1.68, 44.68, 1.03, 1.02);
    aMap.addData(3.3, 0.8, 1.55, 2.75, 76.9, 0.78, 2.15);

    // Camera C — columns: avgTagDist, maxAmbiguity, mt1x, mt1y, mt1theta, mt2x, mt2y
    cMap.addData(2.4, 0.018, 0.43, 1.26, 48, 0.38, 0.73);
    cMap.addData(2.0, 0.15, 1.1, 1.9, 104, 0.93, 1.9);
    cMap.addData(4.8, 0.089, 0.88, 3.0, 38, 0.8, 2.9);
    cMap.addData(1.3, 0.036, 0.9, 0.73, 39, 0.4, 0.56);
    cMap.addData(2.7, 0.21, 3.63, 0.6, 57.9, 3.7, 0.48);
  }

  public void update() {
    if (getDisableVision()) {
      SmartDashboard.putString("/vision/limelight-a_rejectReason", "vision-disabled");
      SmartDashboard.putString("/vision/limelight-b_rejectReason", "vision-disabled");
      SmartDashboard.putString("/vision/limelight-c_rejectReason", "vision-disabled");
      return;
    }

    limelightaOnline = isLimeLightOnline(LIMELIGHT_A);
    limelightbOnline = isLimeLightOnline(LIMELIGHT_B);
    limelightcOnline = isLimeLightOnline(LIMELIGHT_C);

    SwerveDriveState swerveDriveState = drivetrain.getState();
    visionPoseTracking =
        new VisionPoseTracking(
            swerveDriveState, swerveDriveState.Speeds, new Pose3d(swerveDriveState.Pose));

    // Compute defense state once per update cycle so all cameras see the same value.
    // isUnderDefense() uses lastFieldPose and visionPoseTracking, both of which are
    // set before any camera processes — this is intentional.
    boolean underDefense = isUnderDefense(visionPoseTracking);
    SmartDashboard.putBoolean("/vision/underDefense", underDefense);

    processCamera(
        ACamera, limelightaOnline, rawFieldPose3dEntryA, visionPoseTracking, underDefense, aMap);
    processCamera(
        BCamera, limelightbOnline, rawFieldPose3dEntryB, visionPoseTracking, underDefense, bMap);
    // uncomment if when using intake pose
    // if (intakePivot.isAtTarget(2, IntakePivot.DEPLOYED_POS)) {
    processCamera(
        CCamera, limelightcOnline, rawFieldPose3dEntryC, visionPoseTracking, underDefense, cMap);
    // }
    updateCameraView(visionPoseTracking);
  }

  private void processCamera(
      LLCamera camera,
      boolean cameraOnline,
      StructPublisher<Pose3d> rawFieldPose3dEntry,
      VisionPoseTracking visionPoseTracking,
      boolean underDefense,
      VisionStdDevMap stdDevMap) {
    if (!cameraOnline) return;
    RawFiducial[] rawFiducials = camera.getRawFiducials();
    if (rawFiducials == null) return;

    double maxAmbiguity = getMaxAmbiguity(rawFiducials);
    BetterPoseEstimate mt1Estimate = camera.getBetterPoseEstimate();
    BetterPoseEstimate mt2Estimate = camera.getPoseEstimateMegatag2();

    processLimelight(
        mt1Estimate,
        rawFieldPose3dEntry,
        maxAmbiguity,
        visionPoseTracking,
        rawFiducials,
        camera,
        underDefense,
        stdDevMap);
    processLimelight(
        mt2Estimate,
        rawFieldPose3dEntry,
        maxAmbiguity,
        visionPoseTracking,
        rawFiducials,
        camera,
        underDefense,
        stdDevMap);
  }

  private void processLimelight(
      BetterPoseEstimate estimate,
      StructPublisher<Pose3d> rawFieldPoseEntry,
      double maxAmbiguity,
      VisionPoseTracking visionPoseTracking,
      RawFiducial[] rawFiducials,
      LLCamera camera,
      boolean underDefense,
      VisionStdDevMap stdDevMap) {

    if (estimate == null || estimate.tagCount <= 0) return;

    Pose2d visionPose2d = estimate.pose3d.toPose2d();
    if (estimate.timestampSeconds == camera.getLastTimestampSeconds()) {
      publishDiagnostics(estimate, visionPose2d, camera, "stale-timestamp");
      return;
    }

    rawFieldPoseEntry.set(estimate.pose3d);

    if (RobotType.isAlpha()
        && (Math.abs(visionPoseTracking.swerveSpeeds.vxMetersPerSecond)
                > VisionConstants.MAX_XY_VELO_ALPHA
            || Math.abs(visionPoseTracking.swerveSpeeds.vyMetersPerSecond)
                > VisionConstants.MAX_XY_VELO_ALPHA
            || Math.abs(visionPoseTracking.swerveSpeeds.omegaRadiansPerSecond)
                > VisionConstants.MAX_TURN_VELO_ALPHA)) {
      publishDiagnostics(estimate, visionPose2d, camera, "alpha-max-speed");
      return;
    }

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
      publishDiagnostics(estimate, visionPose2d, camera, "impossible-rotation or height");
      return;
    }

    if (!isPoseOnField(visionPose2d)) {
      publishDiagnostics(estimate, visionPose2d, camera, "off-field");
      return;
    }

    Pose2d lastVisionPose = estimate.isMegaTag2 ? camera.getlastPoseMT2() : camera.getlastPoseMT1();
    if (!isVelocityPlausible(
        visionPose2d,
        estimate.timestampSeconds,
        lastVisionPose,
        camera.getLastTimestampSeconds(),
        underDefense)) {
      publishDiagnostics(estimate, visionPose2d, camera, "velocity-implausible");
      return;
    }

    double spread = getMultiTagSpread(rawFiducials, estimate.pose3d, AllianceUtils.FIELD_LAYOUT);
    if (spread > VisionConstants.SPREAD_REJECT) {
      SmartDashboard.putNumber("/vision/" + camera.getName() + "_tagSpread", spread);
      publishDiagnostics(estimate, visionPose2d, camera, "inter-tag-inconsistent");
      return;
    }

    double avgTagDist = estimate.avgTagDist;
    stdDevs =
        getEstimationStdDevs(
            avgTagDist,
            estimate.tagCount,
            maxAmbiguity,
            estimate.isMegaTag2,
            stdDevMap,
            camera.getName());

    if (stdDevs.get(0, 0) >= Double.MAX_VALUE) {
      publishDiagnostics(estimate, visionPose2d, camera, "ambiguity-or-range");
      return;
    }

    if (underDefense) {
      stdDevs = stdDevs.times(DEFENSE_STD_DEV_SCALE.get());
    }

    if (spread > VisionConstants.SPREAD_INFLATE_START) {
      double spreadInflation = Math.pow(spread / VisionConstants.SPREAD_INFLATE_START, 2.0);
      stdDevs = stdDevs.times(spreadInflation);
    }

    if (m_showVisionOnField != null) {
      m_showVisionOnField.showPointInTimeVisionEstimate(
          ShowVisionOnField.FieldType.SIMULATION_FIELD,
          camera.getName(),
          estimate.isMegaTag2,
          java.util.Optional.of(estimate.pose3d.toPose2d()));
    }

    maybeResetToVision(visionPose2d, maxAmbiguity, estimate.tagCount, camera.getName());

    drivetrain.addVisionMeasurement(
        visionPose2d, Utils.fpgaToCurrentTime(estimate.timestampSeconds), stdDevs);
    robotField.setRobotPose(drivetrain.getState().Pose);

    if (estimate.isMegaTag2) {
      camera.setlastPoseMT2(visionPose2d);
    } else {
      camera.setlastPoseMT1(visionPose2d);
    }
    camera.setLastTimestampSeconds(estimate.timestampSeconds);

    if (estimate.timestampSeconds >= lastTimestampSeconds) {
      fieldPose3dEntry.set(estimate.pose3d);
      lastFieldPose = visionPose2d;
      rawVisionFieldObject.setPose(lastFieldPose);
      lastTimestampSeconds = estimate.timestampSeconds;
    }

    SmartDashboard.putNumber("/vision/" + camera.getName() + "_tagSpread", spread);
    publishDiagnostics(estimate, visionPose2d, camera, "none");
  }

  private boolean isUnderDefense(VisionPoseTracking visionPoseTracking) {
    if (visionPoseTracking == null) return false;
    ChassisSpeeds speeds = visionPoseTracking.swerveSpeeds;

    double omega = Math.abs(speeds.omegaRadiansPerSecond);
    double wheelSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    boolean likelySpun =
        omega > VisionConstants.DEFENSE_SPIN_OMEGA
            && wheelSpeed < VisionConstants.DEFENSE_SPIN_MAX_TRANSLATION;

    boolean positionDiverged = false;
    if (lastFieldPose != null) {
      double timeSinceVision = Timer.getFPGATimestamp() - lastTimestampSeconds;
      Pose2d drivePose = visionPoseTracking.drivePose3d.toPose2d();
      double disagreement = lastFieldPose.getTranslation().getDistance(drivePose.getTranslation());
      double expectedDrift =
          VisionConstants.DEFENSE_DRIFT_RATE * timeSinceVision
              + VisionConstants.DEFENSE_DIVERGE_BASE;
      positionDiverged = disagreement > expectedDrift;
    }

    return likelySpun || positionDiverged;
  }

  /**
   * Returns true if the pose is within the field boundary plus margin. FIELD_X_MAX and FIELD_Y_MAX
   * already include FIELD_MARGIN on the positive side.
   */
  private boolean isPoseOnField(Pose2d pose) {
    return pose.getX() >= -VisionConstants.FIELD_MARGIN
        && pose.getX() <= VisionConstants.FIELD_X_MAX
        && pose.getY() >= -VisionConstants.FIELD_MARGIN
        && pose.getY() <= VisionConstants.FIELD_Y_MAX;
  }

  /**
   * Vision-to-vision velocity plausibility check. Uses per-camera, per-solve-mode last accepted
   * pose — not odometry. Speed limit is widened under defense contact.
   */
  private boolean isVelocityPlausible(
      Pose2d newPose,
      double newTimestamp,
      Pose2d lastPose,
      double lastTimestamp,
      boolean underDefense) {
    if (lastPose == null) return true;
    double dt = newTimestamp - lastTimestamp;
    if (dt <= 0 || dt > 1.0) return true;
    double dist = newPose.getTranslation().getDistance(lastPose.getTranslation());
    double impliedSpeed = dist / dt;
    double limit =
        underDefense
            ? VisionConstants.MAX_VISION_IMPLIED_SPEED_DEFENSE
            : VisionConstants.MAX_VISION_IMPLIED_SPEED;
    return impliedSpeed < limit;
  }

  /**
   * RMS spread of per-tag independent distance estimates vs the reported pose.
   *
   * <p>Both sides are full 3D distances: distancePerPose = 3D Euclidean from reported robot origin
   * (Z=0) to tag (Z=tagHeight) rf.distToRobot = 3D distance reported by Limelight from robot origin
   * to tag
   *
   * <p>This makes both sides consistent. No camera height constant needed because distToRobot is
   * already robot-origin referenced (Limelight applies the robot-to-camera transform internally).
   *
   * <p>Returns 0.0 for single-tag observations (nothing to compare against). Returns
   * SPREAD_REJECT+1 if any tag ID is unknown (immediate hard reject).
   */
  private double getMultiTagSpread(
      RawFiducial[] fiducials, Pose3d reportedPose, AprilTagFieldLayout aprilTagFieldLayout) {
    if (aprilTagFieldLayout == null || fiducials == null || fiducials.length < 2) return 0.0;

    double sumSqErr = 0;
    int count = 0;
    for (RawFiducial rf : fiducials) {
      var tagPoseOpt = aprilTagFieldLayout.getTagPose(rf.id);
      if (tagPoseOpt.isEmpty()) {
        // Unknown tag ID — could be a misread. Reject the whole observation.
        return VisionConstants.SPREAD_REJECT + 1.0;
      }
      double distancePerPose =
          reportedPose
              .getTranslation()
              .getDistance(tagPoseOpt.get().getTranslation()); // what the pose says
      double err = distancePerPose - rf.distToRobot;
      sumSqErr += err * err;
      count++;
    }
    if (count == 0) return 0.0;
    return Math.sqrt(sumSqErr / count);
  }

  /**
   * Resets translation only when odometry has walked off the field (unambiguous divergence) and
   * vision simultaneously passes a high-confidence bar. Heading is preserved — resetTranslation
   * avoids fighting the gyro.
   */
  private void maybeResetToVision(
      Pose2d visionPose, double maxAmbiguity, int tagCount, String cameraName) {
    Pose2d odomPose = drivetrain.getState().Pose;
    boolean odomOffField = !isPoseOnField(odomPose);
    boolean visionTrusted =
        maxAmbiguity < VisionConstants.RESET_MAX_AMBIGUITY
            && tagCount >= VisionConstants.RESET_MIN_TAGS
            && isPoseOnField(visionPose);
    if (odomOffField && visionTrusted) {
      drivetrain.resetTranslation(visionPose.getTranslation());
      SmartDashboard.putString("/vision/" + cameraName + "_rejectReason", "odometry-reset");
    }
  }

  /**
   * Returns interpolated standard deviations for the given measurement state using the per-camera
   * VisionStdDevMap. Hard gates (ambiguity, distance) are applied before lookup. MT2 always returns
   * MAX_VALUE for theta — heading comes from the gyro, not vision. MT1 returns MAX_VALUE for theta
   * on single-tag observations.
   */
  private Matrix<N3, N1> getEstimationStdDevs(
      double avgTagDist,
      int numOfTags,
      double maxAmbiguity,
      boolean isMegaTag2,
      VisionStdDevMap stdDevMap,
      String cameraName) {

    double maxDistGate =
        isMegaTag2 ? VisionConstants.MAX_DISTANCE_MT2 : VisionConstants.MAX_DISTANCE_MT1;

    if (maxAmbiguity >= VisionConstants.MAX_AMBIGUITY) {
      return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    }
    if (numOfTags == 1 && avgTagDist > maxDistGate) {
      return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    }

    VisionStdDevs devs = stdDevMap.getInterpolated(avgTagDist, maxAmbiguity);

    if (isMegaTag2) {
      SmartDashboard.putNumber("/vision/" + cameraName + " Mt2 STD xy", devs.mt2X);
      // Heading from gyro on MT2 — never trust vision heading
      return VecBuilder.fill(devs.mt2X, devs.mt2Y, Double.MAX_VALUE);
    } else {
      double theta = (numOfTags == 1) ? Double.MAX_VALUE : devs.mt1Theta;
      SmartDashboard.putNumber("/vision/" + cameraName + " Mt1 STD xy", devs.mt1X);
      SmartDashboard.putNumber("/vision/" + cameraName + " Mt1 STD theta", theta);
      return VecBuilder.fill(devs.mt1X, devs.mt1Y, theta);
    }
  }

  /**
   * Returns the maximum ambiguity across all visible fiducials. Gating on max (not average) is
   * correct: one bad tag in a multi-tag solve poisons the whole observation regardless of how good
   * the other tags are.
   */
  public double getMaxAmbiguity(RawFiducial[] rfs) {
    if (rfs == null || rfs.length == 0) return 0;
    double max = 0;
    for (RawFiducial rf : rfs) max = Math.max(max, rf.ambiguity);
    return max;
  }

  private void publishDiagnostics(
      BetterPoseEstimate estimate, Pose2d visionPose2d, LLCamera camera, String rejectionReason) {
    if (estimate.timestampSeconds >= lastTimestampSeconds) {
      SmartDashboard.putString("/vision/" + camera.getName() + "_rejectReason", rejectionReason);
      SmartDashboard.putNumber(
          "/vision/" + camera.getName() + "_visionError",
          getVisionPoseError(visionPose2d, estimate.timestampSeconds));
      SmartDashboard.putNumber(
          "/vision/" + camera.getName() + "_Last timestamp", camera.getLastTimestampSeconds());
      SmartDashboard.putNumber(
          "/vision/" + camera.getName() + "_Num targets", camera.getNumTargets());
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

  public boolean getDisableVision() {
    return disableVision.get(false);
  }

  public Pose2d getLastVisionPose2d() {
    return lastFieldPose;
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

  private double getVisionPoseError(Pose2d visionPose2d, double timestampSeconds) {
    if (drivetrain == null) return 0;
    var historicPose = drivetrain.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));
    return historicPose
        .map(pose2d -> getDistanceToTargetViaPoseEstimation(pose2d, visionPose2d))
        .orElse(0.0);
  }

  public void setShowVisionOnField(ShowVisionOnField m_showVisionOnField) {
    this.m_showVisionOnField = m_showVisionOnField;
  }
}
