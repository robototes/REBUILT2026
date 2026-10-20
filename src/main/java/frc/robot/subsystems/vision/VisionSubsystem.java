package frc.robot.subsystems.vision;

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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;
import frc.robot.sim.ShowVisionOnField;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.subsystems.vision.VisionIO.TagObservation;
import frc.robot.util.AllianceUtils;
import frc.robot.util.robotType.RobotType;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class VisionSubsystem extends SubsystemBase {

  private static final String[] CAMERA_NAMES = {
    Hardware.LIMELIGHT_A, Hardware.LIMELIGHT_B, Hardware.LIMELIGHT_C
  };
  private static final int CAM_A = 0;
  private static final int CAM_B = 1;
  private static final int CAM_C = 2;

  public boolean limelightaOnline = false;
  public boolean limelightbOnline = false;
  public boolean limelightcOnline = false;

  private static LoggedNetworkNumber A_XY_MT2 =
      new LoggedNetworkNumber("Tuning/vision/A_XY_MT2", 0.07);
  private static LoggedNetworkNumber A_XY_MT1 =
      new LoggedNetworkNumber("Tuning/vision/A_XY_MT1", 0.09);
  private static LoggedNetworkNumber P_XY = new LoggedNetworkNumber("Tuning/vision/P_XY", 1.4);

  // How much to reduce std devs when defense slip is detected.
  // <1.0 = trust vision more (0.5 = half the std dev = 4x the filter weight).
  // Tune this at practice with someone actively defending.
  private static LoggedNetworkNumber DEFENSE_STD_DEV_SCALE =
      new LoggedNetworkNumber("Tuning/vision/defenseStdDevScale", 0.5);

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
          new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(90)));
  public static final Transform3d COMP_BOT_FRONT_CAMERA =
      new Transform3d(0.295, -0.288, 0.273, new Rotation3d(0, Units.degreesToRadians(-20), 0));
  public static final Transform3d COMP_BOT_RIGHT_CAMERA =
      new Transform3d(
          0.212,
          -0.371,
          0.273,
          new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(-90)));
  private final Field2d robotField;
  private final FieldObject2d rawVisionFieldObject;
  private final LoggedNetworkBoolean disableVision =
      new LoggedNetworkBoolean("disableVision", false);

  private static final String RAW_POSE_KEY_A = "vision/rawFieldPose3dA";
  private static final String RAW_POSE_KEY_B = "vision/rawFieldPose3dB";
  private static final String RAW_POSE_KEY_C = "vision/rawFieldPose3dC";
  private static final String[] RAW_POSE_KEYS = {RAW_POSE_KEY_A, RAW_POSE_KEY_B, RAW_POSE_KEY_C};

  /**
   * Per-camera processing state. This used to live inside LLCamera. It is derived from accepted
   * measurements (not from hardware).
   */
  private static class CameraState {
    double lastTimestampSeconds = 0;
    Pose2d lastPoseMT1 = null;
    Pose2d lastPoseMT2 = null;
  }

  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final CameraState[] cameraStates;

  private double lastTimestampSeconds = 0;
  private Pose2d lastFieldPose = null;
  private CommandSwerveDrivetrain drivetrain;

  private record VisionPoseTracking(
      SwerveDriveState swerveState, ChassisSpeeds swerveSpeeds, Pose3d drivePose3d) {}

  private VisionPoseTracking visionPoseTracking;
  private ShowVisionOnField m_showVisionOnField;

  public VisionSubsystem(CommandSwerveDrivetrain drivetrain, VisionIO[] io) {
    this.drivetrain = drivetrain;
    this.io = io;
    inputs = new VisionIOInputsAutoLogged[io.length];
    cameraStates = new CameraState[io.length];
    for (int i = 0; i < io.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
      cameraStates[i] = new CameraState();
    }

    // TODO: Log this with Telemetry WPILib 2027 no support or workaround from advantage kit
    robotField = new Field2d();
    SmartDashboard.putData(robotField);
    rawVisionFieldObject = robotField.getObject("RawVision");

    Logger.recordOutput("vision/limelight-a_Last timestamp", 0.0);
    Logger.recordOutput("vision/limelight-b_Last timestamp", 0.0);
    Logger.recordOutput("vision/limelight-c_Last timestamp", 0.0);
    Logger.recordOutput("vision/limelight-a_Num targets", 0);
    Logger.recordOutput("vision/limelight-b_Num targets", 0);
    Logger.recordOutput("vision/limelight-c_Num targets", 0);
    Logger.recordOutput("vision/limelight-a_time since last reading", 0.0);
    Logger.recordOutput("vision/limelight-b_time since last reading", 0.0);
    Logger.recordOutput("vision/limelight-c_time since last reading", 0.0);
  }

  /** Must be called once per robot loop. */
  public void update() {
    // 1. Read + log inputs (or load them from the log in replay). This always runs, even when
    //    vision is disabled, so the log stays complete.
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("vision/" + CAMERA_NAMES[i], inputs[i]);
    }

    limelightaOnline = inputs[CAM_A].connected;
    limelightbOnline = inputs[CAM_B].connected;
    limelightcOnline = inputs[CAM_C].connected;

    // 2. Logic + outputs
    if (getDisableVision()) {
      for (String name : CAMERA_NAMES) {
        Logger.recordOutput("/vision/" + name + "_rejectReason", "vision-disabled");
      }
      return;
    }

    SwerveDriveState swerveDriveState = drivetrain.getState();
    visionPoseTracking =
        new VisionPoseTracking(
            swerveDriveState, swerveDriveState.Speeds, new Pose3d(swerveDriveState.Pose));

    // Compute defense state once per update cycle so all cameras see the same value.
    // isUnderDefense() uses lastFieldPose and visionPoseTracking, both of which are
    // set before any camera processes — this is intentional.
    boolean underDefense = isUnderDefense(visionPoseTracking);
    Logger.recordOutput("vision/underDefense", underDefense);

    processCamera(CAM_A, visionPoseTracking, underDefense);
    processCamera(CAM_B, visionPoseTracking, underDefense);
    processCamera(CAM_C, visionPoseTracking, underDefense);
    updateCameraView(visionPoseTracking);
  }

  private void processCamera(
      int cameraIndex, VisionPoseTracking visionPoseTracking, boolean underDefense) {
    VisionIOInputsAutoLogged cameraInputs = inputs[cameraIndex];
    if (!cameraInputs.connected) return;

    TagObservation[] fiducials = cameraInputs.fiducials;
    double maxAmbiguity = getMaxAmbiguity(fiducials);

    // Observations are processed MT1 then MT2.
    for (PoseObservation observation : cameraInputs.poseObservations) {
      processLimelight(
          observation, cameraIndex, maxAmbiguity, visionPoseTracking, fiducials, underDefense);
    }
  }

  private void processLimelight(
      PoseObservation observation,
      int cameraIndex,
      double maxAmbiguity,
      VisionPoseTracking visionPoseTracking,
      TagObservation[] fiducials,
      boolean underDefense) {

    if (observation == null || observation.tagCount() <= 0) return;

    String cameraName = CAMERA_NAMES[cameraIndex];
    CameraState state = cameraStates[cameraIndex];
    boolean isMegaTag2 = observation.type() == PoseObservationType.MEGATAG_2;
    Pose3d pose3d = observation.pose();
    double timestamp = observation.timestamp();

    Logger.recordOutput(RAW_POSE_KEYS[cameraIndex] + (isMegaTag2 ? "_MT2" : "_MT1"), pose3d);

    Pose2d visionPose2d = pose3d.toPose2d();
    if (timestamp == state.lastTimestampSeconds) {
      publishDiagnostics(observation, visionPose2d, cameraIndex, "stale-timestamp");
      return;
    }

    if (RobotType.isAlpha()
        && (Math.abs(visionPoseTracking.swerveSpeeds.vxMetersPerSecond)
                > VisionConstants.MAX_XY_VELO_ALPHA
            || Math.abs(visionPoseTracking.swerveSpeeds.vyMetersPerSecond)
                > VisionConstants.MAX_XY_VELO_ALPHA
            || Math.abs(visionPoseTracking.swerveSpeeds.omegaRadiansPerSecond)
                > VisionConstants.MAX_TURN_VELO_ALPHA)) {
      publishDiagnostics(observation, visionPose2d, cameraIndex, "alpha-max-speed");
      return;
    }

    if (!MathUtil.isNear(
            0,
            pose3d.getRotation().getX(),
            Units.degreesToRadians(VisionConstants.ROTATION_TOLERANCE))
        || !MathUtil.isNear(
            0,
            pose3d.getRotation().getY(),
            Units.degreesToRadians(VisionConstants.ROTATION_TOLERANCE))
        || (lastFieldPose != null && lastFieldPose.equals(visionPose2d))) {
      publishDiagnostics(observation, visionPose2d, cameraIndex, "impossible-rotation or height");
      return;
    }

    if (!isPoseOnField(visionPose2d)) {
      publishDiagnostics(observation, visionPose2d, cameraIndex, "off-field");
      return;
    }

    Pose2d lastVisionPose = isMegaTag2 ? state.lastPoseMT2 : state.lastPoseMT1;
    if (!isVelocityPlausible(
        visionPose2d, timestamp, lastVisionPose, state.lastTimestampSeconds, underDefense)) {
      publishDiagnostics(observation, visionPose2d, cameraIndex, "velocity-implausible");
      return;
    }

    double spread = getMultiTagSpread(fiducials, pose3d, AllianceUtils.FIELD_LAYOUT);
    if (spread > VisionConstants.SPREAD_REJECT) {
      Logger.recordOutput("vision/" + cameraName + "_tagSpread", spread);
      publishDiagnostics(observation, visionPose2d, cameraIndex, "inter-tag-inconsistent");
      return;
    }

    double avgTagDist = observation.averageTagDistance();
    Matrix<N3, N1> stdDevs;
    if (isMegaTag2) {
      stdDevs =
          getEstimationStdDevsLimelightMT2(
              avgTagDist, observation.tagCount(), maxAmbiguity, fiducials, cameraName);
    } else {
      stdDevs =
          getEstimationStdDevsLimelightMT1(
              avgTagDist, observation.tagCount(), maxAmbiguity, fiducials, cameraName);
    }

    if (stdDevs.get(0, 0) >= Double.MAX_VALUE) {
      publishDiagnostics(observation, visionPose2d, cameraIndex, "ambiguity-or-range");
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
          cameraName,
          isMegaTag2,
          java.util.Optional.of(pose3d.toPose2d()));
    }

    maybeResetToVision(visionPose2d, maxAmbiguity, observation.tagCount(), cameraName);

    drivetrain.addVisionMeasurement(visionPose2d, Utils.fpgaToCurrentTime(timestamp), stdDevs);
    robotField.setRobotPose(drivetrain.getState().Pose);

    if (isMegaTag2) {
      state.lastPoseMT2 = visionPose2d;
    } else {
      state.lastPoseMT1 = visionPose2d;
    }
    state.lastTimestampSeconds = timestamp;

    if (timestamp >= lastTimestampSeconds) {
      Logger.recordOutput("vision/fieldPose3d", pose3d);
      lastFieldPose = visionPose2d;
      rawVisionFieldObject.setPose(lastFieldPose);
      lastTimestampSeconds = timestamp;
    }

    Logger.recordOutput("vision/" + cameraName + "_tagSpread", spread);
    publishDiagnostics(observation, visionPose2d, cameraIndex, "none");
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
      double timeSinceVision = now() - lastTimestampSeconds;
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
   * Replay-safe replacement for Timer.getFPGATimestamp(). Logger.getTimestamp() returns
   * microseconds and follows the logged clock during replay.
   */
  private static double now() {
    return Logger.getTimestamp() / 1.0e6;
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
   * (Z=0) to tag (Z=tagHeight) rf.distToRobot() = 3D distance reported by Limelight from robot
   * origin to tag
   *
   * <p>This makes both sides consistent. No camera height constant needed because distToRobot is
   * already robot-origin referenced (Limelight applies the robot-to-camera transform internally).
   *
   * <p>Returns 0.0 for single-tag observations (nothing to compare against). Returns
   * SPREAD_REJECT+1 if any tag ID is unknown (immediate hard reject).
   */
  private double getMultiTagSpread(
      TagObservation[] fiducials, Pose3d reportedPose, AprilTagFieldLayout aprilTagFieldLayout) {
    if (aprilTagFieldLayout == null || fiducials == null || fiducials.length < 2) return 0.0;

    double sumSqErr = 0;
    int count = 0;
    for (TagObservation rf : fiducials) {
      var tagPoseOpt = aprilTagFieldLayout.getTagPose(rf.id());
      if (tagPoseOpt.isEmpty()) {
        // Unknown tag ID — could be a misread. Reject the whole observation.
        return VisionConstants.SPREAD_REJECT + 1.0;
      }
      double distancePerPose =
          reportedPose
              .getTranslation()
              .getDistance(tagPoseOpt.get().getTranslation()); // what the pose says
      double err = distancePerPose - rf.distToRobot();
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
      Logger.recordOutput("vision/" + cameraName + "_rejectReason", "odometry-reset");
    }
  }

  // Std dev computation — MT1
  private Matrix<N3, N1> getEstimationStdDevsLimelightMT1(
      double avgTagDist,
      int numOfTags,
      double maxAmbiguity,
      TagObservation[] fiducials,
      String cameraName) {

    if (maxAmbiguity >= VisionConstants.MAX_AMBIGUITY) {
      return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    }
    if (numOfTags == 1 && avgTagDist > VisionConstants.MAX_DISTANCE_MT1) {
      return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    }

    double ambiguityInflation = 1.0 / Math.pow(1.0 - maxAmbiguity, 2.0);
    double harmonicSum = computeHarmonicSum(fiducials);
    if (harmonicSum <= 0) harmonicSum = 1.0 / (avgTagDist * avgTagDist + 1e-6);

    double xy =
        A_XY_MT1.get()
            * Math.pow(avgTagDist, P_XY.get())
            / Math.sqrt(harmonicSum)
            * ambiguityInflation;
    double theta =
        (numOfTags == 1)
            ? Double.MAX_VALUE
            : VisionConstants.STD_DEVS_MT1_THETA * ambiguityInflation;

    Logger.recordOutput("vision/" + cameraName + " Mt1 STD xy", xy);
    Logger.recordOutput("vision/" + cameraName + " Mt1 STD theta", theta);
    return VecBuilder.fill(xy, xy, theta);
  }

  // Std dev computation — MT2
  private Matrix<N3, N1> getEstimationStdDevsLimelightMT2(
      double avgTagDist,
      int numOfTags,
      double maxAmbiguity,
      TagObservation[] fiducials,
      String cameraName) {

    if (maxAmbiguity >= VisionConstants.MAX_AMBIGUITY) {
      return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    }
    if (numOfTags == 1 && avgTagDist > VisionConstants.MAX_DISTANCE_MT2) {
      return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    }

    double ambiguityInflation = 1.0 / Math.pow(1.0 - maxAmbiguity, 2.0);
    double harmonicSum = computeHarmonicSum(fiducials);
    if (harmonicSum <= 0) harmonicSum = 1.0 / (avgTagDist * avgTagDist + 1e-6);

    double xy =
        A_XY_MT2.get()
            * Math.pow(avgTagDist, P_XY.get())
            / Math.sqrt(harmonicSum)
            * ambiguityInflation;

    Logger.recordOutput("vision/" + cameraName + " Mt2 STD xy", xy);
    // θ = MAX_VALUE: heading comes from gyro, not vision
    return VecBuilder.fill(xy, xy, Double.MAX_VALUE);
  }

  private double computeHarmonicSum(TagObservation[] fiducials) {
    if (fiducials == null || fiducials.length == 0) return 0.0;
    double sum = 0.0;
    for (TagObservation rf : fiducials) {
      double dist = rf.distToRobot() > 0.01 ? rf.distToRobot() : rf.distToCamera();
      if (dist > 0.01) sum += 1.0 / (dist * dist);
    }
    return sum;
  }

  /**
   * Returns the maximum ambiguity across all visible fiducials. Gating on max (not average) is
   * correct: one bad tag in a multi-tag solve poisons the whole observation regardless of how good
   * the other tags are.
   */
  public double getMaxAmbiguity(TagObservation[] fiducials) {
    if (fiducials == null || fiducials.length == 0) return 0;
    double max = 0;
    for (TagObservation rf : fiducials) max = Math.max(max, rf.ambiguity());
    return max;
  }

  private void publishDiagnostics(
      PoseObservation observation, Pose2d visionPose2d, int cameraIndex, String rejectionReason) {
    if (observation.timestamp() >= lastTimestampSeconds) {
      String cameraName = CAMERA_NAMES[cameraIndex];
      Logger.recordOutput("vision/" + cameraName + "_rejectReason", rejectionReason);
      Logger.recordOutput(
          "vision/" + cameraName + "_visionError",
          getVisionPoseError(visionPose2d, observation.timestamp()));
      Logger.recordOutput(
          "vision/" + cameraName + "_Last timestamp",
          cameraStates[cameraIndex].lastTimestampSeconds);
      Logger.recordOutput("vision/" + cameraName + "_Num targets", inputs[cameraIndex].numTargets);
    }
  }

  public int getNumTargets() {
    if (!RobotType.isAlpha()) {
      return inputs[CAM_A].numTargets + inputs[CAM_B].numTargets + inputs[CAM_C].numTargets;
    } else {
      return inputs[CAM_C].numTargets;
    }
  }

  public double getLastTimestampSeconds() {
    return lastTimestampSeconds;
  }

  public boolean getDisableVision() {
    return disableVision.get();
  }

  public Pose2d getLastVisionPose2d() {
    return lastFieldPose;
  }

  public double getTimeSinceLastReading() {
    return now() - lastTimestampSeconds;
  }

  public double getDistanceToTargetViaPoseEstimation(Pose2d yourPose, Pose2d targetPose) {
    if (yourPose == null || targetPose == null) return 0;
    double distance =
        Math.hypot(targetPose.getX() - yourPose.getX(), targetPose.getY() - yourPose.getY());
    return (double) Math.round(distance * 1000) / 1000;
  }

  private void updateCameraView(VisionPoseTracking visionPoseTracking) {
    if (visionPoseTracking != null && visionPoseTracking.drivePose3d != null) {
      Logger.recordOutput(
          "vision/compBotLeftCameraView",
          visionPoseTracking.drivePose3d.transformBy(COMP_BOT_LEFT_CAMERA));
      Logger.recordOutput(
          "vision/compBotFrontCameraView",
          visionPoseTracking.drivePose3d.transformBy(COMP_BOT_FRONT_CAMERA));
      Logger.recordOutput(
          "vision/compBotRightCameraView",
          visionPoseTracking.drivePose3d.transformBy(COMP_BOT_RIGHT_CAMERA));
    }
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
