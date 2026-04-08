package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import frc.robot.util.AllianceUtils;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import frc.robot.util.LimelightHelpers.RawFiducial;
import frc.robot.util.tuning.NtTunableBoolean;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

public class VisionSubsystemV2 extends SubsystemBase {
  private final CommandSwerveDrivetrain driveBase;

  // HB cache
  private final Map<String, Double> lastBeats = new HashMap<>();

  // Tracking for Velocity/Duplicate gates
  private final Map<String, Pose2d> lastVisionPoses = new HashMap<>();
  private final Map<String, Double> lastVisionTimestamps = new HashMap<>();

  // -- CONFIGURATION & PUB MAPS -- //
  private final Map<String, NtTunableBoolean> LL_enabled = new HashMap<>();
  private final Map<String, BooleanPublisher> LL_status_pubs = new HashMap<>();
  private final Map<String, StructPublisher<Pose2d>> LL_pose_pubs = new HashMap<>();
  private final Map<String, Boolean> LL_online = new HashMap<>();
  private final Map<String, NtTunableBoolean> FILTER_ENABLED = new HashMap<>();
  private final Map<String, Debouncer> heartbeatDebouncers = new HashMap<>();

  private final String[] names = {Hardware.LIMELIGHT_A, Hardware.LIMELIGHT_B, Hardware.LIMELIGHT_C};

  // Hardware objects & Signals
  private final Pigeon2 gyro;
  private final StatusSignal<Angle> gyro_yaw, gyro_pitch, gyro_roll;
  private final StatusSignal<AngularVelocity> gyro_vx, gyro_vy, gyro_vz;

  // Constants
  private static final double MAX_ANGULAR_VELOCITY = 720;
  private static final double MAX_DISTANCE_METERS = 8.5;
  private static final double MAX_TILT = 10;
  private static final double STALENESS_THRESHOLD = 2.0; // seconds
  private static final double AMBIGUITY_THRESHOLD = 0.4;
  private static final double ROTATION_STD_DEV_REJECT = 999999;
  private static final double RMSE_REJECT_THRESHOLD = 0.5;
  private static final double STD_DEV_SCALAR = 0.035;
  private static final double DISTANCE_POWER_FACTOR = 1.4;
  private static final double IMU_ASSIST_ALPHA = 0.1;
  private static final double RESET_MAX_AMBIGUITY_THRESHOLD = 0.15;
  private static final int RESET_MIN_TAGS = 2;
  private static final double MIN_RMSE = 0.01;
  private static final double MIN_STD_DEV = 0.01;
  private static final double MIN_DIST_DELTA = 0.01;

  // Sanity Gate Constants
  private static final double FIELD_MARGIN = 0.5;
  private static final double MAX_VISION_IMPLIED_SPEED = 7.0;
  private static final double SPREAD_INFLATE_START = 0.10;

  // Limelight settings
  private static final int APRIL_TAG_COMP_PIPELINE = 0;
  private static final int ENABLED_IMU_MODE = 4;
  private static final int DISABLED_IMU_MODE = 1;
  private static final int THROTTLE_VALUE_ON = 150;
  private static final int THROTTLE_VALUE_OFF = 0;

  private final AprilTagFieldLayout field = AllianceUtils.FIELD_LAYOUT;
  private final double FIELD_LENGTH = field.getFieldLength();
  private final double FIELD_WIDTH = field.getFieldWidth();

  public VisionSubsystemV2(CommandSwerveDrivetrain drivetrain) {
    this.driveBase = drivetrain;
    this.gyro = driveBase.getPigeon2();
    var inst = NetworkTableInstance.getDefault();

    for (String name : names) {
      // All cameras default to enabled for competition
      LL_enabled.put(name, new NtTunableBoolean("SmartDashboard/Vision/Enabled_" + name, true));

      // Status (Online/Offline)
      LL_status_pubs.put(
          name, inst.getBooleanTopic("SmartDashboard/Vision/Status_" + name).publish());

      // Pose (Where the camera thinks the robot is)
      LL_pose_pubs.put(
          name, inst.getStructTopic("SmartDashboard/Vision/Pose_" + name, Pose2d.struct).publish());

      LL_online.put(name, false);
      LimelightHelpers.SetIMUAssistAlpha(name, IMU_ASSIST_ALPHA);
      lastBeats.put(name, LimelightHelpers.getHeartbeat(name));
      heartbeatDebouncers.put(
          name, new Debouncer(STALENESS_THRESHOLD, Debouncer.DebounceType.kFalling));
    }

    FILTER_ENABLED.put(
        "RMSE Filter", new NtTunableBoolean("SmartDashboard/Vision/Filters/RMSE enabled", true));
    FILTER_ENABLED.put(
        "Harmonic sum Filter",
        new NtTunableBoolean("SmartDashboard/Vision/Filters/Harmonic enabled", true));

    gyro_yaw = gyro.getYaw();
    gyro_pitch = gyro.getPitch();
    gyro_roll = gyro.getRoll();
    gyro_vx = gyro.getAngularVelocityXWorld();
    gyro_vy = gyro.getAngularVelocityYWorld();
    gyro_vz = gyro.getAngularVelocityZWorld();
  }

  public void update() {
    updateLimeLightStatus();
    BaseStatusSignal.refreshAll(gyro_yaw, gyro_pitch, gyro_roll, gyro_vx, gyro_vy, gyro_vz);

    // All angular velocities and angles are already defaulted to deg/s
    double yaw = gyro_yaw.getValueAsDouble();
    double pitch = gyro_pitch.getValueAsDouble();
    double roll = gyro_roll.getValueAsDouble();
    double vx = gyro_vx.getValueAsDouble();
    double vy = gyro_vy.getValueAsDouble();
    double vz = gyro_vz.getValueAsDouble();

    for (String name : names) {
      if (isEnabled(name) && isOnline(name)) {
        LimelightHelpers.SetRobotOrientation_NoFlush(name, yaw, vz, pitch, vy, roll, vx);
        PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        processVision(name, estimate, vz, roll, pitch);
      }
    }
    LimelightHelpers.Flush();
  }

  private void processVision(
      String name, PoseEstimate estimate, double angVel, double roll, double pitch) {
    if (estimate == null || estimate.tagCount == 0) return;

    // --- NT PUBLISHING ---
    // Publish the raw vision pose before any gates so we can see what the LL is seeing
    if (LL_pose_pubs.containsKey(name)) {
      LL_pose_pubs.get(name).set(estimate.pose);
    }

    // --- SANITY GATES ---
    if (Math.abs(angVel) > MAX_ANGULAR_VELOCITY) return;
    if (Math.abs(roll) > MAX_TILT || Math.abs(pitch) > MAX_TILT) return;
    if (estimate.avgTagDist >= MAX_DISTANCE_METERS) return;
    if (!isPoseOnField(estimate.pose)) return;

    // Duplicate Pose Gate
    Pose2d lastPose = lastVisionPoses.get(name);
    if (lastPose != null
        && lastPose.getTranslation().getDistance(estimate.pose.getTranslation()) < MIN_DIST_DELTA)
      return;

    // Velocity Gate
    double lastTs = lastVisionTimestamps.getOrDefault(name, 0.0);
    double dt = estimate.timestampSeconds - lastTs;
    if (lastPose != null && dt > 0 && dt <= 1.0) {
      double impliedSpeed =
          estimate.pose.getTranslation().getDistance(lastPose.getTranslation()) / dt;
      if (impliedSpeed > MAX_VISION_IMPLIED_SPEED) return;
    }

    // --- STANDARD DEVIATION CALCULATIONS ---
    double stdDevXY = STD_DEV_SCALAR * Math.pow(estimate.avgTagDist, DISTANCE_POWER_FACTOR);
    double RMSE = 1;
    double actualRMSE = 0;
    double harmonicSum = 1;

    if (FILTER_ENABLED.get("RMSE Filter").get()) {
      actualRMSE = RMSE(estimate);
      RMSE = Math.max(actualRMSE, MIN_RMSE);
      if (RMSE > RMSE_REJECT_THRESHOLD) return;
    }

    if (FILTER_ENABLED.get("Harmonic sum Filter").get()) {
      harmonicSum = harmonicSum(estimate.rawFiducials);
      if (harmonicSum == 0) return;
    }

    stdDevXY = Math.max(stdDevXY / Math.sqrt(harmonicSum) * Math.max(RMSE, MIN_RMSE), MIN_STD_DEV);

    // Spread Inflation
    if (FILTER_ENABLED.get("RMSE Filter").get() && actualRMSE > SPREAD_INFLATE_START) {
      stdDevXY *= Math.pow(actualRMSE / SPREAD_INFLATE_START, 2.0);
    }

    robotPoseOutOfBoundsReset(estimate, estimate.tagCount);

    // Apply to Drivetrain
    driveBase.addVisionMeasurement(
        estimate.pose,
        estimate.timestampSeconds,
        VecBuilder.fill(stdDevXY, stdDevXY, ROTATION_STD_DEV_REJECT));

    // Cache state
    lastVisionPoses.put(name, estimate.pose);
    lastVisionTimestamps.put(name, estimate.timestampSeconds);
  }

  // --- HELPERS ---

  private double harmonicSum(RawFiducial[] tags) {
    if (tags == null) return 0;
    return Arrays.stream(tags)
        .filter(tag -> tag.distToCamera > 0 && tag.ambiguity <= AMBIGUITY_THRESHOLD)
        .mapToDouble(tag -> 1 / Math.pow(tag.distToCamera, 2))
        .sum();
  }

  private double RMSE(PoseEstimate estimate) {
    RawFiducial[] tags = estimate.rawFiducials;
    if (tags == null || tags.length == 0) return 0;
    double error = 0;
    for (RawFiducial tag : tags) {
      var tagPoseReal = field.getTagPose(tag.id);
      if (tagPoseReal.isEmpty()) return Double.MAX_VALUE;
      double actualDist3d =
          tagPoseReal
              .get()
              .getTranslation()
              .getDistance(new Translation3d(estimate.pose.getX(), estimate.pose.getY(), 0.0));
      error += Math.pow(actualDist3d - tag.distToRobot, 2);
    }
    return Math.sqrt(error / tags.length);
  }

  private void robotPoseOutOfBoundsReset(PoseEstimate estimate, int tagCount) {
    Pose2d odomPose = driveBase.getState().Pose;
    if (!isPoseOnField(odomPose)) {
      double avgAmbiguity =
          Arrays.stream(estimate.rawFiducials).mapToDouble(t -> t.ambiguity).average().orElse(1.0);
      if (avgAmbiguity < RESET_MAX_AMBIGUITY_THRESHOLD
          && tagCount >= RESET_MIN_TAGS
          && isPoseOnField(estimate.pose)) {
        driveBase.resetPose(estimate.pose);
      }
    }
  }

  private boolean isPoseOnField(Pose2d pose) {
    double x = pose.getX();
    double y = pose.getY();
    return x >= -FIELD_MARGIN
        && x <= FIELD_LENGTH + FIELD_MARGIN
        && y >= -FIELD_MARGIN
        && y <= FIELD_WIDTH + FIELD_MARGIN;
  }

  private void updateLimeLightStatus() {
    for (String name : names) {
      if (!isEnabled(name)) {
        changeStatus(name, false);
        continue;
      }
      double currentBeat = LimelightHelpers.getHeartbeat(name);
      boolean beating = currentBeat != lastBeats.get(name);
      lastBeats.put(name, currentBeat);
      boolean online = heartbeatDebouncers.get(name).calculate(beating);
      changeStatus(name, online);
    }
  }

  private void changeStatus(String name, boolean status) {
    LL_online.put(name, status);
    if (LL_status_pubs.containsKey(name)) LL_status_pubs.get(name).set(status);
  }

  public boolean isEnabled(String name) {
    NtTunableBoolean tunable = LL_enabled.get(name);
    return tunable != null && tunable.get();
  }

  public boolean isOnline(String name) {
    return LL_online.getOrDefault(name, false);
  }

  public void limelightRobotDisabled() {
    for (String name : names)
      if (isOnline(name) && isEnabled(name)) setupLimelightForAprilTags(name, true);
  }

  public void limelightDisabledExit() {
    for (String name : names)
      if (isOnline(name) && isEnabled(name)) setupLimelightForAprilTags(name, false);
  }

  private void setupLimelightForAprilTags(String limelightName, boolean isEnteringDisabled) {
    if (isEnteringDisabled) {
      LimelightHelpers.SetThrottle(limelightName, THROTTLE_VALUE_ON);
      LimelightHelpers.SetIMUMode(limelightName, DISABLED_IMU_MODE);
    } else {
      LimelightHelpers.SetThrottle(limelightName, THROTTLE_VALUE_OFF);
      LimelightHelpers.SetIMUMode(limelightName, ENABLED_IMU_MODE);
    }
    LimelightHelpers.setPipelineIndex(limelightName, APRIL_TAG_COMP_PIPELINE);
  }
}
