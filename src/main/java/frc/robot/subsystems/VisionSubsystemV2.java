package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import frc.robot.util.AllianceUtils;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import frc.robot.util.LimelightHelpers.RawFiducial;
import frc.robot.util.robotType.RobotType;
import frc.robot.util.tuning.NtTunableBoolean;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

public class VisionSubsystemV2 extends SubsystemBase {
  private final CommandSwerveDrivetrain driveBase;

  // HB cache
  private final Map<String, Double> lastBeats = new HashMap<>();

  // Pose & Timestamp tracking for Velocity/Duplicate gates
  private final Map<String, Pose2d> lastVisionPoses = new HashMap<>();
  private final Map<String, Double> lastVisionTimestamps = new HashMap<>();

  // -- CONFIGURATION MAPS -- //
  private final Map<String, NtTunableBoolean> LL_enabled = new HashMap<>();
  private final Map<String, BooleanPublisher> LL_status_pubs = new HashMap<>();
  private final Map<String, Boolean> LL_online = new HashMap<>();
  private final Map<String, NtTunableBoolean> FILTER_ENABLED = new HashMap<>();
  private final HashMap<String, Transform3d> transforms = new HashMap<>();
  private final String[] names = {Hardware.LIMELIGHT_A, Hardware.LIMELIGHT_B, Hardware.LIMELIGHT_C};

  // Hardware objects
  private final Pigeon2 gyro;

  // status signals
  private final StatusSignal<Angle> gyro_yaw, gyro_pitch, gyro_roll;
  private final StatusSignal<AngularVelocity> gyro_vx, gyro_vy, gyro_vz;

  // Constants
  private static final double MAX_ANGULAR_VELOCITY = 720;
  private static final double MAX_DISTANCE_METERS = 8.5;
  private static final double MAX_TILT = 10;

  private static final double STALENESS_THRESHOLD = 1.0; // Seconds
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

  // New Constants from alternative vision system
  private static final double FIELD_MARGIN = 0.5; // Meters
  private static final double MAX_VISION_IMPLIED_SPEED = 7.0; // m/s
  private static final double SPREAD_INFLATE_START = 0.10; // meters

  // Limelight settings
  private static final int APRIL_TAG_COMP_PIPELINE = 0;
  private static final int ENABLED_IMU_MODE = 4;
  private static final int DISABLED_IMU_MODE = 1;
  private static final int THROTTLE_VALUE_ON = 150;
  private static final int THROTTLE_VALUE_OFF = 0;

  // Field dimensions
  private final AprilTagFieldLayout field = AllianceUtils.FIELD_LAYOUT;
  private final double FIELD_LENGTH = field.getFieldLength();
  private final double FIELD_WIDTH = field.getFieldWidth();

  public VisionSubsystemV2(CommandSwerveDrivetrain drivetrain) {
    this.driveBase = drivetrain;
    this.gyro = driveBase.getPigeon2();

    var inst = NetworkTableInstance.getDefault();

    // Setup limelight statuses
    for (String name : names) {
      // Determine default based on bot type
      boolean defaultEnabled;
      if (RobotType.isAlpha()) {
        defaultEnabled = name.equals(Hardware.LIMELIGHT_C);
      } else {
        defaultEnabled = name.equals(Hardware.LIMELIGHT_A) || name.equals(Hardware.LIMELIGHT_B);
      }

      // Create the tunable first, THEN put it in the map
      LL_enabled.put(
          name, new NtTunableBoolean("SmartDashboard/Vision/Enabled_" + name, defaultEnabled));
      LL_status_pubs.put(
          name, inst.getBooleanTopic("SmartDashboard/Vision/Status_" + name).publish());
      LL_online.put(name, false);
      LimelightHelpers.SetIMUAssistAlpha(name, IMU_ASSIST_ALPHA);

      lastBeats.put(name, LimelightHelpers.getHeartbeat(name));
    }
    // Setup filter enabled map
    FILTER_ENABLED.put(
        "RMSE Filter", new NtTunableBoolean("SmartDashboard/Vision/Filters/RMSE enabled", true));
    FILTER_ENABLED.put(
        "Harmonic sum Filter",
        new NtTunableBoolean("SmartDashboard/Vision/Filters/Harmonic enabled", true));

    // Setup Gyro Signals
    gyro_yaw = gyro.getYaw();
    gyro_pitch = gyro.getPitch();
    gyro_roll = gyro.getRoll();
    gyro_vx = gyro.getAngularVelocityXWorld();
    gyro_vy = gyro.getAngularVelocityYWorld();
    gyro_vz = gyro.getAngularVelocityZWorld();

    // Add transforms
    Transform3d COMP_BOT_FRONT_CAMERA =
        new Transform3d(0.267, -0.051, 0.451, new Rotation3d(0, Units.degreesToRadians(15), 0));
    Transform3d COMP_BOT_LEFT_CAMERA =
        new Transform3d(
            -0.076,
            0.311,
            0.284,
            new Rotation3d(0, Units.degreesToRadians(8), Units.degreesToRadians(90)));

    // TODO: Update these numbers with the actual physical location of Camera C!
    Transform3d COMP_BOT_BACK_CAMERA =
        new Transform3d(
            -0.25, // X meters from center
            0.0, // Y meters from center
            0.45, // Z meters from floor
            new Rotation3d(0, 0, Math.PI) // Facing backward (180 degrees)
            );

    for (String name : names) {
      // Updated: All cameras default to TRUE for competition
      boolean defaultEnabled = true;

      LL_enabled.put(
          name, new NtTunableBoolean("SmartDashboard/Vision/Enabled_" + name, defaultEnabled));
      LL_status_pubs.put(
          name, inst.getBooleanTopic("SmartDashboard/Vision/Status_" + name).publish());
      LL_online.put(name, false);

      LimelightHelpers.SetIMUAssistAlpha(name, IMU_ASSIST_ALPHA);
      lastBeats.put(name, LimelightHelpers.getHeartbeat(name));

      // Map the transforms correctly
      if (name.equals(Hardware.LIMELIGHT_A)) transforms.put(name, COMP_BOT_FRONT_CAMERA);
      else if (name.equals(Hardware.LIMELIGHT_B)) transforms.put(name, COMP_BOT_LEFT_CAMERA);
      else if (name.equals(Hardware.LIMELIGHT_C)) transforms.put(name, COMP_BOT_BACK_CAMERA);
    }
  }

  // This is specificaly called periodically in robot.java
  public void update() {
    updateLimeLightStatus();

    // refresh all signals
    BaseStatusSignal.refreshAll(gyro_yaw, gyro_pitch, gyro_roll, gyro_vx, gyro_vy, gyro_vz);

    double yaw = gyro_yaw.getValueAsDouble();
    double pitch = gyro_pitch.getValueAsDouble();
    double roll = gyro_roll.getValueAsDouble();
    double vx = gyro_vx.getValueAsDouble();
    double vy = gyro_vy.getValueAsDouble();
    double vz = gyro_vz.getValueAsDouble();

    for (String name : names) {
      if (isEnabled(name) && isOnline(name)) {
        // orientation to be updated immediately before retrieval
        LimelightHelpers.SetRobotOrientation(name, yaw, vz, pitch, vy, roll, vx);

        PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        processVision(name, estimate, vz, roll, pitch);
      }
    }
  }

  private void processVision(
      String name, PoseEstimate estimate, double angVel, double roll, double pitch) {
    // if no tags
    if (estimate == null || estimate.tagCount == 0) return;
    // if spinning too fast
    if (Math.abs(angVel) > MAX_ANGULAR_VELOCITY) return;
    // if tilted
    if (Math.abs(roll) > MAX_TILT || Math.abs(pitch) > MAX_TILT) return;

    double dist = estimate.avgTagDist;
    // if too far
    if (dist >= MAX_DISTANCE_METERS) return;

    // if the pose is outside bounds (now checks with a configured margin)
    if (!isPoseOnField(estimate.pose)) return;

    // Duplicate pose check
    Pose2d lastPose = lastVisionPoses.get(name);
    if (lastPose != null && lastPose.equals(estimate.pose)) return;

    // Velocity Plausibility Gate
    double lastTs = lastVisionTimestamps.getOrDefault(name, 0.0);
    double dt = estimate.timestampSeconds - lastTs;
    if (lastPose != null && dt > 0 && dt <= 1.0) {
      double impliedSpeed =
          estimate.pose.getTranslation().getDistance(lastPose.getTranslation()) / dt;
      if (impliedSpeed > MAX_VISION_IMPLIED_SPEED) return;
    }

    // -- Standard deviation scaling -- //

    // calculate root mean sum error
    double stdDevXY = STD_DEV_SCALAR * Math.pow(dist, DISTANCE_POWER_FACTOR);
    double RMSE = 1;
    double actualRMSE = 0; // Track actual RMSE separately to inflate correctly
    double harmonicSum = 1;

    if (FILTER_ENABLED.get("RMSE Filter").get()) {
      actualRMSE = RMSE(estimate);
      RMSE = Math.max(actualRMSE, MIN_RMSE);
      // check to see if RMSE is too large
      if (RMSE > RMSE_REJECT_THRESHOLD) return;
    }

    if (FILTER_ENABLED.get("Harmonic sum Filter").get()) {
      // Calculate the harmonic sum of all tags detected by this limelight
      harmonicSum = harmonicSum(estimate.rawFiducials);
      // if limelight isn't certain about ANY tags
      if (harmonicSum == 0) return;
    }

    // calculate std dev: It's broken into three parts, penalize for distance, reward for certain
    // tags, RMSE of tags
    stdDevXY = Math.max(stdDevXY / Math.sqrt(harmonicSum) * Math.max(RMSE, MIN_RMSE), MIN_STD_DEV);

    // Dynamic Spread Inflation: Inflate standard deviation if tags disagree mildly
    if (FILTER_ENABLED.get("RMSE Filter").get() && actualRMSE > SPREAD_INFLATE_START) {
      double spreadInflation = Math.pow(actualRMSE / SPREAD_INFLATE_START, 2.0);
      stdDevXY *= spreadInflation;
    }

    robotPoseOutOfBoundsReset(estimate, estimate.tagCount);

    // add the vision measurement to robot pose
    driveBase.addVisionMeasurement(
        estimate.pose,
        estimate.timestampSeconds,
        VecBuilder.fill(stdDevXY, stdDevXY, ROTATION_STD_DEV_REJECT) // Trust gyro for rotation
        );

    // Update state cache for velocity/duplicate checks
    lastVisionPoses.put(name, estimate.pose);
    lastVisionTimestamps.put(name, estimate.timestampSeconds);
  }

  // --- STD-DEVS helpers --- //

  private double harmonicSum(RawFiducial[] tags) {
    // Don't do anything if there are no tags
    if (tags == null) {
      return 0;
    }
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

      double estimatedDist = tag.distToRobot;
      error += Math.pow(actualDist3d - estimatedDist, 2);
    }
    return Math.sqrt(error / tags.length);
  }

  private void robotPoseOutOfBoundsReset(PoseEstimate estimate, int tagCount) {
    double avgAmbiguity =
        Arrays.stream(estimate.rawFiducials)
            .mapToDouble(tag -> tag.ambiguity)
            .average()
            .orElse(Double.MAX_VALUE);
    Pose2d odomPose = driveBase.getState().Pose;
    boolean odomOffField = !isPoseOnField(odomPose);
    if (!odomOffField) {
      return; // return early
    }
    boolean visionTrusted =
        avgAmbiguity < RESET_MAX_AMBIGUITY_THRESHOLD
            && tagCount >= RESET_MIN_TAGS
            && isPoseOnField(estimate.pose);
    if (visionTrusted) {
      driveBase.resetPose(estimate.pose);
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

  // ---- Limelight Status helpers ---- //
  private void updateLimeLightStatus() {
    for (String name : names) {
      if (!isEnabled(name)) {
        changeStatus(name, false);
        continue;
      }
      double currentBeat = LimelightHelpers.getHeartbeat(name) / 1_000_000.0;
      boolean online = (currentBeat - lastBeats.getOrDefault(name, 0.0)) < STALENESS_THRESHOLD;
      lastBeats.put(name, currentBeat);
      changeStatus(name, online);
    }
  }

  private void changeStatus(String name, boolean status) {
    LL_online.put(name, status);
    if (LL_status_pubs.containsKey(name)) {
      LL_status_pubs.get(name).set(status);
    }
  }

  public boolean isEnabled(String name) {
    NtTunableBoolean tunable = LL_enabled.get(name);
    return tunable != null && tunable.get();
  }

  public boolean isOnline(String name) {
    return LL_online.getOrDefault(name, false);
  }

  // --- Competition setup --- //

  public void limelightRobotDisabled() {
    for (String name : names) {
      if (isOnline(name) && isEnabled(name)) {
        setupLimelightForAprilTags(name, true);
      }
    }
  }

  public void limelightDisabledExit() {
    for (String name : names) {
      if (isOnline(name) && isEnabled(name)) {
        setupLimelightForAprilTags(name, false);
      }
    }
  }

  private void setupLimelightForAprilTags(String limelightName, boolean isEnteringDisabled) {
    if (isEnteringDisabled) {
      LimelightHelpers.SetThrottle(limelightName, THROTTLE_VALUE_ON);
      LimelightHelpers.SetIMUMode(limelightName, DISABLED_IMU_MODE);
    } else {
      LimelightHelpers.SetThrottle(limelightName, THROTTLE_VALUE_OFF);
      LimelightHelpers.SetIMUMode(limelightName, ENABLED_IMU_MODE); // MT2 External IMU Mode
    }
    LimelightHelpers.setPipelineIndex(limelightName, APRIL_TAG_COMP_PIPELINE);
  }
}
