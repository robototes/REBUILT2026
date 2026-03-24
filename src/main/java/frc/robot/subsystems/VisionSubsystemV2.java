package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import frc.robot.util.LimelightHelpers.RawFiducial;
import frc.robot.util.robotType.RobotType;
import frc.robot.util.tuning.NtTunableBoolean;
import java.util.HashMap;
import java.util.Map;

public class VisionSubsystemV2 extends SubsystemBase {
  private final CommandSwerveDrivetrain driveBase;

  // -- CONFIGURATION MAPS -- //
  private final Map<String, NtTunableBoolean> LL_enabled = new HashMap<>();
  private final Map<String, BooleanPublisher> LL_status_pubs = new HashMap<>();
  private final Map<String, Boolean> LL_online = new HashMap<>();

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
  private static final double STALENESS_THRESHOLD = 1.0;
  private static final double AMBIGUITY_THRESHOLD = 0.2;
  private static final double STD_DEV_SCALAR = 0.035;
  private static final double POWER = 1.4;
  private static final double IMU_ASSIST_ALPHA = 0.05;

  // Field dimensions
  private static final double FIELD_LENGTH = 16.54;
  private static final double FIELD_WIDTH = 8.02;

  public VisionSubsystemV2(CommandSwerveDrivetrain drivetrain) {
    this.driveBase = drivetrain;
    this.gyro = driveBase.getPigeon2();

    var inst = NetworkTableInstance.getDefault();

    for (String name : names) {
      // Determine default based on bot type
      boolean defaultEnabled = false;
      if (RobotType.isAlpha()) {
        if (name.equals(Hardware.LIMELIGHT_C)) defaultEnabled = true;
      } else {
        if (name.equals(Hardware.LIMELIGHT_A) || name.equals(Hardware.LIMELIGHT_B))
          defaultEnabled = true;
      }

      // Create the tunable first, THEN put it in the map
      LL_enabled.put(
          name, new NtTunableBoolean("SmartDashboard/Vision/Enabled_" + name, defaultEnabled));
      LL_status_pubs.put(
          name, inst.getBooleanTopic("SmartDashboard/Vision/Status_" + name).publish());
      LL_online.put(name, false);
      LimelightHelpers.SetIMUAssistAlpha(name, IMU_ASSIST_ALPHA);
    }

    // Setup Gyro Signals
    gyro_yaw = gyro.getYaw();
    gyro_pitch = gyro.getPitch();
    gyro_roll = gyro.getRoll();
    gyro_vx = gyro.getAngularVelocityXWorld();
    gyro_vy = gyro.getAngularVelocityYWorld();
    gyro_vz = gyro.getAngularVelocityZWorld();
  }

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
        processVision(estimate, vz, roll, pitch);
      }
    }
  }

  private void processVision(PoseEstimate estimate, double angVel, double roll, double pitch) {
    // if no tags
    if (estimate == null || estimate.tagCount == 0) return;
    // if spinning too fast
    if (Math.abs(angVel) > MAX_ANGULAR_VELOCITY) return;
    // if tilted
    if (Math.abs(roll) > MAX_TILT || Math.abs(pitch) > MAX_TILT) return;

    double dist = estimate.avgTagDist;
    // if too far
    if (dist >= MAX_DISTANCE_METERS) return;

    // if the pose is outside bounds
    if (estimate.pose.getX() < 0
        || estimate.pose.getX() > FIELD_LENGTH
        || estimate.pose.getY() < 0
        || estimate.pose.getY() > FIELD_WIDTH) return;

    // -- Standard deviation scaling -- //

    // Calculate the harmonic sum of all tags detected by this limelight
    double harmonicSum = HarmonicSum(estimate.rawFiducials);
    // if limelight isn't certain about ANY tags
    if (harmonicSum == 0) return;
    // calculate std dev: It's broken into two parts, penalize for distance and reward for certain
    // tags
    double stdDevXY = STD_DEV_SCALAR * Math.pow(dist, POWER) / Math.sqrt(harmonicSum);

    // add the vision measurement to robot pose
    driveBase.addVisionMeasurement(
        estimate.pose,
        estimate.timestampSeconds,
        VecBuilder.fill(stdDevXY, stdDevXY, 999999) // Trust gyro for rotation
        );
  }

  private double HarmonicSum(RawFiducial[] detectedTags) {
    // Don't do anything if there are no tags
    if (detectedTags == null) return 0;
    double sum = 0;
    for (RawFiducial tag : detectedTags) {
      if (tag.ambiguity > AMBIGUITY_THRESHOLD) continue;
      sum += 1 / Math.pow(tag.distToCamera, 2);
    }
    return sum;
  }

  public void updateLimeLightStatus() {
    for (String name : names) {
      if (!isEnabled(name)) {
        changeStatus(name, false);
        continue;
      }

      var table = NetworkTableInstance.getDefault().getTable(name);
      long lastChange = table.getEntry("hb").getLastChange();
      double lastChangeSecs = lastChange / 1_000_000.0;
      boolean online = (Timer.getFPGATimestamp() - lastChangeSecs) < STALENESS_THRESHOLD;

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

  public void setupLimelightForAprilTags(String limelightName, boolean isEnteringDisabled) {
    if (isEnteringDisabled) {
      LimelightHelpers.SetThrottle(limelightName, 150);
      LimelightHelpers.SetIMUMode(limelightName, 1);
    } else {
      LimelightHelpers.SetThrottle(limelightName, 0);
      LimelightHelpers.SetIMUMode(limelightName, 4); // MT2 External IMU Mode
    }
    LimelightHelpers.setPipelineIndex(limelightName, 0);
  }
}
