package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import frc.robot.util.robotType.RobotType;
import java.util.HashMap;

public class VisionSubsystemV2 extends SubsystemBase {
  // Required subsystem(s)
  private final CommandSwerveDrivetrain driveBase;

  // -- LIMELIGHT ENABLED MAP -- //
  public static final HashMap<String, Boolean> LL_status = new HashMap<>();

  static {
    if (RobotType.isAlpha()) {
      LL_status.put(Hardware.LIMELIGHT_C, true);
      LL_status.put(Hardware.LIMELIGHT_A, false);
      LL_status.put(Hardware.LIMELIGHT_B, false);
    } else {
      LL_status.put(Hardware.LIMELIGHT_C, false);
      LL_status.put(Hardware.LIMELIGHT_A, true);
      LL_status.put(Hardware.LIMELIGHT_B, true);
    }
  }

  // Hardware objects
  private final Pigeon2 gyro;

  // Limelight names (derived from map keys)
  private final String[] names = {Hardware.LIMELIGHT_A, Hardware.LIMELIGHT_B, Hardware.LIMELIGHT_C};

  // Pigeon 2 gyro data
  private final StatusSignal<Angle> gyro_yaw;
  private final StatusSignal<Angle> gyro_pitch;
  private final StatusSignal<Angle> gyro_roll;
  private final StatusSignal<AngularVelocity> gyro_AngularVelocity_X;
  private final StatusSignal<AngularVelocity> gyro_AngularVelocity_Y;
  private final StatusSignal<AngularVelocity> gyro_AngularVelocity_Z;

  // Magic numbers
  private final double MAX_ANGULAR_VELOCITY = 720; // Degrees/s
  private final double MAX_DISTANCE_METERS = 8.5;
  private final double MAX_TILT = 10;
  private final double IMU_ASSIST_ALPHA = 3;
  private final double SCALAR_RANK_1 = 0.001;
  private final double SCALAR_RANK_2 = 0.03;
  private final double DEVIATION_BASE_1 = 0.05;
  private final double DEVIATION_BASE_2 = 0.15;
  private final int THROTTLE_ON = 150;
  private final int THROTTLE_OFF = 0;
  private final int APRILTAG_PIPELINE = 0;
  // Field dimensions
  private final double FIELD_LENGTH = 16; // Meters
  private final double FIELD_WIDTH = 8; // Meters
  private final double ZERO_THRESHOLD = 0.5;

  // Cached Pigeon 2 values
  private double yaw;
  private double pitch;
  private double roll;
  private double vy;
  private double vx;
  private double vz;

  /**
   * Constructor for VisionUpdate. This class now extends SubsystemBase and will run automatically
   * via the scheduler.
   */
  public VisionSubsystemV2(CommandSwerveDrivetrain drivetrain) {
    // Set drivebase object
    this.driveBase = drivetrain;

    // Grab third party IMU
    this.gyro = driveBase.getPigeon2();

    // Fill the array with status signals providing data for the class
    gyro_yaw = gyro.getYaw(); // Deg
    gyro_pitch = gyro.getPitch(); // Deg
    gyro_roll = gyro.getRoll(); // Deg
    gyro_AngularVelocity_X = gyro.getAngularVelocityXWorld(); // Deg/s
    gyro_AngularVelocity_Y = gyro.getAngularVelocityYWorld(); // Deg/s
    gyro_AngularVelocity_Z = gyro.getAngularVelocityZWorld(); // Deg/s

    // Set IMU mode for all enabled cameras
    for (String name : names) {
      if (LL_status.getOrDefault(name, false)) {
        LimelightHelpers.SetIMUMode(name, 4);
        LimelightHelpers.SetIMUAssistAlpha(name, IMU_ASSIST_ALPHA);
      }
    }
  }

  public void update() {
    // refresh data
    BaseStatusSignal.refreshAll(
        gyro_yaw,
        gyro_pitch,
        gyro_roll,
        gyro_AngularVelocity_X,
        gyro_AngularVelocity_Y,
        gyro_AngularVelocity_Z);

    // Grab values
    yaw = gyro_yaw.getValueAsDouble();
    pitch = gyro_pitch.getValueAsDouble();
    roll = gyro_roll.getValueAsDouble();
    vy = gyro_AngularVelocity_Y.getValueAsDouble();
    vx = gyro_AngularVelocity_X.getValueAsDouble();
    vz = gyro_AngularVelocity_Z.getValueAsDouble();

    // process vision for all enabled cameras
    for (String name : names) {
      if (LL_status.getOrDefault(name, false)) {
        // Apply gyro data to the robot pose for this specific camera
        LimelightHelpers.SetRobotOrientation(name, yaw, vz, pitch, vy, roll, vx);

        // Get MegaTag2 estimate and process
        LimelightHelpers.PoseEstimate estimate =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

        processVision(estimate, vz);
      }
    }
  }

  private void processVision(PoseEstimate estimate, double angularVelocity) {
    // Basic safety checks
    if (estimate == null || estimate.tagCount == 0) return;
    // Don't accept if robot is spinning too fast
    if (Math.abs(angularVelocity) > MAX_ANGULAR_VELOCITY) return;

    // Trust scaling: If tags are far away (> 4 meters), trust them less.
    // (x, y, theta) standard deviations. 999999 means "ignore camera rotation".
    double distance = estimate.avgTagDist;
    double xyStdDev = 0;

    // If too far, do nothing
    if (distance >= MAX_DISTANCE_METERS) {
      return;
    }
    // If tag out of bounds, it's wrong
    double estimatePoseX = estimate.pose.getX();
    double estimatePoseY = estimate.pose.getY();
    if (estimatePoseX < ZERO_THRESHOLD
        || estimatePoseX > FIELD_LENGTH
        || estimatePoseY < ZERO_THRESHOLD
        || estimatePoseY > FIELD_WIDTH) {
      return;
    }

    // If the camera thinks the robot is tilting more than MAX_TILT, ignore it.
    if (Math.abs(roll) > MAX_TILT || Math.abs(pitch) > MAX_TILT) {
      return;
    }

    // merge the x and y distance
    if (estimate.tagCount >= 2) {
      xyStdDev = DEVIATION_BASE_1 + (Math.pow(distance, 2) * SCALAR_RANK_1);
    } else {
      xyStdDev = DEVIATION_BASE_2 + (Math.pow(distance, 2) * SCALAR_RANK_2);
    }

    // rotation should be 999999 because we already trust the pigeon 2. Keep things simple
    driveBase.addVisionMeasurement(
        estimate.pose, estimate.timestampSeconds, VecBuilder.fill(xyStdDev, xyStdDev, 999999));
  }

  /** Pre-match setup for all enabled Limelights */
  public void limelightRobotDisabled() {
    for (String name : names) {
      if (LL_status.getOrDefault(name, false)) {
        setupLimelightForAprilTags(name, true);
      }
    }
  }

  public void limelightDisabledExit() {
    for (String name : names) {
      if (LL_status.getOrDefault(name, false)) {
        setupLimelightForAprilTags(name, false);
      }
    }
  }

  /** Configures a specific Limelight for either disabled (low heat) or enabled (active) mode. */
  public void setupLimelightForAprilTags(String limelightName, boolean isEnteringDisabled) {
    if (isEnteringDisabled) {
      // Throttle to reduce heat
      LimelightHelpers.SetThrottle(limelightName, THROTTLE_ON);
      // seed internal limelight imu for mt2
      LimelightHelpers.SetIMUMode(limelightName, 1);
      LimelightHelpers.setPipelineIndex(limelightName, APRILTAG_PIPELINE);
    } else {
      // get rid of throttle to get rid of throttle "glazing"
      LimelightHelpers.SetThrottle(limelightName, THROTTLE_OFF);
      // Limelight Use internal IMU + external IMU
      LimelightHelpers.SetIMUMode(limelightName, 4);
    }
  }
}
