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

public class VisionSubsystemV2 extends SubsystemBase {
  // Required subsystem(s)
  private final CommandSwerveDrivetrain driveBase;

  // -- LIMELIGHT ENABLED -- //
  public boolean LIMELIGHT_A_ENABLED = true;
  public boolean LIMELIGHT_B_ENABLED = true;
  public boolean LIMELIGHT_C_ENABLED = false;

  // Hardware objects
  private final Pigeon2 gyro;

  // Limelight names
  private final String N_LL_A;
  private final String N_LL_B;
  private final String N_LL_C;

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
  private static final double DEVIATION_BASE_2 = 0.15;
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

    // set names
    N_LL_A = Hardware.LIMELIGHT_A;
    N_LL_B = Hardware.LIMELIGHT_B;
    N_LL_C = Hardware.LIMELIGHT_C;

    // Fill the array with status signals providing data for the class
    gyro_yaw = gyro.getYaw(); // Deg
    gyro_pitch = gyro.getPitch(); // Deg
    gyro_roll = gyro.getRoll(); // Deg
    gyro_AngularVelocity_X = gyro.getAngularVelocityXWorld(); // Deg/s
    gyro_AngularVelocity_Y = gyro.getAngularVelocityYWorld(); // Deg/s
    gyro_AngularVelocity_Z = gyro.getAngularVelocityZWorld(); // Deg/s

    // Set IMU mode. Uses pigeon 2's data to support robot prediction
    if (LIMELIGHT_A_ENABLED) {
      LimelightHelpers.SetIMUMode(N_LL_A, 4);
      LimelightHelpers.SetIMUAssistAlpha(N_LL_A, IMU_ASSIST_ALPHA);
    }
    if (LIMELIGHT_B_ENABLED) {
      LimelightHelpers.SetIMUMode(N_LL_B, 4);
      LimelightHelpers.SetIMUAssistAlpha(N_LL_B, IMU_ASSIST_ALPHA);
    }
    if (LIMELIGHT_C_ENABLED) {
      LimelightHelpers.SetIMUMode(N_LL_C, 4);
      LimelightHelpers.SetIMUAssistAlpha(N_LL_C, IMU_ASSIST_ALPHA);
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

    // get the gyro data from the IMU, and apply to the robot pose
    LimelightHelpers.SetRobotOrientation(N_LL_A, yaw, vz, pitch, vy, roll, vx);

    // process vision
    if (LIMELIGHT_A_ENABLED) {
      LimelightHelpers.PoseEstimate LL_A_mt2 =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(N_LL_A);
      LimelightHelpers.SetRobotOrientation(N_LL_A, yaw, vz, pitch, vy, roll, vx);
      processVision(LL_A_mt2, vz);
    }
    if (LIMELIGHT_B_ENABLED) {
      LimelightHelpers.PoseEstimate LL_B_mt2 =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(N_LL_B);
      LimelightHelpers.SetRobotOrientation(N_LL_B, yaw, vz, pitch, vy, roll, vx);
      processVision(LL_B_mt2, vz);
    }
    if (LIMELIGHT_C_ENABLED) {
      LimelightHelpers.PoseEstimate LL_C_mt2 =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(N_LL_C);
      LimelightHelpers.SetRobotOrientation(N_LL_C, yaw, vz, pitch, vy, roll, vx);
      processVision(LL_C_mt2, vz);
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
}
