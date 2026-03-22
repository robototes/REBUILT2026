import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;

public class VisionSubsystemV2 {
  // Required subsystem(s)
  private final CommandSwerveDrivetrain driveBase;

  // Hardware objects
  private final Pigeon2 gyro;

  // Limelight names
  private final String N_LL_A = "limelightA";
  private final String N_LL_B = "limelightB";

  // Pigeon 2 gyro data
  private final StatusSignal<Angle> gyro_yaw;
  private final StatusSignal<Angle> gyro_pitch;
  private final StatusSignal<Angle> gyro_roll;
  private final StatusSignal<AngularVelocity> gyro_AngularVelocity_X;
  private final StatusSignal<AngularVelocity> gyro_AngularVelocity_Y;
  private final StatusSignal<AngularVelocity> gyro_AngularVelocity_Z;

  public VisionSubsystemV2(CommandSwerveDrivetrain driveBase) {
    // Grab third party IMU
    gyro = driveBase.getPigeon2();

    // Fill the array with statussignals providing data for the clas
    gyro_yaw = gyro.getYaw(); // Deg
    gyro_pitch = gyro.getPitch(); // Deg
    gyro_roll = gyro.getRoll(); // Deg
    gyro_AngularVelocity_X = gyro.getAngularVelocityXWorld(); // Deg/s
    gyro_AngularVelocity_Y = gyro.getAngularVelocityYWorld(); // Deg/s
    gyro_AngularVelocity_Z = gyro.getAngularVelocityZWorld(); // Deg/s
  }

  public void update() {
    boolean rejectUpdate = false;
    double angularVelocity = gyro.getAngularVelocityZDevice().getValueAsDouble();
    LimelightHelpers.PoseEstimate LL_A_mt2 =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(N_LL_A);
    LimelightHelpers.PoseEstimate LL_B_mt2 =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(N_LL_B);
    LimelightHelpers.SetRobotOrientation(
        N_LL_A,
        gyro_yaw.refresh().getValueAsDouble(),
        gyro_AngularVelocity_X.refresh().getValueAsDouble(),
        gyro_pitch.refresh().getValueAsDouble(),
        gyro_AngularVelocity_Z.refresh().getValueAsDouble(),
        gyro_roll.refresh().getValueAsDouble(),
        gyro_AngularVelocity_Y.refresh().getValueAsDouble());

    // if the angular velocity o fhte robot is too much to handle
    if (Math.abs(angularVelocity) > 360) {
      rejectUpdate = true;
    }
    // If there are no tags visible
    if (mt2.tagCount == 0) {
      rejectUpdate = true;
    }
    if (!rejectUpdate) {
      // Get each bot pose estimate based off of 2 different cameras
      PoseEstimate ll_a_estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(N_LL_A);
      PoseEstimate ll_b_estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(N_LL_B);

      LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(N_LL_B);
    }
  }
}
