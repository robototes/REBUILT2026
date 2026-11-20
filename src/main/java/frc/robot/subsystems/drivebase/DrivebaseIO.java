package frc.robot.subsystems.drivebase;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.littletonrobotics.junction.AutoLog;

public interface DrivebaseIO {

  @AutoLog
  public static class DrivebaseIOInputs {
    public Pose2d Pose = new Pose2d();
    public ChassisSpeeds Speeds = new ChassisSpeeds();
    public Rotation2d RawHeading = new Rotation2d();
    public double Timestamp = 0;
    public double OdometryPeriod = 0;
    public int SuccessfulDaqs = 0;
    public int FailedDaqs = 0;
  }

  public default void updateInputs(DrivebaseIOInputs inputs, SwerveDriveState state) {}
}
