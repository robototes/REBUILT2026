package frc.robot.subsystems.drivebase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.littletonrobotics.junction.AutoLog;

public interface DrivebaseIO {

  @AutoLog
  public static class DrivebaseIOInputs {
    public Pose2d Pose;
    public ChassisSpeeds Speeds;
    public Rotation2d RawHeading;
    public double Timestamp;
    public double OdometryPeriod;
    public int SuccessfulDaqs;
    public int FailedDaqs;
  }

  public default void updateInputs(DrivebaseIOInputs inputs) {}
}
