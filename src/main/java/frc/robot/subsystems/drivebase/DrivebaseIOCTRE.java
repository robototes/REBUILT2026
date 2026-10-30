package frc.robot.subsystems.drivebase;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import frc.robot.subsystems.drivebase.DrivebaseIO.DrivebaseIOInputs;

public class DrivebaseIOCTRE implements DrivebaseIO {
  private SwerveDriveState state;

  public DrivebaseIOCTRE(SwerveDriveState state) {
    this.state = state;
  }

  @Override
  public void updateInputs(DrivebaseIOInputs inputs) {
    inputs.Pose = state.Pose;
    inputs.Speeds = state.Speeds;
    inputs.RawHeading = state.RawHeading;
    inputs.Timestamp = state.Timestamp;
    inputs.OdometryPeriod = state.OdometryPeriod;
    inputs.SuccessfulDaqs = state.SuccessfulDaqs;
    inputs.FailedDaqs = state.FailedDaqs;
  }
}
