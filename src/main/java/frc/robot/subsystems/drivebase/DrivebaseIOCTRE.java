package frc.robot.subsystems.drivebase;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

public class DrivebaseIOCTRE implements DrivebaseIO {
  private final Supplier<SwerveDriveState> stateSupplier;

  public DrivebaseIOCTRE(Supplier<SwerveDriveState> stateSupplier) {
    this.stateSupplier = stateSupplier;
  }

  @Override
  public void updateInputs(DrivebaseIOInputs inputs) {
    SwerveDriveState state = stateSupplier.get();
    inputs.Pose = state.Pose;
    inputs.Speeds = state.Speeds;
    inputs.RawHeading = state.RawHeading;
    inputs.Timestamp = state.Timestamp;
    inputs.OdometryPeriod = state.OdometryPeriod;
    inputs.SuccessfulDaqs = state.SuccessfulDaqs;
    inputs.FailedDaqs = state.FailedDaqs;
  }
}
