package frc.robot.subsystems.drivebase;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import frc.robot.util.StatusSignalRefresher;
import java.util.Objects;

public class DrivebaseIOCTRE implements DrivebaseIO {
  private SwerveDriveState state;
  private CommandSwerveDrivetrain drivetrain;
  private StatusSignal<Current>[] driveSupply;
  private StatusSignal<Current>[] driveStator;
  private StatusSignal<Current>[] steerStator;
  private StatusSignal<Temperature>[] driveTemp;

  public DrivebaseIOCTRE(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = Objects.requireNonNull(drivetrain, "drivetrain is null");
    this.state = drivetrain.getStateCopy();
    var refresher = StatusSignalRefresher.getInstance();

    int n = drivetrain.getModules().length;
    driveSupply = new StatusSignal[n];
    driveStator = new StatusSignal[n];
    steerStator = new StatusSignal[n];
    driveTemp = new StatusSignal[n];

    for (int i = 0; i < n; i++) {
      var drive = drivetrain.getModule(i).getDriveMotor();
      var steer = drivetrain.getModule(i).getSteerMotor();

      driveSupply[i] = drive.getSupplyCurrent();
      driveStator[i] = drive.getStatorCurrent();
      steerStator[i] = steer.getStatorCurrent();
      driveTemp[i] = drive.getDeviceTemp();

      BaseStatusSignal.setUpdateFrequencyForAll(50, driveSupply[i], driveStator[i], steerStator[i]);
      refresher.addStatusSignals(driveSupply[i], driveStator[i], steerStator[i]);
      refresher.addStatusSignals(5, driveTemp[i]); // slow signal
    }
  }

  @Override
  public void updateInputs(DrivebaseIOInputs inputs, SwerveDriveState state) {
    this.state = state;
    inputs.Pose = state.Pose;
    inputs.Speeds = state.Speeds;
    inputs.RawHeading = state.RawHeading;
    inputs.Timestamp = state.Timestamp;
    inputs.OdometryPeriod = state.OdometryPeriod;
    inputs.SuccessfulDaqs = state.SuccessfulDaqs;
    inputs.FailedDaqs = state.FailedDaqs;
    for (int i = 0; i < driveSupply.length; i++) {
      inputs.driveSupplyCurrentAmps[i] = driveSupply[i].getValueAsDouble();
      inputs.driveStatorCurrentAmps[i] = driveStator[i].getValueAsDouble();
      inputs.steerStatorCurrentAmps[i] = steerStator[i].getValueAsDouble();
      inputs.driveTempCelsius[i] = driveTemp[i].getValueAsDouble();
    }
  }
}
