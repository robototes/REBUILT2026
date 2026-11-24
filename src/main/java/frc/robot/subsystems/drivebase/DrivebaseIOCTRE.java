package frc.robot.subsystems.drivebase;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import frc.robot.util.StatusSignalRefresher;
import java.util.Objects;

public class DrivebaseIOCTRE implements DrivebaseIO {

  private SwerveDriveState state;
  private final CommandSwerveDrivetrain drivetrain;
  private final StatusSignal<Current>[] driveSupply;
  private final StatusSignal<Current>[] driveStator;
  private final StatusSignal<Current>[] steerStator;
  private final StatusSignal<Temperature>[] driveTemp;

  @SuppressWarnings("unchecked")
  public DrivebaseIOCTRE(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = Objects.requireNonNull(drivetrain, "drivetrain is null");
    this.state = drivetrain.getStateCopy();

    int n = drivetrain.getModules().length;
    driveSupply = new StatusSignal[n];
    driveStator = new StatusSignal[n];
    steerStator = new StatusSignal[n];
    driveTemp = new StatusSignal[n];

    var refresher = StatusSignalRefresher.getInstance();

    for (int i = 0; i < n; i++) {
      TalonFX drive = drivetrain.getModule(i).getDriveMotor();
      TalonFX steer = drivetrain.getModule(i).getSteerMotor();

      driveSupply[i] = drive.getSupplyCurrent();
      driveStator[i] = drive.getStatorCurrent();
      steerStator[i] = steer.getStatorCurrent();
      driveTemp[i] = drive.getDeviceTemp();

      BaseStatusSignal.setUpdateFrequencyForAll(50, driveSupply[i], driveStator[i], steerStator[i]);
      BaseStatusSignal.setUpdateFrequencyForAll(4, driveTemp[i]);

      refresher.addStatusSignals(driveSupply[i], driveStator[i], steerStator[i]);
      refresher.addStatusSignals(12, driveTemp[i]); // every 12 loops ~ 4 Hz
    }

    optimizeBusUtilization(n);
  }

  /**
   * Pin the signals the odometry thread needs, then disable everything else on the swerve motors
   * and Pigeon. Must run after the extra signals above have their own frequencies set.
   */
  private void optimizeBusUtilization(int moduleCount) {

    double odomHz = drivetrain.getOdometryFrequency();

    TalonFX[] motors = new TalonFX[moduleCount * 2];
    for (int i = 0; i < moduleCount; i++) {
      TalonFX drive = drivetrain.getModule(i).getDriveMotor();
      TalonFX steer = drivetrain.getModule(i).getSteerMotor();
      motors[2 * i] = drive;
      motors[2 * i + 1] = steer;

      BaseStatusSignal.setUpdateFrequencyForAll(
          odomHz,
          drive.getPosition(false),
          drive.getVelocity(false),
          steer.getPosition(false),
          steer.getVelocity(false));
    }

    Pigeon2 pigeon = drivetrain.getPigeon2();
    BaseStatusSignal.setUpdateFrequencyForAll(
        odomHz, pigeon.getYaw(false), pigeon.getAngularVelocityZWorld(false));
    ParentDevice.optimizeBusUtilizationForAll(motors);
    pigeon.optimizeBusUtilization();
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
