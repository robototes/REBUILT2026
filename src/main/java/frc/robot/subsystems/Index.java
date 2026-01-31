package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;
import frc.robot.util.NtTunableDouble;

public class Index extends SubsystemBase {

  private final TalonFX Index;
  private NtTunableDouble speed;

  public Index() {
    Index = new TalonFX(Hardware.FEEDER_MOTOR_ID);
    configureMotors();

    speed = new NtTunableDouble("/index/speed", 0.4);
  }

  private void configureMotors() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    TalonFXConfigurator indexerConfigurator = Index.getConfigurator();

    // set current limits
    config.CurrentLimits.SupplyCurrentLimit = 30;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 60;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    // create brake mode for motors
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    indexerConfigurator.apply(config);
  }

  public Command setPowerCommand(double power) {
    return runEnd(
        () -> {
          System.out.println("SHOOTING");
          Index.set(power);
        },
        () -> {
          Index.set(0);
        });
  }

  public Command setTunerPowerCommand() {
    return runEnd(
        () -> {
          Index.set(speed.get());
        },
        () -> {
          Index.set(0);
        });
  }
}
