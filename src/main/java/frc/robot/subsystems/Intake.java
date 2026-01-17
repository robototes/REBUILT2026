package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;
import frc.robot.generated.CompTunerConstants;

public class Intake extends SubsystemBase {

  private final TalonFX intake;

  public Intake() {
    intake = new TalonFX(Hardware.INTAKE_MOTOR_ID, CompTunerConstants.kCANBus);
    configureMotors();
  }

  private void configureMotors() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    TalonFXConfigurator indexerConfigurator = intake.getConfigurator();

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
          intake.set(power);
        },
        () -> {
          intake.set(0);
        });
  }
}
