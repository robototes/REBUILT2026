package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;
import frc.robot.util.NtTunableDouble;

public class Serializer extends SubsystemBase {

  private final TalonFX Serializer;
  private NtTunableDouble speed;

  public Serializer() {
    Serializer = new TalonFX(Hardware.SPINDEXER_MOTOR_ID);
    configureMotors();

    speed = new NtTunableDouble("Serializer/speed", 0.4);
  }

  private void configureMotors() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    TalonFXConfigurator SerializererConfigurator = Serializer.getConfigurator();

    // set current limits
    config.CurrentLimits.SupplyCurrentLimit = 30;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 60;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    // create brake mode for motors
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    SerializererConfigurator.apply(config);
  }

  public Command setPowerCommand(double power) {
    return runEnd(
        () -> {
          System.out.println("shooting");
          Serializer.set(power);
        },
        () -> {
          Serializer.set(0);
        });
  }

  public Command setTunerPowerCommand() {
    return runEnd(
        () -> {
          Serializer.set(speed.get());
        },
        () -> {
          Serializer.set(0);
        });
  }
}
