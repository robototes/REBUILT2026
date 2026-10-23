package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.AnalogInput;

public class TurretIOTalonFX implements TurretIO {

  private TalonFX TurretMotor;
  private AnalogInput LimitSwitch;

  public TurretIOTalonFX(TalonFX motor, AnalogInput lSwitch) {
    TurretMotor = motor;
    LimitSwitch = lSwitch;
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.positionSignal = TurretMotor.getPosition().getValueAsDouble();
    inputs.velocitySignal = TurretMotor.getVelocity().getValueAsDouble();
    inputs.statorCurrentSignal = TurretMotor.getStatorCurrent().getValueAsDouble();
    inputs.limitSwitchCurrent = LimitSwitch.getVoltage();
  }
}
