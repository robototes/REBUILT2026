package frc.robot.subsystems.spindexer;

import com.ctre.phoenix6.hardware.TalonFX;

public class SpindexerIOTalonFX implements SpindexerIO {

  private TalonFX SpindexerMotor;

  public SpindexerIOTalonFX(TalonFX motor) {
    SpindexerMotor = motor;
  }

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    inputs.rps = SpindexerMotor.getVelocity().getValueAsDouble();
    inputs.stator = SpindexerMotor.getStatorCurrent().getValueAsDouble();
    inputs.supply = SpindexerMotor.getSupplyCurrent().getValueAsDouble();
  }
}
