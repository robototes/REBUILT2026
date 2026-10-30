package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;

public class IntakePivotIOTalonFX implements PivotIO {
  private TalonFX pivotMotor;

  public IntakePivotIOTalonFX(TalonFX pivotMotors) {
    pivotMotor = pivotMotors;
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.currentPos = pivotMotor.getPosition().getValueAsDouble();
    inputs.statorCurrentPivot = pivotMotor.getStatorCurrent().getValueAsDouble();
    inputs.supplyCurrentPivot = pivotMotor.getSupplyCurrent().getValueAsDouble();
    inputs.isConnected = pivotMotor.isConnected();
    inputs.motorTemp = pivotMotor.getDeviceTemp().getValueAsDouble();
  }
}
