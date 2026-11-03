package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

public class IntakePivotIOTalonFX implements PivotIO {
  private TalonFX pivotMotor;

  public IntakePivotIOTalonFX(TalonFX pivotMotors) {
    pivotMotor = pivotMotors;
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    StatusSignal.refreshAll(
        inputs.currentPos,
        inputs.targetPos,
        inputs.isConnected,
        inputs.motorTemp,
        inputs.statorCurrentPivot,
        inputs.supplyCurrentPivot);
  }
}
