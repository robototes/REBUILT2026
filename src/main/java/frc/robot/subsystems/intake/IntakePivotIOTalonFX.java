package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;

public class IntakePivotIOTalonFX implements PivotIO {
  private TalonFX pivotMotor;
  private final StatusSignal<Current> intakePivotStatorCurrent;
  private final StatusSignal<Current> intakePivotSupplyCurrent;
  private final StatusSignal<Angle> position;
  private final StatusSignal<Integer> isConnected;

  public IntakePivotIOTalonFX(TalonFX pivotMotors) {
    pivotMotor = pivotMotors;
    intakePivotStatorCurrent = pivotMotor.getStatorCurrent();
    position = pivotMotor.getPosition();
    intakePivotSupplyCurrent = pivotMotor.getSupplyCurrent();
    isConnected = pivotMotor.getVersion();
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    StatusSignal.refreshAll(position, intakePivotStatorCurrent, intakePivotSupplyCurrent);
    inputs.currentPos = position.getValueAsDouble();
    inputs.isConnected = isConnected.getTimestamp().getLatency() < 0.5;
    inputs.motorTemp = pivotMotor.getDeviceTemp().getValueAsDouble();
    inputs.statorCurrentPivot = intakePivotStatorCurrent.getValueAsDouble();
    inputs.supplyCurrentPivot = intakePivotSupplyCurrent.getValueAsDouble();
  }
}
