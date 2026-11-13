package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;

public class IntakePivotIOTalonFX implements PivotIO {
  private TalonFX pivotMotor;
  private final StatusSignal<Current> intakePivotStatorCurrent;
  private final StatusSignal<Current> intakePivotSupplyCurrent;
  private final StatusSignal<Angle> position;
  private final StatusSignal<Temperature> motorTemp;

  public IntakePivotIOTalonFX(TalonFX pivotMotors) {
    pivotMotor = pivotMotors;
    intakePivotStatorCurrent = pivotMotor.getStatorCurrent();
    position = pivotMotor.getPosition();
    intakePivotSupplyCurrent = pivotMotor.getSupplyCurrent();
    motorTemp = pivotMotor.getDeviceTemp();
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    StatusSignal.refreshAll(position, intakePivotStatorCurrent, intakePivotSupplyCurrent);
    inputs.currentPos = position.getValueAsDouble();
    inputs.isConnected = pivotMotor.isConnected();
    inputs.motorTemp = motorTemp.getValueAsDouble();
    inputs.statorCurrentPivot = intakePivotStatorCurrent.getValueAsDouble();
    inputs.supplyCurrentPivot = intakePivotSupplyCurrent.getValueAsDouble();
  }
}
