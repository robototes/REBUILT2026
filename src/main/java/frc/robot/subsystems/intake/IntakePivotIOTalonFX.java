package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Temperature;

public class IntakePivotIOTalonFX implements PivotIO {
  private TalonFX pivotMotor;
  private final StatusSignal<Current> intakePivotStatorCurrent;
  private final StatusSignal<Current> intakePivotSupplyCurrent;
  private final StatusSignal<Angle> position;
  private final StatusSignal<Temperature> motorTemp;

  private final BaseStatusSignal[] signals;

  public IntakePivotIOTalonFX(TalonFX pivotMotors) {
    pivotMotor = pivotMotors;
    intakePivotStatorCurrent = pivotMotor.getStatorCurrent();
    position = pivotMotor.getPosition();
    intakePivotSupplyCurrent = pivotMotor.getSupplyCurrent();
    motorTemp = pivotMotor.getDeviceTemp();
    BaseStatusSignal.setUpdateFrequencyForAll(
        Frequency.ofBaseUnits(
            40, Hertz), // Balanced frequency where signals are being reused and refreshed at proper
        // timings.
        intakePivotStatorCurrent,
        intakePivotSupplyCurrent,
        position);
    BaseStatusSignal.setUpdateFrequencyForAll(Frequency.ofBaseUnits(4, Hertz), motorTemp);
    pivotMotor.optimizeBusUtilization();
    signals =
        new BaseStatusSignal[] {
          position, intakePivotStatorCurrent, intakePivotSupplyCurrent, motorTemp
        };
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    StatusSignal.refreshAll(signals);
    inputs.currentPos = position.getValueAsDouble();
    inputs.isConnected = BaseStatusSignal.isAllGood(signals);
    inputs.motorTemp = motorTemp.getValueAsDouble();
    inputs.statorCurrentPivot = intakePivotStatorCurrent.getValueAsDouble();
    inputs.supplyCurrentPivot = intakePivotSupplyCurrent.getValueAsDouble();
  }
}
