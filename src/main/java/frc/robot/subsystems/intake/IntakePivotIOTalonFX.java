package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import frc.robot.util.StatusSignalRefresher;

public class IntakePivotIOTalonFX implements PivotIO {
  private final TalonFX pivotMotor;
  private final StatusSignal<Current> intakePivotStatorCurrent;
  private final StatusSignal<Current> intakePivotSupplyCurrent;
  private final StatusSignal<Angle> position;
  private final StatusSignal<Temperature> motorTemp;

  private final BaseStatusSignal[] fastSignals;

  public IntakePivotIOTalonFX(TalonFX pivotMotor) {
    this.pivotMotor = pivotMotor;
    intakePivotStatorCurrent = pivotMotor.getStatorCurrent();
    intakePivotSupplyCurrent = pivotMotor.getSupplyCurrent();
    position = pivotMotor.getPosition();
    motorTemp = pivotMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, position, intakePivotStatorCurrent, intakePivotSupplyCurrent);
    BaseStatusSignal.setUpdateFrequencyForAll(4, motorTemp);

    pivotMotor.optimizeBusUtilization();

    fastSignals =
        new BaseStatusSignal[] {position, intakePivotStatorCurrent, intakePivotSupplyCurrent};

    var refresher = StatusSignalRefresher.getInstance();
    refresher.addStatusSignals(fastSignals);
    refresher.addStatusSignals(12, motorTemp); // every 12 loops ~ 4 Hz
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.currentPos = position.getValueAsDouble();
    inputs.isConnected = BaseStatusSignal.isAllGood(fastSignals);
    inputs.motorTemp = motorTemp.getValueAsDouble();
    inputs.statorCurrentPivot = intakePivotStatorCurrent.getValueAsDouble();
    inputs.supplyCurrentPivot = intakePivotSupplyCurrent.getValueAsDouble();
  }
}
