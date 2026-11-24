package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import frc.robot.util.StatusSignalRefresher;

public class HoodIOTalonFX implements HoodIO {
  private final TalonFX hoodMotor;
  private final StatusSignal<Current> hoodMotorStatorCurrent;
  private final StatusSignal<Current> hoodMotorSupplyCurrent;
  private final StatusSignal<Angle> hoodMotorPosition;
  private final StatusSignal<Temperature> hoodMotorTemp;

  public HoodIOTalonFX(TalonFX hoodMotor) {
    this.hoodMotor = hoodMotor;
    hoodMotorStatorCurrent = hoodMotor.getStatorCurrent();
    hoodMotorSupplyCurrent = hoodMotor.getSupplyCurrent();
    hoodMotorPosition = hoodMotor.getPosition();
    hoodMotorTemp = hoodMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, hoodMotorPosition, hoodMotorStatorCurrent, hoodMotorSupplyCurrent);
    BaseStatusSignal.setUpdateFrequencyForAll(4, hoodMotorTemp);

    hoodMotor.optimizeBusUtilization();

    var refresher = StatusSignalRefresher.getInstance();
    refresher.addStatusSignals(hoodMotorPosition, hoodMotorStatorCurrent, hoodMotorSupplyCurrent);
    refresher.addStatusSignals(12, hoodMotorTemp); // ~4 Hz
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {

    inputs.position = hoodMotorPosition.getValueAsDouble();
    inputs.hoodStatorCurrent = hoodMotorStatorCurrent.getValueAsDouble();
    inputs.hoodSupplyCurrent = hoodMotorSupplyCurrent.getValueAsDouble();
    inputs.temp = hoodMotorTemp.getValueAsDouble();
  }
}
