package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;

public class HoodIOTalonFX implements HoodIO {
  private TalonFX hoodMotor;
  private final StatusSignal<Current> hoodMotorStatorCurrent;
  private final StatusSignal<Current> hoodMotorSupplyCurrent;
  private final StatusSignal<Angle> hoodMotorPosition;
  private final StatusSignal<Temperature> hoodMotorTemp;

  public HoodIOTalonFX(TalonFX hoodMotors) {
    hoodMotor = hoodMotors;
    hoodMotorStatorCurrent = hoodMotors.getStatorCurrent();
    hoodMotorPosition = hoodMotors.getPosition();
    hoodMotorSupplyCurrent = hoodMotors.getSupplyCurrent();
    hoodMotorTemp = hoodMotors.getDeviceTemp();
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    StatusSignal.refreshAll(hoodMotorPosition, hoodMotorStatorCurrent, hoodMotorSupplyCurrent);
    inputs.position = hoodMotorPosition.getValueAsDouble();
    inputs.hoodStatorCurrent = hoodMotorStatorCurrent.getValueAsDouble();
    inputs.hoodSupplyCurrent = hoodMotorSupplyCurrent.getValueAsDouble();
    inputs.temp = hoodMotorTemp.getValueAsDouble();
  }
}
