package frc.robot.subsystems.index;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Temperature;

public class FeederIOTalonFX implements FeederIO {
  private TalonFX feedMotor;
  private final StatusSignal<AngularVelocity> feedRPS;
  private final StatusSignal<Current> statorCurrentAmps;
  private final StatusSignal<Current> supplyCurrentAmps;
  private final StatusSignal<Temperature> motorTemp;

  private final BaseStatusSignal[] signals;

  public FeederIOTalonFX(TalonFX feederMotor) {
    feedMotor = feederMotor;
    feedRPS = feedMotor.getVelocity();
    statorCurrentAmps = feedMotor.getStatorCurrent();
    supplyCurrentAmps = feedMotor.getSupplyCurrent();
    motorTemp = feedMotor.getDeviceTemp();

    signals = new BaseStatusSignal[] {feedRPS, statorCurrentAmps, supplyCurrentAmps, motorTemp};
    BaseStatusSignal.setUpdateFrequencyForAll(
        Frequency.ofBaseUnits(40, Hertz), feedRPS, statorCurrentAmps, supplyCurrentAmps);
    BaseStatusSignal.setUpdateFrequencyForAll(Frequency.ofBaseUnits(4, Hertz), motorTemp);
    feederMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    StatusSignal.refreshAll(signals);
    inputs.feederRPS = feedRPS.getValueAsDouble();
    inputs.statorCurrent = statorCurrentAmps.getValueAsDouble();
    inputs.supplyCurrent = supplyCurrentAmps.getValueAsDouble();
    inputs.motorTemp = motorTemp.getValueAsDouble();
    inputs.isConnected = BaseStatusSignal.isAllGood(signals);
  }
}
