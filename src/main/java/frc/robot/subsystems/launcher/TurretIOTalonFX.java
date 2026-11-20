package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.AnalogInput;

import static edu.wpi.first.units.Units.Hertz;

import java.util.Objects;

public class TurretIOTalonFX implements TurretIO {

  private final TalonFX turretMotor;
  private final AnalogInput limitSwitch;
  private final StatusSignal<Angle> positionRotations;
  private final StatusSignal<AngularVelocity> velocityRPS;
  private final StatusSignal<Current> statorCurrentAmps;
  private final StatusSignal<Temperature> motorTemp;

  private final BaseStatusSignal[] signals;

  public TurretIOTalonFX(TalonFX motor, AnalogInput lSwitch) {
    this.turretMotor = Objects.requireNonNull(motor, "Turret motor cannot be null");
    this.limitSwitch = Objects.requireNonNull(lSwitch, "Turret limit switch cannot be null");
    this.positionRotations = turretMotor.getPosition();
    this.velocityRPS = turretMotor.getVelocity();
    this.statorCurrentAmps = turretMotor.getStatorCurrent();
    this.motorTemp = turretMotor.getDeviceTemp();
    BaseStatusSignal.setUpdateFrequencyForAll(Frequency.ofBaseUnits(50, Hertz), positionRotations, velocityRPS, statorCurrentAmps);
    BaseStatusSignal.setUpdateFrequencyForAll(Frequency.ofBaseUnits(4, Hertz), motorTemp);
    signals = new BaseStatusSignal[] {
      positionRotations, velocityRPS, statorCurrentAmps, motorTemp
    };
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    BaseStatusSignal.refreshAll(signals);
    inputs.positionRotations = positionRotations.getValueAsDouble();
    inputs.velocityRPS = velocityRPS.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrentAmps.getValueAsDouble();
    inputs.limitSwitchVolts = limitSwitch.getVoltage();
    inputs.motorTemp = motorTemp.getValueAsDouble();
    inputs.isConnected = BaseStatusSignal.isAllGood(signals);
  }
}
