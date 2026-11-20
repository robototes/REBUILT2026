package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.AnalogInput;
import java.util.Objects;

public class TurretIOTalonFX implements TurretIO {

  private TalonFX turretMotor;
  private AnalogInput limitSwitch;
  private final StatusSignal<Angle> positionRotations;
  private final StatusSignal<AngularVelocity> velocityRPS;
  private final StatusSignal<Current> statorCurrentAmps;
  private final StatusSignal<Temperature> motorTemp;

  public TurretIOTalonFX(TalonFX motor, AnalogInput lSwitch) {
    this.turretMotor = Objects.requireNonNull(motor, "Turret motor cannot be null");
    this.limitSwitch = Objects.requireNonNull(lSwitch, "Turret limit switch cannot be null");
    this.positionRotations = turretMotor.getPosition();
    this.velocityRPS = turretMotor.getVelocity();
    this.statorCurrentAmps = turretMotor.getStatorCurrent();
    this.motorTemp = turretMotor.getDeviceTemp();
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    BaseStatusSignal.refreshAll(positionRotations, velocityRPS, statorCurrentAmps);
    inputs.positionRotations = positionRotations.getValueAsDouble();
    inputs.velocityRPS = velocityRPS.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrentAmps.getValueAsDouble();
    inputs.limitSwitchVolts = limitSwitch.getVoltage();
    inputs.motorTemp = motorTemp.getValueAsDouble();
  }
}
