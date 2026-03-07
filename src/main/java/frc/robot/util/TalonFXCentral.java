package frc.robot.util;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.DoubleSupplier;

public class TalonFXCentral {
  private TalonFX motor;
  private MotionMagicVoltage request = new MotionMagicVoltage(0);
  private TalonFXConfiguration config = new TalonFXConfiguration();
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private Translation2d offSet;
  private int MOTOR_ID;
  private Subsystem sub;

  public record PIDConstants(
      double P, double I, double D, double G, double S, double V, double A) {}

  public record TFCMotionMagicConfig(double cruiseVelocity, double acceleration, double jerk) {}

  public record TFCCurrentLimits(
      boolean supplyEnabled, boolean statorEnabled, double statorLimit, double supplyLimit) {}

  public record TFCsoftLimits(boolean enabled, double min, double max) {}

  public record TFCmotorConfigs(
      int MOTOR_ID,
      PIDConstants pid,
      TFCMotionMagicConfig motionMagicConfig,
      TFCsoftLimits softLimits,
      TFCCurrentLimits currentLimits,
      Translation2d offSet,
      double GEAR_RATIO,
      Subsystem motorSubsystem) {}

  public TalonFXCentral(TFCmotorConfigs params) {
    this.offSet = params.offSet();
    MOTOR_ID = params.MOTOR_ID();
    motor = new TalonFX(MOTOR_ID);
    configureMotor(
        params.pid(), params.motionMagicConfig(), params.currentLimits(), params.softLimits());
    sub = params.motorSubsystem();
  }

  public Command moveToPoint(double rotations) {
    return Commands.run(
        () -> {
          motor.setControl(request.withPosition(rotations));
        },
        sub);
  }

  public Command joystickMove(DoubleSupplier x, double maxVolts) {
    return Commands.runEnd(
        () -> {
          double volts =
              MathUtil.clamp(
                  MathUtil.applyDeadband(x.getAsDouble(), 0.1) * maxVolts, -maxVolts, maxVolts);
          applyMotorVoltage(volts);
        },
        () -> {
          applyMotorVoltage(0);
        },
        sub);
  }

  public void applyMotorVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  public int getMotorID() {
    return MOTOR_ID;
  }

  public TalonFX getMotor() {
    return motor;
  }

  public void configureMotor(
      PIDConstants c, TFCMotionMagicConfig m, TFCCurrentLimits l, TFCsoftLimits s) {
    Slot0Configs slot0 = config.Slot0;
    slot0.kP = c.P();
    slot0.kI = c.I();
    slot0.kD = c.D();
    slot0.kG = c.G();
    slot0.kS = c.S();
    slot0.kV = c.V();
    slot0.kA = c.A();

    config.CurrentLimits.StatorCurrentLimit = l.statorLimit();
    config.CurrentLimits.SupplyCurrentLimit = l.supplyLimit();
    config.CurrentLimits.StatorCurrentLimitEnable = l.statorEnabled();
    config.CurrentLimits.SupplyCurrentLimitEnable = l.supplyEnabled();

    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.degreesToRotations(s.max());
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.degreesToRotations(s.min());
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = s.enabled();
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = s.enabled();

    config.MotionMagic.MotionMagicCruiseVelocity = m.cruiseVelocity();
    config.MotionMagic.MotionMagicAcceleration = m.acceleration();
    config.MotionMagic.MotionMagicJerk = m.jerk();

    motor.getConfigurator().apply(config);
  }
}
