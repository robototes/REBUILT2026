package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import frc.robot.util.TurretSubsystemSim;
import frc.robot.util.TurretUtils.TurretState;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class TurretSubsystem2 extends SubsystemBase {
  private MotionMagicVoltage request;
  private CommandSwerveDrivetrain driveTrain;
  private TalonFX m_turret;

  private final double GEAR_RATIO = 20;
  private final double TURRET_MIN = 0;
  private final double TURRET_MAX = 180;
  private final VoltageOut FIVE_VOLTS = new VoltageOut(5);
  private final VoltageOut ZERO_VOLTS = new VoltageOut(0);

  private final double k_S = 0.41;
  private final double k_V = 0.9;
  private final double k_A = 0.12;

  private final double k_P = 2.97;
  private final double k_I = 0;
  private final double k_D = 1.5;

  TurretSubsystemSim sim;

  public TurretSubsystem2(CommandSwerveDrivetrain train) {
    request = new MotionMagicVoltage(0);
    driveTrain = train;
    m_turret = new TalonFX(24);
    sim = new TurretSubsystemSim(m_turret, GEAR_RATIO, -Math.PI / 2, Math.PI / 2);
    configureMotors();
  }

  // public Command autoRotate()

  public Command ManualMove(DoubleSupplier joystick) {
    return Commands.run(
        () -> {
          double cmd = MathUtil.applyDeadband(joystick.getAsDouble(), 0.10);
          double volts = cmd * 2.0;
          m_turret.setControl(new VoltageOut(volts));
        });
  }

  public Command PositionMove(DoubleSupplier joystick) {
    return Commands.run(
        () -> {
          double multiplier = MathUtil.applyDeadband(joystick.getAsDouble(), 0.1);
          double position = multiplier * 0.5;
          setMotorPosition(position);
        });
  }

  private void setMotorPosition(double rotations) {
    m_turret.setControl(request.withPosition(rotations));
  }

  private double wrapDegreesToSoftLimits(double targetDegrees, double currentDegrees) {
    return MathUtil.clamp(
        currentDegrees + ((targetDegrees - currentDegrees + 540) % 360) - 180,
        TURRET_MIN,
        TURRET_MAX);
  }

  public Trigger ManualRotateTrigger(Supplier<TurretState> state) {
    return new Trigger(
        () -> {
          System.out.println(state.get());
          return state.get() == TurretState.MANUAL;
        });
  }

  public Trigger PositionRotateTrigger(Supplier<TurretState> state) {
    return new Trigger(
        () -> {
          return state.get().equals(TurretState.POSITION);
        });
  }

  public Trigger AutoRotateTrigger(Supplier<TurretState> state) {
    return new Trigger(
        () -> {
          return state.get() == TurretState.AUTO;
        });
  }

  private class Autorotate extends Command {
    public Autorotate() {}
  }

  private void configureMotors() {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    Slot0Configs slot0 = configs.Slot0;

    slot0.kP = k_P;
    slot0.kI = k_I;
    slot0.kD = k_D;

    slot0.kS = k_S;
    slot0.kV = k_V;
    slot0.kA = k_A;

    // ---CURRENT LIMITS --- //
    configs.CurrentLimits.StatorCurrentLimit = 25;
    configs.CurrentLimits.SupplyCurrentLimit = 10;
    configs.CurrentLimits.StatorCurrentLimitEnable = true;
    configs.CurrentLimits.SupplyCurrentLimitEnable = true;

    // -----SOFT LIMITS ----- //
    // configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.radiansToRotations(TURRET_MAX);
    // configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.radiansToRotations(TURRET_MIN);
    // configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    // configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    // ---- MOTION MAGIC---- //
    var mm = configs.MotionMagic;
    mm.MotionMagicCruiseVelocity = 5;
    mm.MotionMagicAcceleration = 20;
    mm.MotionMagicJerk = 300;

    // --- FEEDBACK / OUTPUT --- //
    configs.Feedback.SensorToMechanismRatio = GEAR_RATIO;
    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    m_turret.getConfigurator().apply(configs);
  }

  @Override
  public void simulationPeriodic() {
    if (sim != null) {
      sim.update(0.02);
    }
  }
}
