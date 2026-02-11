package frc.robot.subsystems.Launcher;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import java.util.function.Supplier;

public class TurretSubsystem extends SubsystemBase {
  private final TalonFX turretMotor;
  private final MotionMagicVoltage request = new MotionMagicVoltage(0);
  private final CommandSwerveDrivetrain driveTrain;
  private final AutoTurretRotation autoTurretRotation;

  // Positions
  private double targetPos;
  public static final double FRONT_POSITION = Units.degreesToRotations(0);
  public static final double LEFT_POSITION = Units.degreesToRotations(-90);
  public static final double RIGHT_POSITION = Units.degreesToRotations(90);
  public static final double BACK_POSITION = Units.degreesToRotations(180);

  // PID variables
  private static final double kP = 2.97;
  private static final double kI = 0;
  private static final double kD = 1.5;
  private static final double kG = 0;
  private static final double kS = 0.41;
  private static final double kV = 0.9;
  private static final double kA = 0.12;

  // Current limits
  private static final int STATOR_CURRENT_LIMIT = 60; // amps
  private static final int SUPPLY_CURRENT_LIMIT = 40; // amps

  // Motion Magic Config
  private static final double CRUISE_VELOCITY = 5;
  private static final double ACCELERATION = 20;
  private static final double JERK = 300;

  // Gear Ratio
  private static final double GEAR_RATIO = 20;

  // Soft Limits
  private static final double TURRET_MAX = -42.1 - 180; // degrees
  private static final double TURRET_MIN = -132.1 - 180; // degrees
  private static final double TURRET_DEADBAND = -0.5; // degrees

  public TurretSubsystem(CommandSwerveDrivetrain driveTrain) {
    this.driveTrain = driveTrain;
    autoTurretRotation = new AutoTurretRotation(this, driveTrain);
    turretMotor = new TalonFX(Hardware.TURRET_MOTOR_ID);
    turretConfig();
    turretMotor.setPosition(0);
  }

  public void turretConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Feedback.SensorToMechanismRatio = 1; // Use outside of testing GEAR_RATIO;

    config.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.degreesToRadians(TURRET_MAX);
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false; // false for testing
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.degreesToRadians(TURRET_MIN);
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false; // false for testing

    config.MotionMagic.MotionMagicCruiseVelocity = CRUISE_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = ACCELERATION;
    config.MotionMagic.MotionMagicJerk = JERK;

    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    config.Slot0.kG = kG;
    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kA = kA;

    turretMotor.getConfigurator().apply(config);
  }

  public Command setTurretPosition(double pos) {
    return runOnce(
        () -> {
          turretMotor.setControl(request.withPosition(pos));
          targetPos = pos;
        });
  }

  public Command zeroTurret() {
    return runOnce(
        () -> {
          turretMotor.setPosition(0);
          targetPos =
              0; // Use Units.degreesToRotations(TURRET_MAX - TURRET_DEADBAND); outside of testing
        });
  }

  public Command manualMovingVoltage(Supplier<Voltage> speed) {
    return runEnd(
        () -> turretMotor.setVoltage(speed.get().in(Volts)), () -> turretMotor.stopMotor());
  }

  public Command pointFacingJoystick(Supplier<Double> xSupplier, Supplier<Double> ySupplier) {
    return run(
        () -> {
          double x = xSupplier.get();
          double y = ySupplier.get();

          double magnitude = Math.sqrt(x * x + y * y);
          if (magnitude > 0.1) {
            double angle = Units.radiansToRotations(Math.atan2(y, x));
            turretMotor.setControl(request.withPosition(angle));
            targetPos = angle;
          }
        });
  }

  public double getTurretPosition() {
    return turretMotor.getPosition().getValueAsDouble();
  }

  public boolean atTarget() {
    return Math.abs(turretMotor.getPosition().getValueAsDouble() - targetPos)
        < Units.degreesToRotations(2);
  }

  public Command rotateToHub() {
    return autoTurretRotation.trackHub();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret/Position", turretMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Turret/Target", targetPos);
    SmartDashboard.putNumber("Turret/Velocity", turretMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Turret/Current", turretMotor.getStatorCurrent().getValueAsDouble());
  }
}
