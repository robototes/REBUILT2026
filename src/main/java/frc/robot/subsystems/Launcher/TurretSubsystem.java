package frc.robot.subsystems.Launcher;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import frc.robot.util.AllianceUtils;
import java.util.function.Supplier;

public class TurretSubsystem extends SubsystemBase {
  private final TalonFX turretMotor;
  private final MotionMagicVoltage request = new MotionMagicVoltage(0);
  private final CommandSwerveDrivetrain driveTrain;

  public static final double TURRET_MANUAL_SPEED = 3; // Volts

  // Offsets
  private static final double TURRET_X_OFFSET = 0.2159;
  private static final double TURRET_Y_OFFSET = 0.1397;
  private static final Transform2d TURRET_OFFSET =
      new Transform2d(TURRET_X_OFFSET, -TURRET_Y_OFFSET, Rotation2d.k180deg);

  // Positions
  private double targetPos;
  public static final double FRONT_POSITION = 0.16748;
  public static final double LEFT_POSITION = 0.3122;
  public static final double RIGHT_POSITION = 0;
  public static final double BACK_POSITION = 0.5;

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
  private static final double TURRET_MAX = 170; // degrees
  private static final double TURRET_MIN = 0; // degrees
  private static final double TURRET_DEADBAND = -0.5; // degrees

  public TurretSubsystem(CommandSwerveDrivetrain driveTrain) {
    this.driveTrain = driveTrain;
    turretMotor = new TalonFX(Hardware.TURRET_MOTOR_ID);
    turretConfig();
    turretMotor.setPosition(0);
  }

  public void turretConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

    config.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.degreesToRotations(TURRET_MAX);
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.degreesToRotations(TURRET_MIN);
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

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
          turretMotor.setPosition(Units.degreesToRotations(0));
          targetPos = Units.degreesToRotations(0);
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

          // Joystick angle: 0° = forward, CCW positive
          double degrees = Math.toDegrees(Math.atan2(y, x));

          // Rotate so 0° = robot forward
          degrees -= 90.0;

          // Subtract robot angle
          degrees -= driveTrain.getState().Pose.getRotation().getDegrees();

          // Shift so 0° = backward
          degrees += 180.0;

          // Normalize to [0, 360)
          degrees = (degrees % 360 + 360) % 360;

          // Clamp to turret range
          degrees = MathUtil.clamp(degrees, 0, 170);

          double rotations = Units.degreesToRotations(degrees);

          turretMotor.setControl(request.withPosition(rotations));
          targetPos = rotations;
        });
  }

  public double getTurretPosition() {
    return turretMotor.getPosition().getValueAsDouble();
  }

  public boolean atTarget() {
    return Math.abs(turretMotor.getPosition().getValueAsDouble() - targetPos)
        < Units.degreesToRotations(2);
  }

  private double calculateTurretAngle() {
    // Get current robot pose
    Pose2d turretPose = driveTrain.getState().Pose.plus(TURRET_OFFSET);
    Translation2d robotTranslation = turretPose.getTranslation();
    Rotation2d robotRotation = turretPose.getRotation();

    // Get hub position
    Translation2d hubTranslation = AllianceUtils.getHubTranslation2d();

    // Calculate vector from robot to hub
    Translation2d robotToHub = hubTranslation.minus(robotTranslation);

    // Calculate absolute field angle to hub
    Rotation2d absoluteAngleToHub =
        new Rotation2d(Math.atan2(robotToHub.getY(), robotToHub.getX()));

    // Calculate turret angle relative to robot's forward direction
    // Subtract robot's rotation to get robot-relative angle
    Rotation2d turretAngle = absoluteAngleToHub.minus(robotRotation);

    // Convert to degrees
    double degrees = turretAngle.getDegrees();

    // Convert to clockwise positive
    degrees = -degrees;

    // Normalize to [0, 360)
    degrees = (degrees % 360 + 360) % 360;

    // Clamp to turret limits
    degrees = MathUtil.clamp(degrees, 0, 170);

    // Convert to rotations
    double rotations = Units.degreesToRotations(degrees);

    return rotations;
  }

  public Command rotateToHub() {
    return runEnd(
        () -> {
          double targetRotations = calculateTurretAngle();
          turretMotor.setControl(request.withPosition(targetRotations));
          targetPos = targetRotations;
        },
        () -> turretMotor.stopMotor());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret/Position", turretMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Turret/Target", targetPos);
    SmartDashboard.putNumber("Turret/Velocity", turretMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Turret/Current", turretMotor.getStatorCurrent().getValueAsDouble());
  }
}
