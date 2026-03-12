package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;
import frc.robot.Robot;
import frc.robot.generated.CompTunerConstants;
import frc.robot.subsystems.LaunchCalculator;
import frc.robot.subsystems.LaunchCalculator.LaunchingParameters;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import frc.robot.util.GetTargetFromPose;
import frc.robot.util.robotType.RobotType;
import frc.robot.util.tuning.LauncherConstants;
import java.util.function.Supplier;

public class TurretSubsystem extends SubsystemBase {
  private final TalonFX turretMotor;
  private final MotionMagicVoltage request = new MotionMagicVoltage(0);
  private final VoltageOut voltageRequest = new VoltageOut(0).withIgnoreSoftwareLimits(true);
  private final CommandSwerveDrivetrain driveTrain;

  public static final double TURRET_MANUAL_SPEED = 3; // Volts
  private static final double AUTO_ZERO_VOLTAGE = 0.5;

  // Positions
  private double targetPos;
  public static final double FRONT_POSITION = 0;
  public static final double LEFT_POSITION = -0.15;
  public static final double RIGHT_POSITION = 0.15;
  public static final double BACK_POSITION = 0.5;

  // PID variables
  private static final double kP = RobotType.isAlpha() ? 2.97 : 1000;
  private static final double kI = 0;
  private static final double kD = RobotType.isAlpha() ? 1 : 0;
  private static final double kG = 0;
  private static final double kS = RobotType.isAlpha() ? 0.41 : 0.82;
  private static final double kV = 0.9;
  private static final double kA = 0.12;

  // Current limits
  private static final int STATOR_CURRENT_LIMIT = 40; // amps
  private static final int SUPPLY_CURRENT_LIMIT = 20; // amps

  // Motion Magic Config
  private static final double CRUISE_VELOCITY = 200;
  private static final double ACCELERATION = 600;
  private static final double JERK = 2000;

  // Gear Ratio
  private static final double GEAR_RATIO = RobotType.isAlpha() ? 24 : 72;

  // Soft Limits
  public static final double TURRET_MAX = RobotType.isAlpha() ? 190 : 270; // degrees
  public static final double TURRET_MIN = RobotType.isAlpha() ? 0 : -90; // degrees

  private final BooleanPublisher zeroPublisher =
      NetworkTableInstance.getDefault().getBooleanTopic("/Zero/turretZero").publish();

  StructArrayPublisher<Pose2d> turretRotation =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("lines/turretRotation", Pose2d.struct)
          .publish();

  public TurretSubsystem(CommandSwerveDrivetrain driveTrain) {
    this.driveTrain = driveTrain;
    turretMotor =
        new TalonFX(
            Hardware.TURRET_MOTOR_ID,
            RobotType.isAlpha() ? CANBus.roboRIO() : CompTunerConstants.kCANBus);
    zeroPublisher.set(false);
    turretConfig();
    turretRotation.set(new Pose2d[2]);
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

  public void setTurretRawPosition(double pos, Double FFVelocity) {
    if (FFVelocity != null && FFVelocity != 0) {
      double ffRotationsPerSec = Units.radiansToRotations(FFVelocity);
      double feedforwardVolts = ffRotationsPerSec * kV;

      // .withFeedForward() to actually apply the volts
      turretMotor.setControl(request.withPosition(pos).withFeedForward(feedforwardVolts));
    } else {
      // Normal position control if no velocity is provided
      turretMotor.setControl(request.withPosition(pos).withFeedForward(0));
    }
    targetPos = pos;
  }

  public Command zeroTurret() {
    return runOnce(
        () -> {
          turretMotor.setPosition(0);
          targetPos = 0;
          zeroPublisher.set(true);
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

          // Normalize to [-90, 270] (input modulus always need 360)
          degrees = MathUtil.inputModulus(degrees, TURRET_MIN, TURRET_MAX);

          // Clamp to soft limits
          degrees = MathUtil.clamp(degrees, TURRET_MIN, TURRET_MAX);

          double rotations = Units.degreesToRotations(degrees);

          turretMotor.setControl(request.withPosition(rotations));
          targetPos = rotations;
        });
  }

  public double getTurretPosition() {
    return turretMotor.getPosition().getValueAsDouble();
  }

  public boolean atTarget(double degreeTolerance) {
    return Math.abs(turretMotor.getPosition().getValueAsDouble() - targetPos)
        < Units.degreesToRotations(degreeTolerance);
  }

  public double calculateTurretAngle(Translation2d target) {
    // Get current turret pose
    Translation2d turretTranslation =
        LauncherConstants.launcherFromRobot(driveTrain.getState().Pose);

    // Add 180 degrees to account for 0 posistion
    Rotation2d turretRotation =
        driveTrain.getState().Pose.getRotation().plus(new Rotation2d(Units.degreesToRadians(-75)));

    // Get hub position
    Translation2d targetTranslation = target;

    // Calculate vector from turret to target
    Translation2d turretToTarget = targetTranslation.minus(turretTranslation);

    // Calculate absolute field angle to target
    Rotation2d absoluteAngleToTarget =
        new Rotation2d(Math.atan2(turretToTarget.getY(), turretToTarget.getX()));

    // Calculate turret angle relative to robot's forward direction
    // Subtract turret's rotation to get robot-relative angle
    Rotation2d turretAngle = absoluteAngleToTarget.minus(turretRotation);
    // Convert to degrees
    double degrees = turretAngle.getDegrees();

    // Convert to clockwise positive
    degrees = -degrees;

    // Normalize to soft limits (inout modulus always needs 360)
    degrees = MathUtil.inputModulus(degrees, -360, 0);

    // Clamp to soft limits
    degrees = MathUtil.clamp(degrees, TURRET_MIN, TURRET_MAX);

    // Convert to rotations
    double rotations = Units.degreesToRotations(degrees);

    return rotations;
  }

  public Command rotateToTarget() {
    return runEnd(
        () -> {
          double targetRotations =
              calculateTurretAngle(GetTargetFromPose.getTargetLocation(driveTrain));
          setTurretRawPosition(targetRotations, null);
          targetPos = targetRotations;
          // System.out.println("Target Rotations: " + targetRotations);
          Transform2d fieldRelativeOffset =
              new Transform2d(new Translation2d(2.0, 0.0), Rotation2d.kZero);
          Pose2d turretPose2 =
              new Pose2d(
                  LauncherConstants.launcherFromRobot(driveTrain.getState().Pose),
                  driveTrain
                      .getState()
                      .Pose
                      .getRotation()
                      .minus(Rotation2d.fromRotations(targetRotations)));
          var array2 = new Pose2d[] {turretPose2, turretPose2.plus(fieldRelativeOffset)};
          turretRotation.set(array2, 0);
        },
        () -> turretMotor.stopMotor());
  }

  public Command rotateToTargetWithCalcx() {
    return runEnd(
        () -> {
          double currentTurretDegrees = Units.rotationsToDegrees(getTurretPosition());
          LaunchingParameters para = LaunchCalculator.getInstance().getParameters(driveTrain);
          double targetTurretDegrees = para.targetTurret().getDegrees();
          targetTurretDegrees = -targetTurretDegrees;
          double shortestDelta =
              MathUtil.inputModulus(
                  targetTurretDegrees - currentTurretDegrees, TURRET_MIN, TURRET_MAX);
          double turretDegrees =
              MathUtil.clamp(currentTurretDegrees + shortestDelta, TURRET_MIN, TURRET_MAX);
          // System.out.println(Units.degreesToRotations(turretDegrees));
          setTurretRawPosition(Units.degreesToRotations(turretDegrees), null);
          targetPos = Units.degreesToRotations(turretDegrees);
        },
        () -> turretMotor.stopMotor());
  }

  // Experimental IF WITHIN BOUNDS go to it instead of clamping
  public Command rotateToTargetWithCalc() {
    return runEnd(
        () -> {

          // This represents where the turret is within a single circle (-180 to 180)
          double wrappedCurrent =
              MathUtil.inputModulus(
                  Units.rotationsToDegrees(getTurretPosition()), TURRET_MIN, TURRET_MIN + 360);

          // Get the target from the calculator
          LaunchingParameters params = LaunchCalculator.getInstance().getParameters(driveTrain);
          double targetDegrees = -params.targetTurret().getDegrees();

          double FFV = params.targetTurretFeedforward();

          // Find the shortest distance to that target from our "wrapped" position
          double shortestDelta = MathUtil.inputModulus(targetDegrees - wrappedCurrent, -180, 180);

          // This is the "ideal" target in the range closest to our current wrapped pos
          double baseTarget = wrappedCurrent + shortestDelta;

          // Check multiple rotations to see which one fits in the hardware limits
          // We check: baseTarget, baseTarget + 360, and baseTarget - 360
          double finalTarget = baseTarget;

          if (baseTarget < TURRET_MIN) {
            if (baseTarget + 360 <= TURRET_MAX) {
              finalTarget = baseTarget + 360;
            } else {
              finalTarget = MathUtil.clamp(baseTarget, TURRET_MIN, TURRET_MAX);
            }
          } else if (baseTarget > TURRET_MAX) {
            if (baseTarget - 360 >= TURRET_MIN) {
              finalTarget = baseTarget - 360;
            } else {
              finalTarget = MathUtil.clamp(baseTarget, TURRET_MIN, TURRET_MAX);
            }
          }

          // Set position (Note: This assumes your PID/Controller uses these degrees)
          // System.out.println(Units.degreesToRotations(finalTarget));
          setTurretRawPosition(Units.degreesToRotations(finalTarget), FFV);
          targetPos = Units.degreesToRotations(finalTarget);
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

  public void brakeTurret() {
    turretMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void coastTurret() {
    turretMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  public Command voltageControl(Supplier<Voltage> voltageSupplier) {
    return runEnd(
            () -> {
              turretMotor.setControl(voltageRequest.withOutput(voltageSupplier.get()));
            },
            () -> {
              turretMotor.stopMotor();
            })
        .withName("Voltage Control");
  }

  public Command autoZeroCommand() {
    if (Robot.isSimulation()) {
      return zeroTurret();
    }
    return Commands.parallel(voltageControl(() -> Volts.of(AUTO_ZERO_VOLTAGE)))
        .until(
            () -> turretMotor.getStatorCurrent().getValueAsDouble() >= (STATOR_CURRENT_LIMIT - 1))
        .andThen(zeroTurret())
        .withTimeout(3)
        .withName("Automatic Zero turret");
  }
}
