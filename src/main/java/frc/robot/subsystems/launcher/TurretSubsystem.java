package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;
import frc.robot.Robot;
import frc.robot.generated.CompTunerConstants;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import frc.robot.subsystems.launcher.LaunchCalculator.LaunchingParameters;
import frc.robot.util.robotType.RobotType;
import java.util.function.Supplier;

public class TurretSubsystem extends SubsystemBase {
  private final TalonFX turretMotor;
  private final PositionVoltage request = new PositionVoltage(0);
  private final VoltageOut voltageRequest = new VoltageOut(0).withIgnoreSoftwareLimits(true);
  private final CommandSwerveDrivetrain driveTrain;

  public static final double TURRET_MANUAL_SPEED = 3; // Volts
  private static final double AUTO_ZERO_VOLTAGE = 0.5;

  // The tolerance is this high because the turret position is always updating so it is not
  // always exactly where it should be, I am mainly using this to stop shooting when the
  // turret hits its wraparound point
  public static final double TURRET_DEGREE_TOLERANCE = 20;

  // Positions
  private double targetPos;
  public static final double FRONT_POSITION = 0;
  public static final double LEFT_POSITION = -0.15;
  public static final double RIGHT_POSITION = 0.15;
  public static final double BACK_POSITION = 0.5;

  // PID variables
  private static final double kP = RobotType.isAlpha() ? 2.97 : 150;
  private static final double kI = 0;
  private static final double kD = RobotType.isAlpha() ? 0 : 15;
  private static final double kG = 0;
  private static final double kS = RobotType.isAlpha() ? 0.41 : 0.36;
  private static final double kV = 12 / 1.29; // volts per requested rps
  private static final double kA = 0.12;

  // Current limits
  private static final int STATOR_CURRENT_LIMIT = 40; // amps
  private static final int SUPPLY_CURRENT_LIMIT = 40; // amps

  // Gear Ratio
  private static final double GEAR_RATIO = RobotType.isAlpha() ? 24 : 72;

  // Soft Limits
  public static final double TURRET_MAX = RobotType.isAlpha() ? 190 : 360; // degrees
  public static final double TURRET_MIN = RobotType.isAlpha() ? 0 : -90; // degrees

  private final BooleanPublisher zeroPublisher =
      NetworkTableInstance.getDefault().getBooleanTopic("/Zero/turretZero").publish();

  StructArrayPublisher<Pose2d> turretRotation =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("lines/turretRotation", Pose2d.struct)
          .publish();

  // Network tables

  private final DoublePublisher posPub;
  private final DoublePublisher targetPub;
  private final DoublePublisher velocityPub;
  private final DoublePublisher currentPub;
  private final DoublePublisher ffPub;

  // Status signals
  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Current> statorCurrentSignal;

  public TurretSubsystem(CommandSwerveDrivetrain driveTrain) {
    this.driveTrain = driveTrain;
    turretMotor =
        new TalonFX(
            Hardware.TURRET_MOTOR_ID,
            RobotType.isAlpha() ? CANBus.roboRIO() : CompTunerConstants.kCANBus);
    zeroPublisher.set(false);
    turretConfig();
    turretMotor.clearStickyFaults();
    turretRotation.set(new Pose2d[2]);

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("SmartDashboard");

    positionSignal = turretMotor.getPosition();
    posPub = table.getDoubleTopic("/Turret/Position").publish();

    velocitySignal = turretMotor.getVelocity();
    velocityPub = table.getDoubleTopic("/Turret/Velocity").publish();

    statorCurrentSignal = turretMotor.getStatorCurrent();
    currentPub = table.getDoubleTopic("/Turret/Current").publish();

    targetPub = table.getDoubleTopic("/Turret/Target").publish();

    ffPub = table.getDoubleTopic("/Turret/FF Volts").publish();
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
          turretMotor.setControl(request.withPosition(pos).withFeedForward(0));
          targetPos = pos;
        });
  }

  public void setTurretRawPosition(double pos, double FFVelocity) {
    // KV must be converted to volts. Right now it's only in
    // dutycycle per requested rotation per second, so multiply
    // 12 to get true voltage
    double feedforwardVolts = Units.radiansToRotations(FFVelocity) * kV;
    ffPub.set(feedforwardVolts);
    turretMotor.setControl(request.withPosition(pos).withFeedForward(feedforwardVolts));
    targetPos = pos;
  }

  public void setTurretRawPosition(double pos) {
    turretMotor.setControl(request.withPosition(pos).withFeedForward(0));
    targetPos = pos;
  }

  public Command zeroTurret() {
    return runOnce(
            () -> {
              turretMotor.setPosition(0);
              targetPos = 0;
              zeroPublisher.set(true);
            })
        .withName("zeroed turret");
  }

  public Command manualMovingVoltage(Supplier<Voltage> speed) {
    return runEnd(
            () -> turretMotor.setVoltage(speed.get().in(Volts)), () -> turretMotor.stopMotor())
        .withName("Turret Manual moving voltage command ");
  }

  public Command pointFacingJoystick(Supplier<Double> xSupplier, Supplier<Double> ySupplier) {
    return run(() -> {
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

          turretMotor.setControl(request.withPosition(rotations).withFeedForward(0));
          targetPos = rotations;
        })
        .withName("Set Turret Position: Joystick point");
  }

  public double getTurretPosition() {
    return turretMotor.getPosition().getValueAsDouble();
  }

  public boolean atTarget() {
    return Math.abs(turretMotor.getPosition().getValueAsDouble() - targetPos)
        < Units.degreesToRotations(TURRET_DEGREE_TOLERANCE);
  }

  /**
   * @return the angular velocity of the turret in rad/s
   */
  public double getOmega() {
    return Units.rotationsToRadians(velocitySignal.getValueAsDouble());
  }

  public Command rotateToTargetWithCalc() {
    return runEnd(
            () -> {
              double currentDegrees = Units.rotationsToDegrees(getTurretPosition());

              LaunchingParameters params =
                  LaunchCalculator.getInstance().getParameters(driveTrain, this);
              double targetDegrees = -params.targetTurret().getDegrees();
              double FFV = params.targetTurretFeedforward();

              double normalizedTarget =
                  MathUtil.inputModulus(targetDegrees, currentDegrees - 180, currentDegrees + 180);

              double[] candidates = {
                normalizedTarget, normalizedTarget + 360, normalizedTarget - 360,
              };

              double finalTarget = MathUtil.clamp(currentDegrees, TURRET_MIN, TURRET_MAX);
              double bestDist = Double.MAX_VALUE;

              for (double candidate : candidates) {
                if (candidate >= TURRET_MIN && candidate <= TURRET_MAX) {
                  double dist = Math.abs(candidate - currentDegrees);
                  if (dist < bestDist) {
                    bestDist = dist;
                    finalTarget = candidate;
                  }
                }
              }

              // System.out.println(Units.degreesToRotations(finalTarget));
              setTurretRawPosition(Units.degreesToRotations(finalTarget), -FFV);
              targetPos = Units.degreesToRotations(finalTarget);
            },
            () -> turretMotor.stopMotor())
        .withName("Set Turret Position: SOTM calculation");
  }

  @Override
  public void periodic() {
    StatusSignal.refreshAll(positionSignal, velocitySignal, statorCurrentSignal); // Refresh
    posPub.set(positionSignal.getValueAsDouble()); // Rotations
    velocityPub.set(velocitySignal.getValueAsDouble()); // RPS
    currentPub.set(statorCurrentSignal.getValueAsDouble()); // Amps
    targetPub.set(targetPos);
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
