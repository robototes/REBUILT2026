package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
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
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Hardware;
import frc.robot.generated.CompTunerConstants;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import frc.robot.subsystems.launcher.LaunchCalculator.LaunchingParameters;
import frc.robot.util.robotType.RobotType;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class TurretSubsystem extends SubsystemBase {
  private final TalonFX turretMotor;
  private final PositionVoltage request = new PositionVoltage(0);
  private final AnalogInput limitSwitch;

  // Dedicated VoltageOut for SysId — does NOT ignore soft limits during characterization.
  // If you want SysId to respect soft limits (safer), use this. If you need full range,
  // swap to voltageRequest (the one with withIgnoreSoftwareLimits(true)).
  private final VoltageOut sysIdVoltageRequest = new VoltageOut(0);
  private final SysIdRoutine sysIdRoutine;

  private final CommandSwerveDrivetrain driveTrain;

  public static final double TURRET_MANUAL_SPEED = 3; // Volts

  // The tolerance is this high because the turret position is always updating so it is not
  // always exactly where it should be, I am mainly using this to stop shooting when the
  // turret hits its wraparound point
  public static final double TURRET_DEGREE_TOLERANCE = 10;
  private static final double HALL_EFFECT_THRESHOLD_VOLTS = 0.5;

  // Positions
  private double targetPos;
  public static final double FRONT_POSITION = 0;
  public static final double LEFT_POSITION = -0.15;
  public static final double RIGHT_POSITION = 0.15;
  public static final double BACK_POSITION = 0.5;

  // PID variables
  private static final double kP = RobotType.isAlpha() ? 25 : 200;
  private static final double kI = 0;
  private static final double kD = RobotType.isAlpha() ? 0 : 2;
  private static final double kG = 0;
  private static final double kS = RobotType.isAlpha() ? 0.41 : 0.65;
  private static final double kV =
      ((8.57 / 1.511) + (5.63 / 0.898438)) / 2; // volts per requested rps
  private static final double kA = 0;

  // Current limits
  private static final int STATOR_CURRENT_LIMIT = 40; // amps
  private static final int SUPPLY_CURRENT_LIMIT = 40; // amps

  // Gear Ratio
  private static final double GEAR_RATIO = RobotType.isAlpha() ? 24 : 40;

  // Soft Limits
  public static final double TURRET_MAX = RobotType.isAlpha() ? 190 : 350; // degrees
  public static final double TURRET_MIN = RobotType.isAlpha() ? 0 : -90; // degrees

  private final BooleanPublisher zeroPublisher =
      NetworkTableInstance.getDefault().getBooleanTopic("/Zero/turretZero").publish();

  StructArrayPublisher<Pose2d> turretRotation =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("lines/turretRotation", Pose2d.struct)
          .publish();
  private final DoublePublisher turretToHubDistance =
      NetworkTableInstance.getDefault()
          .getTable("/SmartDashboard/LiveLauncherData")
          .getDoubleTopic("Turret to hub distance")
          .publish();
  private final DoublePublisher turretToHubDistanceCompensated =
      NetworkTableInstance.getDefault()
          .getTable("/SmartDashboard/LiveLauncherData")
          .getDoubleTopic("Turret to hub compensated")
          .publish();

  // Network tables
  private final DoublePublisher posPub;
  private final DoublePublisher targetPub;
  private final DoublePublisher velocityPub;
  private final DoublePublisher currentPub;
  private final DoublePublisher ffPub;
  private final DoublePublisher limitSwitchPub;

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
    limitSwitch = new AnalogInput(Hardware.HALL_EFFECT_SENSOR_ID);
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
    limitSwitchPub = table.getDoubleTopic("/Turret/LimitSwitchCurrent").publish();

    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, // Default ramp rate: 1 V/s
                Volts.of(4), // Dynamic step voltage: 4 V (safe for 40 A stator limit)
                null, // Default timeout: 10 s
                // Log the SysId state so the hoot log can be correctly parsed
                (state) -> SignalLogger.writeString("sysid-state", state.toString())),
            new SysIdRoutine.Mechanism(
                // Apply voltage to the turret motor
                (volts) -> turretMotor.setControl(sysIdVoltageRequest.withOutput(volts.in(Volts))),
                // Phoenix logs position/velocity/voltage automatically — leave null
                null,
                this));
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
            })
        .withName("Set Turret Position");
  }

  public void setTurretRawPosition(double pos, double FFVelocity) {
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

  public void zeroTurretPosistion() {
    turretMotor.setPosition(0);
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

          double degrees = Math.toDegrees(Math.atan2(y, x));
          degrees -= 90.0;
          degrees -= driveTrain.getState().Pose.getRotation().getDegrees();
          degrees += 180.0;
          degrees = MathUtil.inputModulus(degrees, TURRET_MIN, TURRET_MAX);
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

  public boolean atTarget(DoubleSupplier turretTolerance) {
    return Math.abs(turretMotor.getPosition().getValueAsDouble() - targetPos)
        < Units.radiansToRotations(turretTolerance.getAsDouble());
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

              setTurretRawPosition(Units.degreesToRotations(finalTarget));
              targetPos = Units.degreesToRotations(finalTarget);

              // Network tables
              turretToHubDistance.set(params.currentDist());
              turretToHubDistanceCompensated.set(params.trueDist());
            },
            () -> turretMotor.stopMotor())
        .withName("Set Turret Position: SOTM calculation");
  }

  @Override
  public void periodic() {
    StatusSignal.refreshAll(positionSignal, velocitySignal, statorCurrentSignal);
    posPub.set(positionSignal.getValueAsDouble());
    velocityPub.set(velocitySignal.getValueAsDouble());
    currentPub.set(statorCurrentSignal.getValueAsDouble());
    targetPub.set(targetPos);
    limitSwitchPub.set(limitSwitch.getVoltage());
  }

  public void brakeTurret() {
    turretMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void coastTurret() {
    turretMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  public boolean atLimitSwitch() {
    double velo = velocitySignal.getValueAsDouble();
    return limitSwitch.getVoltage() < HALL_EFFECT_THRESHOLD_VOLTS && velo < -0.01 && velo > -0.5;
  }

  // ------ SYSID COMMANDS ------ //

  /**
   * Quasistatic SysId test — slowly ramps voltage to characterize kS and kV. Run forward then
   * reverse. Start SignalLogger before calling these.
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction).withName("Turret SysId Quasistatic " + direction);
  }

  /**
   * Dynamic SysId test — applies a voltage step to characterize kA. Run forward then reverse. Start
   * SignalLogger before calling these.
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction).withName("Turret SysId Dynamic " + direction);
  }
}
