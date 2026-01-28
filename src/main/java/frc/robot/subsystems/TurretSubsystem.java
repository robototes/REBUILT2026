package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;
import frc.robot.util.AllianceUtils;
import java.util.function.DoubleSupplier;

public class TurretSubsystem extends SubsystemBase {
  // ------ VARIABLES ------//
  private final TalonFX m_turretMotor;
  private MotionMagicVoltage request;

  private final int GEAR_RATIO = 9000; // It's over 9000
  private final double TURRET_MIN = -Math.PI / 2; // In radians
  private final double TURRET_MAX = Math.PI / 2; // In radians

  private final double TURRET_X_OFFSET = 0.1; // METERS
  private final double TURRET_Y_OFFSET = 0.1; // METERS

  private final double STALL_CURRENT = 30; // Amps
  // ------- AUTOZERO ------ //
  private final double MIN_VELOCITY = 0.3;
  private final double MIN_HITS = 10;

  // ----- HARDWARE OBJECTS ----- //
  private final SwerveDrivetrain m_driveTrain;
  private final Translation2d hub;
  private final Transform2d turretTransform;

  // --- STATES ---- //
  public static enum turretState {
    MANUAL,
    AUTO,
    IDLE
  }

  // NETWORKTABLES
  NetworkTableInstance inst;
  NetworkTable TurretNetworkTable;
  final DoublePublisher turretRotation;
  final DoublePublisher turretRotationFieldRelative;
  final DoublePublisher robotRotation;
  final DoublePublisher errorRad;
  final DoublePublisher targetRad;
  final DoublePublisher current;

  // PLACEHOLDER VALUE. This will probably be handled elsewhere
  private boolean readyToShoot = false;
  private static turretState currentState = turretState.IDLE;

  // --- CONSTRUCTOR --- //
  public TurretSubsystem(SwerveDrivetrain drivetrain) {
    // --- MOTOR SETUP ---//
    m_turretMotor = new TalonFX(Hardware.TURRET_MOTOR_ID);
    request = new MotionMagicVoltage(0);
    configureMotors();
    // --- Hardware --- //
    hub = AllianceUtils.getHubTranslation2d();
    m_driveTrain = drivetrain;
    turretTransform =
        new Transform2d(new Translation2d(TURRET_X_OFFSET, TURRET_Y_OFFSET), Rotation2d.kZero);
    // --- NETWORK TABLES --- //
    this.inst = NetworkTableInstance.getDefault();
    TurretNetworkTable = inst.getTable("Turret Subsystem");
    turretRotation = TurretNetworkTable.getDoubleTopic("Turret Rotation").publish();
    turretRotationFieldRelative =
        TurretNetworkTable.getDoubleTopic("Turret Rotation field relative").publish();
    robotRotation = TurretNetworkTable.getDoubleTopic("Robot rotation").publish();
    errorRad = TurretNetworkTable.getDoubleTopic("errorDeg").publish();
    targetRad = TurretNetworkTable.getDoubleTopic("Target Degrees").publish();
    current = TurretNetworkTable.getDoubleTopic("Current").publish();
  }

  private void moveMotor(double targetDegrees) {
    m_turretMotor.setControl(request.withPosition(Units.radiansToRotations(targetDegrees)));
  }

  private double calculateTargetRadians() {
    Pose2d turretPose = m_driveTrain.getState().Pose.transformBy(turretTransform);
    double robotRotation = turretPose.getRotation().getRadians(); // Robot Rotation in radians
    double TurretRotation =
        Units.rotationsToRadians(
            m_turretMotor.getPosition().getValueAsDouble()); // Turret rotation in radians
    double TurretRotationFieldRelative =
        MathUtil.angleModulus(
            robotRotation
                + TurretRotation); // Get the -pi -> pi equivalent of the field relative angle of
    // the turret

    Translation2d difference =
        hub.minus(turretPose.getTranslation()); // get X and Y distance from the turret to the hub
    double requiredAngles = Math.atan2(difference.getY(), difference.getX()); // use

    // Results
    double errorRadians = MathUtil.angleModulus(requiredAngles - TurretRotationFieldRelative);
    double turretRadians =
        MathUtil.clamp(
            MathUtil.angleModulus(requiredAngles - robotRotation), TURRET_MIN, TURRET_MAX);

    // NETWORK TABLES
    this.errorRad.set(errorRadians);
    this.targetRad.set(turretRadians);
    this.robotRotation.set(robotRotation);
    this.turretRotation.set(TurretRotation);
    this.turretRotationFieldRelative.set(Units.radiansToDegrees(TurretRotationFieldRelative));
    return turretRadians;
  }

  // ----- PUBLIC METHODS ----- //

  public Command autoZeroCommand(boolean runAutoZeroRoutine) {
    final int[] hits = {0};
    if (runAutoZeroRoutine) {
      return Commands.parallel(
          Commands.run(
                  () -> {
                    m_turretMotor.setControl(new VoltageOut(0.5));
                    if (m_turretMotor.getStatorCurrent().getValueAsDouble() >= STALL_CURRENT) {
                      hits[0]++;
                    } else {
                      hits[0] = 0;
                    }
                  },
                  TurretSubsystem.this)
              .until(
                  () -> {
                    double motorVelocity = m_turretMotor.getVelocity().getValueAsDouble();
                    return hits[0] > MIN_HITS && motorVelocity <= MIN_VELOCITY;
                  }));
    } else {
      zeroMotor();
      return Commands.none();
    }
  }

  private void zeroMotor() {
    m_turretMotor.setPosition(Units.degreesToRotations(TURRET_MAX + 0.5));
    readyToShoot = true;
  }

  public Command turretControlCommand(DoubleSupplier xJoystick) {
    return run(() -> {
          if (!readyToShoot) {
            stop();
            return;
          }
          switch (TurretSubsystem.currentState) {
            case AUTO:
              moveMotor(calculateTargetRadians());
              break;
            case MANUAL:
              manualMove(xJoystick);
              break;
            case IDLE:
            default:
              stop();
              break;
          }
        })
        .finallyDo(interrupted -> stop());
  }

  public void manualMove(DoubleSupplier joystick) {
    double rad = Units.rotationsToRadians(m_turretMotor.getPosition().getValueAsDouble());
    double cmd = MathUtil.applyDeadband(joystick.getAsDouble(), 0.10);
    double volts = cmd * 2.0;

    if (rad >= TURRET_MAX && volts > 0) {
      stop();
      return;
    } // trying to go further +
    if (rad <= TURRET_MIN && volts < 0) {
      stop();
      return;
    } // trying to go further -
    m_turretMotor.setControl(new VoltageOut(volts));
  }

  public void stop() {
    m_turretMotor.setControl(new VoltageOut(0));
  }

  // --- STATE SETTERS --- //
  public static void IDLE() {
    currentState = turretState.IDLE;
  }

  public static void AUTO() {
    currentState = turretState.AUTO;
  }

  public static void MANUAL() {
    currentState = turretState.MANUAL;
  }

  // --- MOTOR CONFIGS --- //
  private void configureMotors() {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    Slot0Configs slot1 = configs.Slot0;

    slot1.kS = 0.25; // Required voltage to overcome static friction.
    slot1.kV = 2; // 2 volts to maintain 1 rps
    slot1.kA = 4; // 4 volts required to maintain 1 rps/s
    // -- PID -- //
    slot1.kP = 3; // ERROR * P = Output
    slot1.kI = 0;
    slot1.kD = 0.3;
    // -- CURRENT LIMITS -- //
    configs.CurrentLimits.StatorCurrentLimitEnable = true;
    configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    configs.CurrentLimits.StatorCurrentLimit = 20; // Standard limit
    configs.CurrentLimits.SupplyCurrentLimit = 10; // Standard limit

    var MotionMagicConfig = configs.MotionMagic;
    MotionMagicConfig.MotionMagicCruiseVelocity = 1.5;
    MotionMagicConfig.MotionMagicAcceleration = 160;
    MotionMagicConfig.MotionMagicJerk = 1000;

    configs.Feedback.SensorToMechanismRatio = GEAR_RATIO;

    // set to coast
    configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    m_turretMotor.getConfigurator().apply(configs);
  }

  // PERIODIC FUNCTIONS
  @Override
  public void periodic() {
    current.set(m_turretMotor.getStatorCurrent().getValueAsDouble());
    super.periodic();
  }

  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();
  }
}
