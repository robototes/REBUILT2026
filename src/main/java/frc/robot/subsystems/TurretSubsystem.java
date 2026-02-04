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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Controls.TurretState;
import frc.robot.Hardware;
import frc.robot.util.AllianceUtils;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class TurretSubsystem extends SubsystemBase {
  // ------ VARIABLES ------//
  private final TalonFX m_turretMotor;
  private MotionMagicVoltage request;

  private static final double GEAR_RATIO = 10;
  private static final double TURRET_MIN = 0; // In radians
  private static final double TURRET_MAX = Math.PI / 2; // In radians

  private static final double TURRET_X_OFFSET = 0.2159; // METERS
  private static final double TURRET_Y_OFFSET = 0.1397; // METERS

  private static final double STALL_CURRENT = 5; // Amps

  private static final VoltageOut zeroVolts = new VoltageOut(0);
  private static final VoltageOut _0_5volts = new VoltageOut(0.5);
  // ------- AUTOZERO ------ //
  private static final double MIN_VELOCITY = 0.3;
  private static final double MIN_HITS = 5;

  // ----- HARDWARE OBJECTS ----- //
  private final SwerveDrivetrain m_driveTrain;
  private final Pose2d hub;
  private final Transform2d turretTransform;

  private boolean zeroed = false;

  // NETWORKTABLES
  private final NetworkTableInstance inst;
  private final NetworkTable TurretNetworkTable;
  final DoublePublisher turretRotation;
  final DoublePublisher turretRotationFieldRelative;
  final DoublePublisher robotRotation;
  final DoublePublisher errorRad;
  final DoublePublisher targetRad;
  final DoublePublisher current;

  // --- CONSTRUCTOR --- //
  public TurretSubsystem(SwerveDrivetrain drivetrain) {
    // --- MOTOR SETUP ---//
    m_turretMotor = new TalonFX(Hardware.TURRET_MOTOR_ID);
    request = new MotionMagicVoltage(0);
    configureMotors();
    // --- Hardware --- //
    hub = new Pose2d(AllianceUtils.getHubTranslation2d(), Rotation2d.kZero);
    m_driveTrain = drivetrain;
    turretTransform = new Transform2d(TURRET_X_OFFSET, TURRET_Y_OFFSET, Rotation2d.kZero);
    // --- NETWORK TABLES --- //
    this.inst = NetworkTableInstance.getDefault();
    TurretNetworkTable = inst.getTable("Turret Subsystem");
    turretRotation = TurretNetworkTable.getDoubleTopic("Turret Rotation").publish();
    turretRotationFieldRelative =
        TurretNetworkTable.getDoubleTopic("Turret Rotation field relative").publish();
    robotRotation = TurretNetworkTable.getDoubleTopic("Robot rotation").publish();
    errorRad = TurretNetworkTable.getDoubleTopic("errorRad").publish();
    targetRad = TurretNetworkTable.getDoubleTopic("Target Degrees").publish();
    current = TurretNetworkTable.getDoubleTopic("Current").publish();
  }

  private void moveMotor(double TargetRadians) {
    m_turretMotor.setControl(request.withPosition(Units.radiansToRotations(TargetRadians)));
  }

  private double calculateTargetRadians() {
    Pose2d turretPose = m_driveTrain.getState().Pose.transformBy(turretTransform);
    double robotRotation = turretPose.getRotation().getRadians(); // Robot Rotation in radians
    double turretRotation =
        Units.rotationsToRadians(
            m_turretMotor.getPosition().getValueAsDouble()); // Turret rotation in radians
    double turretRotationFieldRelative =
        MathUtil.angleModulus(
            robotRotation
                + turretRotation); // Get the -pi -> pi equivalent of the field relative angle of
    // the turret

    Transform2d difference =
        hub.minus(turretPose); // get X and Y distance from the turret to the hub
    double requiredAngles = Math.atan2(difference.getY(), difference.getX());

    // Results
    double errorRadians = MathUtil.angleModulus(requiredAngles - turretRotationFieldRelative);
    double turretRadians =
        MathUtil.clamp(
            MathUtil.angleModulus(requiredAngles - robotRotation), TURRET_MIN, TURRET_MAX);

    // NETWORK TABLES
    this.errorRad.set(errorRadians);
    this.targetRad.set(turretRadians);
    this.robotRotation.set(robotRotation);
    this.turretRotation.set(turretRotation);
    this.turretRotationFieldRelative.set(Units.radiansToDegrees(turretRotationFieldRelative));
    return turretRadians;
  }

  private void zeroMotor() {
    m_turretMotor.setPosition(
        Units.radiansToRotations(TURRET_MAX + 0.00872665)); // +0.5 Degree of headroom
    zeroed = true;
  }

  // ----- PUBLIC METHODS ----- //

  public Command autoZeroCommand(boolean runAutoZeroRoutine) {
    final int[] hits = {0};
    if (runAutoZeroRoutine) {
      return Commands.parallel(
          Commands.run(
                  () -> {
                    m_turretMotor.setControl(_0_5volts);
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
                    System.out.println("ZERO");
                    return hits[0] > MIN_HITS && motorVelocity <= MIN_VELOCITY;
                  }));
    } else {
      System.out.println("ZERO");
      return Commands.runOnce(() -> zeroMotor());
    }
  }

  public Trigger AutoRotateTrigger(Supplier<TurretState> state) {
    return new Trigger(
        () -> {
          return state.get() == TurretState.AUTO;
        });
  }

  public Trigger ManualRotateTrigger(Supplier<TurretState> state) {
    return new Trigger(
        () -> {
          return state.get() == TurretState.MANUAL;
        });
  }

  public Command AutoRotate() {
    return Commands.run(
        () -> {
          if (zeroed) {
            moveMotor(calculateTargetRadians());
          } else {
            stop();
          }
        },
        this);
  }

  public Command manualMove(DoubleSupplier joystick) {
    if (zeroed) {
      return Commands.run(
          () -> {
            System.out.println("manual");
            double rad = Units.rotationsToRadians(m_turretMotor.getPosition().getValueAsDouble());
            double cmd = MathUtil.applyDeadband(joystick.getAsDouble(), 0.10);
            double volts = cmd * 2.0;

            if (rad >= TURRET_MAX && volts > 0) {
              stop();
            } // trying to go further +
            if (rad <= TURRET_MIN && volts < 0) {
              stop();
            } // trying to go further -
            m_turretMotor.setControl(new VoltageOut(volts));
          },
          TurretSubsystem.this);
    } else {
      return Commands.none();
    }
  }

  public void stop() {
    m_turretMotor.setControl(zeroVolts);
  }

  // --- MOTOR CONFIGS --- //
  private void configureMotors() {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    Slot0Configs slot0 = configs.Slot0;

    slot0.kS = 0.25; // Required voltage to overcome static friction.
    slot0.kV = 2; // 2 volts to maintain 1 rps
    slot0.kA = 1; // 1 volts required to maintain 1 rps/s
    // -- PID -- //
    slot0.kP = 3; // ERROR * P = Output
    slot0.kI = 0;
    slot0.kD = 0.3;
    // -- CURRENT LIMITS -- //
    configs.CurrentLimits.StatorCurrentLimitEnable = true;
    configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    configs.CurrentLimits.StatorCurrentLimit = 10; // Standard limit
    configs.CurrentLimits.SupplyCurrentLimit = 5; // Standard limit

    configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.radiansToRotations(TURRET_MAX);
    configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.radiansToRotations(TURRET_MIN);
    configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    var motionMagicConfig = configs.MotionMagic;
    motionMagicConfig.MotionMagicCruiseVelocity = 1.5;
    motionMagicConfig.MotionMagicAcceleration = 160;
    motionMagicConfig.MotionMagicJerk = 1000;

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
