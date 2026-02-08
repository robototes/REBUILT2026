package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
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
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Hardware;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import frc.robot.util.AllianceUtils;
import frc.robot.util.TurretUtils.TurretState;
import frc.robot.util.TurretUtils.TurretTarget;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class TurretSubsystem extends SubsystemBase {
  // ------ VARIABLES ------//
  private final TalonFX m_turretMotor;
  private MotionMagicVoltage request;

  private static final double GEAR_RATIO = 20;
  // private static final double TURRET_MAX = Units.degreesToRadians(90);
  // private static final double TURRET_MIN = Units.degreesToRadians(-90);
  private static final double TURRET_MAX = Units.degreesToRadians(-42.1);
  private static final double TURRET_MIN = Units.degreesToRadians(-132.1);

  private static final double TURRET_X_OFFSET = 0.2159; // METERS  // 0; //
  private static final double TURRET_Y_OFFSET = 0.1397; // METERS // 0; //

  private static final double STALL_CURRENT = 5; // Amps

  private final double k_S = 0.41;
  private final double k_V = 0.9;
  private final double k_A = 0.12;

  private final double k_P = 2.97;
  private final double k_I = 0;
  private final double k_D = 1.5;

  private static final VoltageOut zeroVolts = new VoltageOut(0);
  private static final VoltageOut _2_volts = new VoltageOut(2);
  // ------- AUTOZERO ------ //
  private static final double MIN_VELOCITY = 0.3;
  private static final double MIN_HITS = 5;

  // ----- HARDWARE OBJECTS ----- //
  private final CommandSwerveDrivetrain m_driveTrain;
  private final Transform2d turretTransform;

  private boolean zeroed = false;
  // --- SIMULATION --- //
  private TurretSubsystemSim m_sim;
  // NETWORKTABLES
  // ---------------- NETWORKTABLES ----------------
  private final NetworkTable m_nt;
  private final DoublePublisher ntErrorRad;
  private final DoublePublisher ntTargetRad;
  private final DoublePublisher ntMotorRad;
  private final DoublePublisher ntMotorCurrent;
  private final DoublePublisher ntRobotRotationRad;
  private final DoublePublisher ntMotorFieldRelativeRad;
  protected StructPublisher<Pose2d> turretPose2d;
  protected StructPublisher<Pose2d> goalPose2d;

  // --- CONSTRUCTOR --- //
  public TurretSubsystem(CommandSwerveDrivetrain drivetrain) {
    // --- MOTOR SETUP ---//
    m_turretMotor = new TalonFX(Hardware.TURRET_MOTOR_ID);
    request = new MotionMagicVoltage(0);
    configureMotors();
    // --- Hardware --- //
    m_driveTrain = drivetrain;
    turretTransform = new Transform2d(TURRET_X_OFFSET, TURRET_Y_OFFSET, Rotation2d.k180deg);
    // --- NETWORK TABLES --- //
    if (RobotBase.isSimulation()) {
      m_sim = new TurretSubsystemSim(m_turretMotor, GEAR_RATIO, TURRET_MIN, TURRET_MAX);
    }
    // --- NETWORK TABLES --- //
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    m_nt = inst.getTable("Turret");

    ntErrorRad = m_nt.getDoubleTopic("ErrorRadians").publish();
    ntTargetRad = m_nt.getDoubleTopic("TargetRadians").publish();
    ntMotorRad = m_nt.getDoubleTopic("MotorRadians").publish();
    ntMotorCurrent = m_nt.getDoubleTopic("MotorCurrent").publish();
    ntRobotRotationRad = m_nt.getDoubleTopic("RobotRotationRadians").publish();
    ntMotorFieldRelativeRad = m_nt.getDoubleTopic("MotorRadiansFieldRelative").publish();
    turretPose2d =
        NetworkTableInstance.getDefault().getStructTopic("Turret Pose2d", Pose2d.struct).publish();
    goalPose2d =
        NetworkTableInstance.getDefault().getStructTopic("Goal Pose2d", Pose2d.struct).publish();
  }

  // ----- AUTO ZERO ----- //

  public Command autoZeroCommand(boolean runAutoZeroRoutine) {
    final int[] hits = {0};
    if (runAutoZeroRoutine) {
      return Commands.parallel(
          Commands.run(
                  () -> {
                    m_turretMotor.setControl(_2_volts);
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
      return Commands.runOnce(() -> zeroMotor());
    }
  }

  private void zeroMotor() {
    m_turretMotor.setPosition(
        Units.radiansToRotations(-0.7348 - 0.00872665)); // +0.5 Degree of headroom
    zeroed = true;
  }

  //////
  ////// ---------- MOST IMPORTANT CLASS!!!!! ----------- ////////
  //////

  private class AutoRotate extends Command {
    private final CommandSwerveDrivetrain driveTrain;
    private final TalonFX turretMotor;
    private Pose2d targetPose;
    private final MotionMagicVoltage request;
    private double error;
    private final Supplier<TurretTarget> target;

    protected Pose2d turretPose;
    private Pose2d hub;
    private Pose2d alliance;

    private final DoublePublisher ntErrorRad;
    private final DoublePublisher ntTargetRad;

    private static final double TRACK_THRESHOLD_RAD = 0.01;

    public AutoRotate(
        CommandSwerveDrivetrain driveTrain,
        TalonFX turretMotor,
        Supplier<TurretTarget> target,
        MotionMagicVoltage request,
        DoublePublisher ntErrorRad,
        DoublePublisher ntTargetRad,
        Subsystem turretSubsystem) {
      this.target = target;
      this.driveTrain = driveTrain;
      this.turretMotor = turretMotor;
      this.request = request;
      this.ntErrorRad = ntErrorRad;
      this.ntTargetRad = ntTargetRad;
      addRequirements(turretSubsystem);
    }

    @Override
    public void initialize() {
      hub = new Pose2d(AllianceUtils.getHubTranslation2d(), Rotation2d.kZero);
      alliance = hub.transformBy(new Transform2d(-2.312797, 0, Rotation2d.k180deg));
      error = Double.POSITIVE_INFINITY;
      turretPose = driveTrain.getState().Pose;
    }

    @Override
    public void execute() {
      targetPose = (target.get() == TurretTarget.ALLIANCE) ? alliance : hub;

      // Grab poses
      Pose2d robotPose = driveTrain.getState().Pose;
      double robotYaw = robotPose.getRotation().getRadians();

      // turret angle relative to robot
      double turretRelRad = Units.rotationsToRadians(turretMotor.getPosition().getValueAsDouble());

      // Vector from robot (or turret pivot) to target in FIELD coordinates
      Translation2d toTarget =
          targetPose
              .getTranslation()
              .minus(robotPose.transformBy(turretTransform).getTranslation());

      // Direction to target in FIELD frame
      double targetFieldRad = Math.atan2(toTarget.getY(), toTarget.getX());

      // Desired turret angle in ROBOT frame (field-to-robot transform)
      double desiredRelRad = MathUtil.angleModulus(targetFieldRad - robotYaw);

      // Choose equivalent desired angle closest to current turret angle
      desiredRelRad =
          MathUtil.inputModulus(desiredRelRad, turretRelRad - Math.PI, turretRelRad + Math.PI);

      // Apply hard stops
      double cmdRelRad = MathUtil.clamp(desiredRelRad, TURRET_MIN, TURRET_MAX);

      // Error in robot frame
      error = MathUtil.angleModulus(cmdRelRad - turretRelRad);

      // Network tables stuff
      ntErrorRad.set(error);
      ntTargetRad.set(targetFieldRad);

      // If the error is greater than the specified threshold, move the motor. Else don't move
      if (Math.abs(error) > TRACK_THRESHOLD_RAD) {
        turretMotor.setControl(request.withPosition(Units.radiansToRotations(cmdRelRad)));
      } else {
        turretMotor.setControl(zeroVolts);
      }

      double turretFieldRad = MathUtil.angleModulus(robotYaw + turretRelRad);
      turretPose = new Pose2d(robotPose.getX(), robotPose.getY(), new Rotation2d(turretFieldRad));
      turretPose2d.accept(turretPose);
      goalPose2d.accept(targetPose);
    }

    @Override
    public void end(boolean interrupted) {
      turretMotor.setControl(zeroVolts);
    }
  }

  ////// -------END OF CLASS------- ///////

  public Command manualMove(DoubleSupplier joystick) {
    return Commands.run(
        () -> {
          if (!zeroed) {
            m_turretMotor.setControl(zeroVolts);
            return;
          }
          double cmd = MathUtil.applyDeadband(joystick.getAsDouble(), 0.10);
          double volts = cmd * 2.0;
          m_turretMotor.setControl(new VoltageOut(volts));
        },
        this);
  }

  // TRIGGERS //
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

  public Command AutoRotate(Supplier<TurretTarget> TurretTarget) {
    return new AutoRotate(
        m_driveTrain, m_turretMotor, TurretTarget, request, ntErrorRad, ntTargetRad, this);
  }

  // --- MOTOR CONFIGS --- //

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
    configs.CurrentLimits.StatorCurrentLimitEnable = true;
    configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    configs.CurrentLimits.StatorCurrentLimit = 25;
    configs.CurrentLimits.SupplyCurrentLimit = 10;

    // -----SOFT LIMITS ----- //
    configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.radiansToRotations(TURRET_MAX);
    configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.radiansToRotations(TURRET_MIN);
    configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    // ---- MOTION MAGIC---- //
    var mm = configs.MotionMagic;
    mm.MotionMagicCruiseVelocity = 4.0;
    mm.MotionMagicAcceleration = 27;
    mm.MotionMagicJerk = 300;

    // --- FEEDBACK / OUTPUT --- //
    configs.Feedback.SensorToMechanismRatio = GEAR_RATIO;
    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    m_turretMotor.getConfigurator().apply(configs);
  }

  @Override
  public void periodic() {
    // Motor angle (mechanism radians) from Talon position (rotations)
    double motorRad = Units.rotationsToRadians(m_turretMotor.getPosition().getValueAsDouble());

    // Motor current
    double motorCurrent = m_turretMotor.getStatorCurrent().getValueAsDouble();

    // Robot rotation (field) from drivetrain pose
    double robotRotRad = m_driveTrain.getState().Pose.getRotation().getRadians();

    // Motor radians field-relative (robot rot + turret rot), wrapped to [-pi, pi)
    double motorFieldRel = MathUtil.angleModulus(robotRotRad + motorRad);

    ntMotorRad.set(motorRad);
    ntMotorCurrent.set(motorCurrent);
    ntRobotRotationRad.set(robotRotRad);
    ntMotorFieldRelativeRad.set(motorFieldRel);
  }

  // in simulationPeriodic():
  public void simulationPeriodic() {
    if (m_sim != null) {
      m_sim.update(0.02);
    }
  }
}
