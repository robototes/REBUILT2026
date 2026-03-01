package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
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
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import frc.robot.util.AllianceUtils;
import java.util.Set;
import java.util.function.DoubleSupplier;

/**
 * This class is designed to handle all things related to Climbing, including auto aligning with the
 * clibing structure and climbing itself.
 *
 * @author Sean Le Nguyen
 */
public class ClimbSubsystem extends SubsystemBase {
  // Climb states
  public enum ClimbLevel {
    L1;
  }

  public enum ClimbState {
    Climbing,
    Idle
  }

  private boolean isZeroed = false;
  private ClimbState climbState = ClimbState.Idle;

  // Motor requests
  private final MotionMagicVoltage request;
  private final VoltageOut volts;
  private final double MAX_VOLTS = 3;

  // hardware and physical objects
  private final CommandSwerveDrivetrain driveTrain;
  private final TalonFX climbMotor;
  private static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AllianceUtils.FIELD_LAYOUT;
  private static final int BLUE_CLIMB_TAG_ID = 31;
  private static final int RED_CLIMB_TAG_ID = 15;

  // Magic Numbers
  private static final double ROBOT_LENGTH = Units.inchesToMeters(35); // Inches
  private static final double CLIMB_Y_OFFSET = Units.inchesToMeters(43.51); // Inches
  private static final double GEAR_RATIO = 15;
  private static final double CLIMB_POSITION_TOLERANCE = 0.1;

  // PID Controller magic numbers
  private static final double POWER_COEFFICIENT = .05;
  private static final double MAX_TRANSLATIONAL_POWER = 2.0;
  private static final double MAX_ROTATIONAL_POWER = 4.0;
  private static final double TRANSLATION_TOLERANCE_METERS = 0.01;
  private static final double ROTATION_TOLERANCE_DEGREES = 1.0;

  // Auto Zero constants
  private static final double STALL_CURRENT = 30; // Amps
  private static final double MAX_HITS = 10;
  private static final double MAX_OMEGA = 0.1;

  // Motor Tunables
  private static final double k_P = 3;
  private static final double k_I = 0;
  private static final double k_D = 3;
  private static final double k_S = 3;
  private static final double k_V = 2;
  private static final double k_A = 2;
  private static final double STATOR_CURRENT_LIMIT = 40;
  private static final double SUPPLY_CURRENT_LIMIT = 30;
  private static final double FORWARD_SOFT_LIMIT = 20; // Rotations
  private static final double REVERSE_SOFT_LIMIT = 0; // Rotations
  private static final double MM_CRUISE_VELOCITY = 1;
  private static final double MM_ACCEL = 1;
  private static final double MM_JERK = 5;

  // Poses and trasnforms
  private static final Transform2d frontBumperOffset =
      new Transform2d(new Translation2d(ROBOT_LENGTH / 2, 0), Rotation2d.kZero);
  private static final Transform2d climbOffSet =
      new Transform2d(new Translation2d(0, CLIMB_Y_OFFSET), Rotation2d.kZero);

  // Climb level positions
  private static final double L1 = 5; // rotations

  // Simulation
  private DoublePublisher ntMotorPos;
  private DoublePublisher ntMotorCurrent;
  private DoublePublisher ntVoltage;

  // Constructor
  public ClimbSubsystem(CommandSwerveDrivetrain driveTrain) {
    // Setup
    this.driveTrain = driveTrain;
    climbMotor = new TalonFX(Hardware.CLIMB_MOTOR_ID);
    configureMotors();
    // Initialize
    request = new MotionMagicVoltage(0);
    volts = new VoltageOut(0);

    // Network tables
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("Climb");
    ntMotorPos = table.getDoubleTopic("Motor Pos (rotations): ").publish();
    ntMotorCurrent = table.getDoubleTopic("Current (amps):").publish();
    ntVoltage = table.getDoubleTopic("Voltage:").publish();
  }

  // Helper functions

  /**
   * sets the motor's position
   *
   * @param position A double representing the absolut target motor rotation, relative to the
   *     motor's zero position
   */
  public void setMotorPosition(double position) {
    climbMotor.setControl(request.withPosition(position));
  }

  /**
   * This method is meant to Manually move the climb subsystem up and down with a voltage output
   *
   * @param joystick A double supplier that supplies the method with the joystick's Y axis value. It
   *     cannot exceed -1 and 1
   */
  public void manualMove(DoubleSupplier joystick) {
    double cmd = MathUtil.applyDeadband(joystick.getAsDouble(), 0.1) * MAX_VOLTS;
    climbMotor.setControl(volts.withOutput(cmd));
  }

  /** zeroes the motor */
  public void zeroMotor() {
    climbMotor.setPosition(0);
  }

  /** stops the motor completely by setting the output voltage to 0 */
  public void stop() {
    climbMotor.setControl(volts.withOutput(0));
  }

  // ---------- COMMANDS ----------- //
  /**
   * This is meant to give the user the option to automatically zero the climb, providing
   * contactless zeroeing.
   *
   * @param runAutoZero a boolean that determines whether to run the routine or not. If no, then
   *     this will just zero the motor where the climb is at
   * @return returns a Command that gives the instructions on how to run AutoZeroRoutine
   */
  public Command AutoZeroRoutine(boolean runAutoZero) {
    final java.util.concurrent.atomic.AtomicInteger hits =
        new java.util.concurrent.atomic.AtomicInteger(0);

    if (!runAutoZero) {
      return Commands.runOnce(
              () -> {
                zeroMotor();
                isZeroed = true;
              },
              this)
          .withName("Manual Zero")
          .onlyIf(() -> !isZeroed);
    }
    return Commands.run(
            () -> {
              climbMotor.setControl(volts.withOutput(-MAX_VOLTS)); // Move down
              // Until it has stalled
              if (Math.abs(climbMotor.getStatorCurrent().getValueAsDouble()) >= STALL_CURRENT
                  && Math.abs(climbMotor.getVelocity().getValueAsDouble()) <= MAX_OMEGA) {
                hits.incrementAndGet();
              } else {
                hits.set(0);
              }
            },
            this)
        .until(() -> hits.get() >= MAX_HITS)
        .finallyDo(
            (interrupted) -> {
              climbMotor.stopMotor();
              if (!interrupted) {
                climbMotor.setPosition(0);
                isZeroed = true;
              }
            })
        .onlyIf(() -> !isZeroed)
        .withTimeout(5);
  }

  // -- CLIMB -- //

  /**
   * This is the Climb routine that will automatically climb if the robot is close enough to the
   * climb
   *
   * @param state an enum value that represents the desired climb level
   * @return returns a Command that provides instructions on how to Climb
   */
  public Command climbRoutine(ClimbLevel state) {
    return Commands.defer(
            () -> {
              final double targetPosition;

              switch (state) {
                case L1:
                  targetPosition = L1;
                  break;
                // case L2: ... (Future proofing)
                default:
                  throw new IllegalArgumentException("Unsupported ClimbLevel: " + state);
              }

              return Commands.run(() -> setMotorPosition(targetPosition), this)
                  .until(
                      () -> {
                        return Math.abs(
                                climbMotor.getPosition().getValueAsDouble() - targetPosition)
                            < CLIMB_POSITION_TOLERANCE;
                      })
                  .andThen(
                      // This pulls the aligns the robot, then pulls up. At this stage we're
                      // assuming the climb subsystem is already hooked on
                      new AutoAlignCommand(), Commands.runOnce(() -> setMotorPosition(0), this))
                  .beforeStarting(() -> climbState = ClimbState.Climbing)
                  .finallyDo((interrupted) -> climbState = ClimbState.Idle);
            },
            Set.of(this)) // Requirements for the deferred command
        .onlyIf(() -> climbState == ClimbState.Idle);
  }

  // -------- AUTO ALIGN -------- //
  /** AutoAlign command that auto aligns the bumper with the climb to the climb */
  public class AutoAlignCommand extends Command {
    // Poses
    private Pose2d climbBumper;
    private Pose2d targetPose;
    // PIDControllers
    private final PIDController pidX = new PIDController(4, 0, 0);
    private final PIDController pidY = new PIDController(4, 0, 0);
    private final PIDController pidRotate = new PIDController(8, 0, 0);
    // Request
    private final SwerveRequest.FieldCentric driveRequest =
        new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest stop =
        driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0);

    // is null
    private boolean m_isInvalid = false;

    public AutoAlignCommand() {
      pidRotate.enableContinuousInput(-Math.PI, Math.PI);
      if (driveTrain != null) {
        addRequirements(ClimbSubsystem.this, driveTrain);
      } else {
        m_isInvalid = true;
      }
      setName("Climb Align");
    }

    @Override
    public void initialize() {
      int tagId = AllianceUtils.isBlue() ? BLUE_CLIMB_TAG_ID : RED_CLIMB_TAG_ID;
      var tagPose = APRIL_TAG_FIELD_LAYOUT.getTagPose(tagId);
      if (tagPose.isEmpty()) {
        m_isInvalid = true;
        return;
      }
      targetPose = tagPose.get().toPose2d().transformBy(climbOffSet);

      pidX.setSetpoint(targetPose.getX());
      pidY.setSetpoint(targetPose.getY());
      // Robot bumper with the climb on it must face to the climb tower
      pidRotate.setSetpoint(targetPose.getRotation().getRadians() + Math.PI);
    }

    @Override
    public void execute() {
      if (m_isInvalid) {
        return;
      }
      climbBumper = driveTrain.getState().Pose.transformBy(frontBumperOffset);
      // X and Y pid calculations
      double powerX = pidX.calculate(climbBumper.getX());
      double powerY = pidY.calculate(climbBumper.getY());
      // Overcome static friction if PID outputs are small
      powerX =
          MathUtil.clamp(
              powerX + POWER_COEFFICIENT * Math.signum(powerX),
              -MAX_TRANSLATIONAL_POWER,
              MAX_TRANSLATIONAL_POWER);
      powerY =
          MathUtil.clamp(
              powerY + POWER_COEFFICIENT * Math.signum(powerY),
              -MAX_TRANSLATIONAL_POWER,
              MAX_TRANSLATIONAL_POWER);
      // Rotational pid calculation
      double powerRotate =
          MathUtil.clamp(
              pidRotate.calculate(climbBumper.getRotation().getRadians()),
              -MAX_ROTATIONAL_POWER,
              MAX_ROTATIONAL_POWER);

      // Apply request
      SwerveRequest request =
          driveRequest.withVelocityX(powerX).withVelocityY(powerY).withRotationalRate(powerRotate);
      driveTrain.setControl(request);
    }

    @Override
    public boolean isFinished() {
      if (m_isInvalid) {
        return true;
      }
      if (climbBumper == null) {
        return false;
      }
      Transform2d robotToClimb = climbBumper.minus(targetPose);
      return robotToClimb.getTranslation().getNorm() < TRANSLATION_TOLERANCE_METERS
          && Math.abs(robotToClimb.getRotation().getDegrees()) < ROTATION_TOLERANCE_DEGREES;
    }

    @Override
    public void end(boolean interrupted) {
      if (driveTrain != null) {
        driveTrain.setControl(stop);
      }
    }
  }

  // ------ Motor configuration ------ //
  private void configureMotors() {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    Slot0Configs slot0 = configs.Slot0;

    slot0.kP = k_P;
    slot0.kI = k_I;
    slot0.kD = k_D;
    slot0.kS = k_S;
    slot0.kV = k_V;
    slot0.kA = k_A;

    // CURRENT LIMITS
    configs.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;
    configs.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT;
    configs.CurrentLimits.StatorCurrentLimitEnable = true;
    configs.CurrentLimits.SupplyCurrentLimitEnable = true;

    // SOFT LIMITS
    configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = FORWARD_SOFT_LIMIT;
    configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = REVERSE_SOFT_LIMIT;
    configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    // Motion magic
    var mm = configs.MotionMagic;
    mm.MotionMagicCruiseVelocity = MM_CRUISE_VELOCITY;
    mm.MotionMagicAcceleration = MM_ACCEL;
    mm.MotionMagicJerk = MM_JERK;

    // Feedback / output
    configs.Feedback.SensorToMechanismRatio = GEAR_RATIO;
    configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    climbMotor.getConfigurator().apply(configs);
  }

  // Periodic
  @Override
  public void periodic() {
    // set motor position in network tables
    ntVoltage.set(climbMotor.getMotorVoltage().getValueAsDouble());
    ntMotorPos.set(climbMotor.getPosition().getValueAsDouble());
    ntMotorCurrent.set(climbMotor.getStatorCurrent().getValueAsDouble());
  }
}
