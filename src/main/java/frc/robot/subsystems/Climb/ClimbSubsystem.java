package frc.robot.subsystems.Climb;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Gs;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;
import frc.robot.Subsystems;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import frc.robot.util.AllianceUtils;
import java.util.HashMap;
import java.util.Set;
import java.util.function.DoubleSupplier;

/**
 * This class is designed to handle all things related to Climbing, including auto aligning with the
 * climbing structure and climbing itself.
 *
 * @author Sean Le Nguyen
 */
public class ClimbSubsystem extends SubsystemBase {
  // Climb states
  public enum ClimbLevel {
    L0,
    L1,
    L2,
    L3
  }

  public enum ClimbState {
    Attached,
    Detached
  }

  // Maps
  private final HashMap<ClimbLevel, Double> climbLevels = new HashMap<>();

  private boolean isZeroed = false;
  private ClimbState climbState = ClimbState.Detached;
  private double targetLevel = 0;

  // Motor requests
  private final MotionMagicVoltage mmRequest;
  private final VoltageOut voltageRequest;
  private static final double MAX_VOLTS = 3;

  // Hardware and physical objects
  private final CommandSwerveDrivetrain driveTrain;
  private final ClimbPivot climbPivotSubsystem;
  private final Pigeon2 imu;
  private final TalonFX climbMotor;
  private static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AllianceUtils.FIELD_LAYOUT;
  private static final int BLUE_CLIMB_TAG_ID = 31;
  private static final int RED_CLIMB_TAG_ID = 15;

  // Physical Constants
  private static final double ROBOT_WIDTH = Units.inchesToMeters(30);
  private static final double GEAR_RATIO = 15;

  // Position offsets and tolerances
  private static final double CLIMB_POSITION_TOLERANCE = 0.1;
  private static final double MIN_DIST = Units.feetToMeters(1.5);
  private static final Rotation2d DETACH_ROTATION = Rotation2d.fromDegrees(40);

  // Auto Align and Climb Constants
  private static final double STAGE1_APPROACH_OFFSET = 0.1; // Meters
  private static final double CLIMB_Y_OFFSET = Units.inchesToMeters(43.51);
  private static final double MIN_Gs = 1;
  private static final double ATTACH_TIMEOUT = 2; // Seconds
  private static final Debouncer has_maintained_current = new Debouncer(0.2, DebounceType.kRising);
  private static final double MIN_ATTACHED_AMPS = 20;

  // PID Controller Constants
  private static final double POWER_COEFFICIENT = .05;
  private static final double MAX_TRANSLATIONAL_POWER = 2.0;
  private static final double MAX_ROTATIONAL_POWER = 4.0;
  private static final double TRANSLATION_TOLERANCE_METERS = 0.03;
  private static final double ROTATION_TOLERANCE_RADIANS = Units.degreesToRadians(0.5);

  // Motor Tunables
  private static final double KP = 3;
  private static final double KI = 0;
  private static final double KD = 3;
  private static final double KS = 3;
  private static final double KV = 2;
  private static final double KA = 2;
  private static final double STATOR_CURRENT_LIMIT = 40;
  private static final double SUPPLY_CURRENT_LIMIT = 30;
  private static final double MM_CRUISE_VELOCITY = 1;
  private static final double MM_ACCEL = 1;
  private static final double MM_JERK = 5;

  // Poses and transforms
  private static final Transform2d CLIMB_TRANSFORM =
      new Transform2d(new Translation2d(ROBOT_WIDTH / 2, 0), Rotation2d.k180deg);
  private static final Transform2d CLIMB_STRUCTURE_OFFSET =
      new Transform2d(new Translation2d(0, CLIMB_Y_OFFSET), Rotation2d.kZero);

  // Network Tables
  private final DoublePublisher ntMotorPos;
  private final DoublePublisher ntMotorCurrent;
  private final DoublePublisher ntVoltage;

  // Status signals
  private final StatusSignal<Voltage> ssVoltage;
  private final StatusSignal<Current> ssCurrent;
  private final StatusSignal<Angle> ssMotorPos;
  private final StatusSignal<LinearAcceleration> ss_xAccel;
  private final StatusSignal<LinearAcceleration> ss_yAccel;

  public ClimbSubsystem(Subsystems s) {
    this.driveTrain = s.drivebaseSubsystem;
    imu = s.drivebaseSubsystem.getPigeon2();
    this.climbPivotSubsystem = s.climbPivotSubsystem;

    climbMotor = new TalonFX(Hardware.CLIMB_MOTOR_ID);
    configureMotors();

    mmRequest = new MotionMagicVoltage(0);
    voltageRequest = new VoltageOut(0);

    NetworkTable table = NetworkTableInstance.getDefault().getTable("Climb");
    ntMotorPos = table.getDoubleTopic("Motor Pos (rotations)").publish();
    ntMotorCurrent = table.getDoubleTopic("Current (amps)").publish();
    ntVoltage = table.getDoubleTopic("Voltage").publish();

    ssVoltage = climbMotor.getMotorVoltage();
    ssCurrent = climbMotor.getStatorCurrent();
    ssMotorPos = climbMotor.getPosition();

    climbLevels.put(ClimbLevel.L0, 0.0);
    climbLevels.put(ClimbLevel.L1, 5.0);
    climbLevels.put(ClimbLevel.L2, 10.0);
    climbLevels.put(ClimbLevel.L3, 15.0);

    ss_xAccel = imu.getAccelerationY();
    ss_yAccel = imu.getAccelerationX();
  }

  // Pose helpers
  private Pose2d getTagPose() {
    return APRIL_TAG_FIELD_LAYOUT
        .getTagPose(AllianceUtils.isBlue() ? BLUE_CLIMB_TAG_ID : RED_CLIMB_TAG_ID)
        .orElseThrow(() -> new IllegalStateException("Climb AprilTag not found."))
        .toPose2d();
  }

  private Pose2d getStage1Pose() {
    return getTagPose()
        .transformBy(
            CLIMB_STRUCTURE_OFFSET.plus(
                new Transform2d(STAGE1_APPROACH_OFFSET, 0, Rotation2d.kZero)));
  }

  private Pose2d getStage2Pose() {
    return getTagPose().transformBy(CLIMB_STRUCTURE_OFFSET);
  }

  // Check to see if we're actually attached
  public boolean PassedAccelerometerTest() {
    StatusSignal.refreshAll(ss_xAccel, ss_yAccel);
    double x = ss_xAccel.getValue().in(Gs);
    double y = ss_yAccel.getValue().in(Gs);
    double totalAccel = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    if (totalAccel >= MIN_Gs) {
      return true;
    }
    return false;
  }

  public boolean PassedrollerTest() {
    boolean aboveThreshold = false;
    if (ssCurrent.getValue().in(Amp) > MIN_ATTACHED_AMPS) aboveThreshold = !aboveThreshold;
    return has_maintained_current.calculate(aboveThreshold);
  }

  // Motor helpers
  public void zeroMotor() {
    isZeroed = true;
    climbMotor.setPosition(0);
  }

  public void stop() {
    climbMotor.setControl(voltageRequest.withOutput(0));
  }

  public boolean isWithinTarget() {
    return Math.abs(targetLevel - ssMotorPos.getValue().in(Rotations)) < CLIMB_POSITION_TOLERANCE;
  }

  // Position modifiers
  public void setMotorPosition(double position) {
    climbMotor.setControl(mmRequest.withPosition(position));
  }

  public void setVoltage(double volts) {
    climbMotor.setVoltage(volts);
  }

  public void manualMove(DoubleSupplier joystick) {
    double cmd = MathUtil.applyDeadband(joystick.getAsDouble(), 0.1) * MAX_VOLTS;
    climbMotor.setControl(voltageRequest.withOutput(cmd));
  }

  // ---------- COMMANDS ----------- //
  public Command manualClimbCommand(DoubleSupplier joystickInput) {
    return Commands.run(() -> manualMove(joystickInput), this).finallyDo(this::stop);
  }

  public Command goToLevel(ClimbLevel state) {
    return Commands.defer(
            () -> {
              double targetPos = climbLevels.get(state);
              Command moveCommand =
                  Commands.runOnce(() -> setMotorPosition(targetPos), this)
                      .beforeStarting(() -> targetLevel = targetPos)
                      .until(this::isWithinTarget);

              return (state == ClimbLevel.L0) ? moveCommand.andThen(detachClimb()) : moveCommand;
            },
            Set.of(this, driveTrain))
        .onlyIf(() -> isZeroed && climbState == ClimbState.Attached);
  }

  public Command detachClimb() {
    return new AutoAlignCommand(
            getStage2Pose().rotateBy(DETACH_ROTATION), CLIMB_STRUCTURE_OFFSET.getTranslation())
        .finallyDo(
            () -> {
              climbPivotSubsystem.stow();
              climbState = ClimbState.Detached;
            });
  }

  public Command attachToClimb() {
    return Commands.sequence(
            Commands.parallel(new AutoAlignCommand(getStage1Pose(), Translation2d.kZero)),
            climbPivotSubsystem.deployCommand().until(() -> climbPivotSubsystem.isDeployed()),
            Commands.parallel(
                new AutoAlignCommand(getStage2Pose(), Translation2d.kZero),
                Commands.sequence(
                        Commands.runOnce(() -> setVoltage(MAX_VOLTS)),
                        Commands.run(
                            () -> {
                              if (PassedAccelerometerTest() && PassedAccelerometerTest()) {
                                climbState = ClimbState.Attached;
                              }
                            }))
                    .onlyWhile(() -> climbState == ClimbState.Detached)
                    .withTimeout(ATTACH_TIMEOUT)))
        .onlyIf(this::isNearEnoughToClimb);
  }

  public boolean isNearEnoughToClimb() {
    return getStage1Pose()
            .getTranslation()
            .minus(driveTrain.getState().Pose.transformBy(CLIMB_TRANSFORM).getTranslation())
            .getNorm()
        < MIN_DIST;
  }

  // -------- AUTO ALIGN -------- //
  public class AutoAlignCommand extends Command {
    private Pose2d currentClimbPose;
    private final Pose2d targetPose;
    private final PIDController pidX = new PIDController(4, 0, 0);
    private final PIDController pidY = new PIDController(4, 0, 0);
    private final PIDController pidRotate = new PIDController(8, 0, 0);

    private final SwerveRequest.FieldCentric driveRequest =
        new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

    private final Translation2d centerOfRotation;
    private final boolean isInvalid;

    public AutoAlignCommand(Pose2d target, Translation2d centerOfRotation) {
      pidRotate.enableContinuousInput(-Math.PI, Math.PI);
      this.targetPose = target;
      this.centerOfRotation = centerOfRotation;
      this.isInvalid = (driveTrain == null);

      if (!isInvalid) addRequirements(ClimbSubsystem.this, driveTrain);
      setName("Climb Align");
    }

    @Override
    public void initialize() {
      pidX.reset();
      pidY.reset();
      pidRotate.reset();

      pidX.setSetpoint(targetPose.getX());
      pidY.setSetpoint(targetPose.getY());
      pidRotate.setSetpoint(targetPose.getRotation().getRadians());

      pidX.setTolerance(TRANSLATION_TOLERANCE_METERS);
      pidY.setTolerance(TRANSLATION_TOLERANCE_METERS);
      pidRotate.setTolerance(ROTATION_TOLERANCE_RADIANS);
    }

    @Override
    public void execute() {
      if (isInvalid) return;

      currentClimbPose = driveTrain.getState().Pose.transformBy(CLIMB_TRANSFORM);

      double powerX = pidX.calculate(currentClimbPose.getX());
      double powerY = pidY.calculate(currentClimbPose.getY());

      // Friction compensation logic
      powerX =
          MathUtil.clamp(
              Math.abs(pidX.getError()) < TRANSLATION_TOLERANCE_METERS
                  ? 0
                  : powerX + POWER_COEFFICIENT * Math.signum(powerX),
              -MAX_TRANSLATIONAL_POWER,
              MAX_TRANSLATIONAL_POWER);

      powerY =
          MathUtil.clamp(
              Math.abs(pidY.getError()) < TRANSLATION_TOLERANCE_METERS
                  ? 0
                  : powerY + POWER_COEFFICIENT * Math.signum(powerY),
              -MAX_TRANSLATIONAL_POWER,
              MAX_TRANSLATIONAL_POWER);

      double powerRotate =
          MathUtil.clamp(
              pidRotate.calculate(currentClimbPose.getRotation().getRadians()),
              -MAX_ROTATIONAL_POWER,
              MAX_ROTATIONAL_POWER);

      driveTrain.setControl(
          driveRequest
              .withVelocityX(powerX)
              .withVelocityY(powerY)
              .withRotationalRate(powerRotate)
              .withCenterOfRotation(centerOfRotation));
    }

    @Override
    public boolean isFinished() {
      return pidX.atSetpoint() && pidY.atSetpoint() && pidRotate.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
      if (!isInvalid) driveTrain.setControl(new SwerveRequest.Idle());
    }
  }

  private void configureMotors() {
    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.Slot0.kP = KP;
    configs.Slot0.kI = KI;
    configs.Slot0.kD = KD;
    configs.Slot0.kS = KS;
    configs.Slot0.kV = KV;
    configs.Slot0.kA = KA;

    configs.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;
    configs.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT;
    configs.CurrentLimits.StatorCurrentLimitEnable = true;
    configs.CurrentLimits.SupplyCurrentLimitEnable = true;

    configs.MotionMagic.MotionMagicCruiseVelocity = MM_CRUISE_VELOCITY;
    configs.MotionMagic.MotionMagicAcceleration = MM_ACCEL;
    configs.MotionMagic.MotionMagicJerk = MM_JERK;

    configs.Feedback.SensorToMechanismRatio = GEAR_RATIO;
    configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    climbMotor.getConfigurator().apply(configs);
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(ssVoltage, ssMotorPos, ssCurrent);

    ntVoltage.set(ssVoltage.getValueAsDouble());
    ntMotorPos.set(ssMotorPos.getValueAsDouble());
    ntMotorCurrent.set(ssCurrent.getValueAsDouble());
  }
}
