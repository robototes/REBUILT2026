package frc.robot.subsystems.drivebase;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.CompTunerConstants;
import frc.robot.util.AllianceUtils;
import java.util.function.Supplier;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily
 * be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean m_hasAppliedOperatorPerspective = false;

  // Field dimensions from the layout
  private static final double FIELD_X_MAX = AllianceUtils.FIELD_LAYOUT.getFieldLength();
  private static final double FIELD_Y_MAX = AllianceUtils.FIELD_LAYOUT.getFieldWidth();

  /** Threshold in degrees to detect if the robot is tilted (on a bump or beached). */
  private static final double kTiltThresholdDegrees = 10.0;

  /* Bump location constants - Zones for both Blue and Red alliances */
  private static final double kBumpBlueXMin = 3.6;
  private static final double kBumpBlueXMax = 5.6;
  private static final double kBumpRedXMin = 11.0;
  private static final double kBumpRedXMax = 12.95;
  private static final double kBumpYMin = 1.5;
  private static final double kBumpYMax = 6.5;

  /* Bump simulation constants */
  private static final double kBumpSimMaxPitchDegrees = 15.0; // Max pitch angle when on bump
  private static final double kBumpSimMaxRollDegrees = 5.0; // Max roll angle when traversing bump width-wise
  private static final double kBumpSimSlopeLength = 0.5; // meters, length of the slope for pitch/roll change

  /* Swerve requests to apply during SysId characterization */
  private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization =
      new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization =
      new SwerveRequest.SysIdSwerveSteerGains();
  private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization =
      new SwerveRequest.SysIdSwerveRotation();

  /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
  private final SysIdRoutine m_sysIdRoutineTranslation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> setControl(m_translationCharacterization.withVolts(output)), null, this));

  /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
  private final SysIdRoutine m_sysIdRoutineSteer =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(7), // Use dynamic voltage of 7 V
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> setControl(m_steerCharacterization.withVolts(volts)), null, this));

  /*
   * SysId routine for characterizing rotation.
   * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
   * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
   */
  private final SysIdRoutine m_sysIdRoutineRotation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              /* This is in radians per second², but SysId only supports "volts per second" */
              Volts.of(Math.PI / 6).per(Second),
              /* This is in radians per second, but SysId only supports "volts" */
              Volts.of(Math.PI),
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
              },
              null,
              this));

  /* The SysId routine to test */
  private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param modules Constants for each specific module
   */
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to
   *     0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
   * @param modules Constants for each specific module
   */
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, odometryUpdateFrequency, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to
   *     0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
   * @param odometryStandardDeviation The standard deviation for odometry calculation in the form
   *     [x, y, theta]ᵀ, with units in meters and radians
   * @param visionStandardDeviation The standard deviation for vision calculation in the form [x, y,
   *     theta]ᵀ, with units in meters and radians
   * @param modules Constants for each specific module
   */
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      Matrix<N3, N1> odometryStandardDeviation,
      Matrix<N3, N1> visionStandardDeviation,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(
        drivetrainConstants,
        odometryUpdateFrequency,
        odometryStandardDeviation,
        visionStandardDeviation,
        modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  /**
   * Returns a command that applies the specified control request to this swerve drivetrain.
   *
   * @param request Function returning the request to apply
   * @return Command to run
   */
  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get())).withName("Apply Drivebase Request");
  }

  /**
   * Runs the SysId Quasistatic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Quasistatic test
   * @return Command to run
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.quasistatic(direction).withName("SysId Quasistatic");
  }

  /**
   * Runs the SysId Dynamic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Dynamic test
   * @return Command to run
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.dynamic(direction).withName("SysId Dynamic");
  }

  @Override
  public void periodic() {
    /*
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is disabled.
     * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
     */
    if (RobotBase.isSimulation()
        || !m_hasAppliedOperatorPerspective
        || DriverStation.isDisabled()) {
      DriverStation.getAlliance()
          .ifPresent(
              alliance -> {
                this.setOperatorPerspectiveForward(
                    alliance == DriverStation.Alliance.Blue
                        ? kBlueAlliancePerspectiveRotation
                        : kRedAlliancePerspectiveRotation);
              });
    }

    // Refresh gyro signals once per loop to ensure all checks use fresh, synchronized data
    getPigeon2().getPitch().refresh();
    getPigeon2().getRoll().refresh();

    clampPoseToField();

    // Pushing telemetry to SmartDashboard every loop as an alternative to initSendable
    SmartDashboard.putBoolean("Drivebase/isOnBump", isOnBump());
    SmartDashboard.putBoolean("Drivebase/inBumpZone", isRobotInBumpZone());
    SmartDashboard.putBoolean("Drivebase/isBeached", isBeached(kTiltThresholdDegrees));
    SmartDashboard.putNumber("Drivebase/pitchDegrees", getPigeon2().getPitch().getValueAsDouble());
    SmartDashboard.putNumber("Drivebase/rollDegrees", getPigeon2().getRoll().getValueAsDouble());
  }

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - m_lastSimTime;
              m_lastSimTime = currentTime;

              /* use the measured time delta, get battery voltage from WPILib */
              // Simulate bump effect on gyro
              Pose2d currentPose = getState().Pose; // Get the current simulated pose

              double simulatedPitchDegrees = 0.0;
              double simulatedRollDegrees = 0.0;

              double x = currentPose.getX();
              double y = currentPose.getY();

              boolean inBlueBumpZone = (x >= kBumpBlueXMin && x <= kBumpBlueXMax && y >= kBumpYMin && y <= kBumpYMax);
              boolean inRedBumpZone = (x >= kBumpRedXMin && x <= kBumpRedXMax && y >= kBumpYMin && y <= kBumpYMax);

              if (inBlueBumpZone || inRedBumpZone) {
                  double currentBumpXMin, currentBumpXMax;
                  if (inBlueBumpZone) {
                      currentBumpXMin = kBumpBlueXMin;
                      currentBumpXMax = kBumpBlueXMax;
                  } else { // inRedBumpZone
                      currentBumpXMin = kBumpRedXMin;
                      currentBumpXMax = kBumpRedXMax;
                  }

                  // Calculate pitch based on X position relative to the bump
                  // Assuming the bump is a trapezoid profile along the X-axis
                  if (x < currentBumpXMin + kBumpSimSlopeLength) { // Rising slope
                      double progress = (x - currentBumpXMin) / kBumpSimSlopeLength;
                      simulatedPitchDegrees = kBumpSimMaxPitchDegrees * progress;
                  } else if (x > currentBumpXMax - kBumpSimSlopeLength) { // Falling slope
                      double progress = (currentBumpXMax - x) / kBumpSimSlopeLength;
                      simulatedPitchDegrees = kBumpSimMaxPitchDegrees * progress;
                  } else { // Flat top of the bump
                      simulatedPitchDegrees = kBumpSimMaxPitchDegrees;
                  }

                  // Simulate roll if the robot is traversing the bump width-wise
                  // This assumes a similar trapezoidal profile along the Y-axis within the bump zone
                  double currentBumpYMin = kBumpYMin;
                  double currentBumpYMax = kBumpYMax;

                  if (y < currentBumpYMin + kBumpSimSlopeLength) { // Rising slope along Y
                      double progress = (y - currentBumpYMin) / kBumpSimSlopeLength;
                      simulatedRollDegrees = kBumpSimMaxRollDegrees * progress;
                  } else if (y > currentBumpYMax - kBumpSimSlopeLength) { // Falling slope along Y
                      double progress = (currentBumpYMax - y) / kBumpSimSlopeLength;
                      simulatedRollDegrees = kBumpSimMaxRollDegrees * progress;
                  } else { // Flat top along Y
                      // If the robot is fully within the Y bounds of the bump,
                      // we might not want a constant roll unless the bump itself is angled.
                      // For simplicity, let's assume roll is only affected on the edges.
                      simulatedRollDegrees = 0.0;
                  }

                  // If the robot is driving diagonally, both pitch and roll could be affected.
                  // For this 2.5D sim, we're applying them independently based on X and Y position.
              }

              // Update the simulated Pigeon2's pitch and roll
              // The Pigeon2's SimState expects degrees
              getPigeon2().getSimState().setPitch(simulatedPitchDegrees);
              getPigeon2().getSimState().setRoll(simulatedRollDegrees);

              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }

  // returns the speeds for logging purposes
  public ChassisSpeeds returnSpeeds() {
    return getState().Speeds;
  }

  // method for on-demand coasting control
  public Command coastMotors() {
    return startEnd(
            () -> {
              configNeutralMode(NeutralModeValue.Coast);
            },
            () -> {
              configNeutralMode(NeutralModeValue.Brake);
            })
        .ignoringDisable(true)
        .withName("Coast Swerve");
  }

  // method for braking the motors after coasting
  public void brakeMotors() {
    configNeutralMode(NeutralModeValue.Brake);
  }

  public boolean isStationary() {
    var speeds = getState().Speeds;
    return MathUtil.isNear(0, speeds.vxMetersPerSecond, 0.01)
        && MathUtil.isNear(0, speeds.vyMetersPerSecond, 0.01)
        && MathUtil.isNear(0, speeds.omegaRadiansPerSecond, Units.degreesToRadians(2));
  }

  public double[] getWheelRotations() {
    double wheelCircumference = tau(CompTunerConstants.kWheelRadius.abs(Meter));
    double[] values = new double[4];
    for (int i = 0; i < values.length; i++) {
      values[i] = getState().ModulePositions[i].distanceMeters / wheelCircumference;
    }
    return values;
  }

  public static double tau(double value) {
    return value * 2 * Math.PI;
  }

  /** Clamps the pose estimator to the field boundary. Does not affect driving. */
  private void clampPoseToField() {
    Pose2d current = getState().Pose;
    double clampedX = MathUtil.clamp(current.getX(), 0.0, FIELD_X_MAX);
    double clampedY = MathUtil.clamp(current.getY(), 0.0, FIELD_Y_MAX);

    if (clampedX != current.getX() || clampedY != current.getY()) {
      resetPose(new Pose2d(new Translation2d(clampedX, clampedY), current.getRotation()));
    }
  }

  /**
   * Checks if the robot's current pose is within the X/Y coordinates defined for the bump.
   *
   * @return True if the robot is in the bump area.
   */
  public boolean isRobotInBumpZone() {
    Pose2d pose = getState().Pose;
    double x = pose.getX();
    double y = pose.getY();

    boolean withinY = y >= kBumpYMin && y <= kBumpYMax;
    boolean inBlueBump = x >= kBumpBlueXMin && x <= kBumpBlueXMax;
    boolean inRedBump = x >= kBumpRedXMin && x <= kBumpRedXMax;

    return withinY && (inBlueBump || inRedBump);
  }

  /**
   * Returns true if the robot's pitch or roll exceeds the default threshold,
   * indicating it is currently on a bump or beached.
   *
   * @return True if the robot is significantly tilted.
   */
  public boolean isOnBump() {
    return  isRobotInBumpZone() && isBeached(kTiltThresholdDegrees);
  }

  /**
   * Detects if the robot is beached or on a bump by checking if either the pitch or roll
   * exceeds the specified threshold. This check is constrained by the
   * robot's position to ensure that tilt caused by other factors (like rapid acceleration
   * or collisions outside the bump area) isn't falsely identified as being beached on the bump.
   *
   * @param pitchThreshold The angle threshold in degrees.
   * @return True if the tilt exceeds the threshold.
   */
  private boolean isBeached(double pitchThreshold) {
    double pitch = Math.abs(getPigeon2().getPitch().getValueAsDouble());
    double roll = Math.abs(getPigeon2().getRoll().getValueAsDouble());

    return(pitch > pitchThreshold || roll > pitchThreshold);
  }
}
