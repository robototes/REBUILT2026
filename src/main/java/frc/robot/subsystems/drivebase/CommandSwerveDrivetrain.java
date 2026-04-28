package frc.robot.subsystems.drivebase;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
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
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.CompTunerConstants;
import frc.robot.util.AllianceUtils;
import frc.robot.util.KinematicFilter;
import frc.robot.util.KinematicFilterInfused;
import java.util.function.Supplier;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily
 * be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private static final double kMaxFilterDt = 0.05;
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

  // Pigeon Status Signals
  private StatusSignal<LinearAcceleration> ss_XAccel;
  private StatusSignal<LinearAcceleration> ss_YAccel;
  private StatusSignal<AngularVelocity> ss_Omega;

  private static final double FIELD_POS_PROCESS_STD_DEV = 0.02;
  private static final double FIELD_VEL_PROCESS_STD_DEV = 0.05;
  private static final double FIELD_ACCEL_PROCESS_STD_DEV = 4.0;
  private static final double FIELD_POS_MEASUREMENT_STD_DEV = 0.10;
  private static final double FIELD_POSE_VEL_MEASUREMENT_STD_DEV = 0.25;
  private static final double FIELD_VISION_VEL_MEASUREMENT_STD_DEV = 0.15;
  private static final double FIELD_VEL_MEASUREMENT_STD_DEV = 0.05;
  private static final double FIELD_ACCEL_MEASUREMENT_STD_DEV = 0.35;
  private static final double VISION_VELOCITY_TRUST_RAMP_SECONDS = 1.0;
  private static final double VISION_VELOCITY_TRUST_DECAY_SECONDS = 0.35;
  private static final double VISION_VELOCITY_MAX_AGE_SECONDS = 0.35;
  private static final double VISION_VELOCITY_CONSISTENCY_TOLERANCE = 2.0;
  private static final double VISION_VELOCITY_MAX_SPEED = 12.0;

  private KinematicFilterInfused filteredFieldX =
      new KinematicFilterInfused(
          FIELD_POS_PROCESS_STD_DEV,
          FIELD_VEL_PROCESS_STD_DEV,
          FIELD_ACCEL_PROCESS_STD_DEV,
          FIELD_POS_MEASUREMENT_STD_DEV,
          FIELD_POSE_VEL_MEASUREMENT_STD_DEV,
          FIELD_VISION_VEL_MEASUREMENT_STD_DEV,
          FIELD_VEL_MEASUREMENT_STD_DEV,
          FIELD_ACCEL_MEASUREMENT_STD_DEV,
          0.02);
  private KinematicFilterInfused filteredFieldY =
      new KinematicFilterInfused(
          FIELD_POS_PROCESS_STD_DEV,
          FIELD_VEL_PROCESS_STD_DEV,
          FIELD_ACCEL_PROCESS_STD_DEV,
          FIELD_POS_MEASUREMENT_STD_DEV,
          FIELD_POSE_VEL_MEASUREMENT_STD_DEV,
          FIELD_VISION_VEL_MEASUREMENT_STD_DEV,
          FIELD_VEL_MEASUREMENT_STD_DEV,
          FIELD_ACCEL_MEASUREMENT_STD_DEV,
          0.02);
  private KinematicFilter filteredAlpha = new KinematicFilter(0.005, 0.5, 0.001, 0.02);

  private final double nominalDt = 0.02;
  private Pigeon2 pigeon;
  private Translation2d lastFieldVelocity = new Translation2d();
  private double lastPeriodicTime = 0;
  private double simAccelX = 0;
  private double simAccelY = 0;
  private Translation2d acceptedVisionFieldVelocity = new Translation2d();
  private double acceptedVisionVelocityTimestamp = Double.NEGATIVE_INFINITY;
  private double acceptedVisionVelocityTrust = 0.0;
  private boolean hasAcceptedVisionVelocity = false;

  private DoublePublisher pub_XAccel;
  private DoublePublisher pub_YAccel;
  private DoublePublisher pub_Alpha;
  private DoublePublisher pub_RawFieldXAccel;
  private DoublePublisher pub_RawFieldYAccel;
  private DoublePublisher pub_PoseDerivedFieldXVelocity;
  private DoublePublisher pub_PoseDerivedFieldYVelocity;
  private DoublePublisher pub_AcceptedVisionFieldXVelocity;
  private DoublePublisher pub_AcceptedVisionFieldYVelocity;
  private DoublePublisher pub_AcceptedVisionVelocityTrust;

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
    init();
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
    init();
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
    init();
  }

  private void init() {
    if (Utils.isSimulation()) {
      startSimThread();
    }
    this.pigeon = getPigeon2();
    ss_XAccel = pigeon.getAccelerationX();
    ss_YAccel = pigeon.getAccelerationY();
    ss_Omega = pigeon.getAngularVelocityZWorld();
    ss_XAccel.setUpdateFrequency(100);
    ss_YAccel.setUpdateFrequency(100);
    ss_Omega.setUpdateFrequency(100);

    var nt = NetworkTableInstance.getDefault();
    pub_XAccel = nt.getDoubleTopic("/DriveState/Accelerations/filteredAccelX").publish();
    pub_YAccel = nt.getDoubleTopic("/DriveState/Accelerations/filteredAccelY").publish();
    pub_Alpha = nt.getDoubleTopic("/DriveState/Accelerations/filteredAccelOmega").publish();
    pub_RawFieldXAccel = nt.getDoubleTopic("/DriveState/Accelerations/rawFieldAccelX").publish();
    pub_RawFieldYAccel = nt.getDoubleTopic("/DriveState/Accelerations/rawFieldAccelY").publish();
    pub_PoseDerivedFieldXVelocity =
        nt.getDoubleTopic("/DriveState/Velocities/poseDerivedFieldVelocityX").publish();
    pub_PoseDerivedFieldYVelocity =
        nt.getDoubleTopic("/DriveState/Velocities/poseDerivedFieldVelocityY").publish();
    pub_AcceptedVisionFieldXVelocity =
        nt.getDoubleTopic("/DriveState/Velocities/acceptedVisionFieldVelocityX").publish();
    pub_AcceptedVisionFieldYVelocity =
        nt.getDoubleTopic("/DriveState/Velocities/acceptedVisionFieldVelocityY").publish();
    pub_AcceptedVisionVelocityTrust =
        nt.getDoubleTopic("/DriveState/Velocities/acceptedVisionVelocityTrust").publish();
    // initialize with zeros
    pub_XAccel.set(0.0);
    pub_YAccel.set(0.0);
    pub_Alpha.set(0.0);
    pub_RawFieldXAccel.set(0.0);
    pub_RawFieldYAccel.set(0.0);
    pub_PoseDerivedFieldXVelocity.set(0.0);
    pub_PoseDerivedFieldYVelocity.set(0.0);
    pub_AcceptedVisionFieldXVelocity.set(0.0);
    pub_AcceptedVisionFieldYVelocity.set(0.0);
    pub_AcceptedVisionVelocityTrust.set(0.0);
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
                m_hasAppliedOperatorPerspective = true;
              });
    }
    clampPoseToField();

    double now = Timer.getFPGATimestamp();
    double dt = (lastPeriodicTime > 0) ? (now - lastPeriodicTime) : nominalDt;
    lastPeriodicTime = now;
    if (!Double.isFinite(dt) || dt <= 0.0) {
      dt = nominalDt;
    } else {
      dt = Math.min(dt, kMaxFilterDt);
    }

    var imuRefreshStatus = StatusSignal.refreshAll(ss_XAccel, ss_YAccel, ss_Omega);
    boolean imuSignalsHealthy = imuRefreshStatus.isOK();
    boolean xAccelHealthy = imuSignalsHealthy && ss_XAccel.getStatus().isOK();
    boolean yAccelHealthy = imuSignalsHealthy && ss_YAccel.getStatus().isOK();
    boolean omegaHealthy = imuSignalsHealthy && ss_Omega.getStatus().isOK();

    var currentState = getState();
    Pose2d currentPose = currentState.Pose;
    Rotation2d poseRotation = currentPose.getRotation();
    ChassisSpeeds robotSpeeds = currentState.Speeds; // robot-relative

    ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, poseRotation);
    Translation2d fieldVelocity =
        new Translation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);

    if (RobotBase.isSimulation()) {
      simAccelX = (fieldVelocity.getX() - lastFieldVelocity.getX()) / dt;
      simAccelY = (fieldVelocity.getY() - lastFieldVelocity.getY()) / dt;
      lastFieldVelocity = fieldVelocity;
    }

    double currentOmega =
        RobotBase.isSimulation()
            ? robotSpeeds.omegaRadiansPerSecond
            : omegaHealthy ? ss_Omega.getValue().in(RadiansPerSecond) : Double.NaN;

    double rawAX =
        RobotBase.isSimulation()
            ? simAccelX
            : xAccelHealthy ? ss_XAccel.getValue().in(MetersPerSecondPerSecond) : Double.NaN;
    double rawAY =
        RobotBase.isSimulation()
            ? simAccelY
            : yAccelHealthy ? ss_YAccel.getValue().in(MetersPerSecondPerSecond) : Double.NaN;

    Translation2d fieldAccel =
        RobotBase.isSimulation()
            ? new Translation2d(rawAX, rawAY)
            : new Translation2d(rawAX, rawAY).rotateBy(poseRotation);
    double visionVelocityTrust = getCurrentAcceptedVisionVelocityTrust(now, dt);
    Translation2d acceptedVisionVelocity =
        visionVelocityTrust > 0.0
            ? acceptedVisionFieldVelocity
            : new Translation2d(Double.NaN, Double.NaN);

    pub_RawFieldXAccel.set(fieldAccel.getX());
    pub_RawFieldYAccel.set(fieldAccel.getY());

    filteredFieldX.update(
        currentPose.getX(),
        fieldVelocity.getX(),
        acceptedVisionVelocity.getX(),
        visionVelocityTrust,
        fieldAccel.getX(),
        dt);
    filteredFieldY.update(
        currentPose.getY(),
        fieldVelocity.getY(),
        acceptedVisionVelocity.getY(),
        visionVelocityTrust,
        fieldAccel.getY(),
        dt);
    filteredAlpha.update(currentOmega, dt);

    double poseDerivedFieldXVelocity = filteredFieldX.getPoseDerivedVelocity();
    double poseDerivedFieldYVelocity = filteredFieldY.getPoseDerivedVelocity();
    pub_PoseDerivedFieldXVelocity.set(
        Double.isFinite(poseDerivedFieldXVelocity) ? poseDerivedFieldXVelocity : 0.0);
    pub_PoseDerivedFieldYVelocity.set(
        Double.isFinite(poseDerivedFieldYVelocity) ? poseDerivedFieldYVelocity : 0.0);
    pub_AcceptedVisionFieldXVelocity.set(
        Double.isFinite(acceptedVisionVelocity.getX()) ? acceptedVisionVelocity.getX() : 0.0);
    pub_AcceptedVisionFieldYVelocity.set(
        Double.isFinite(acceptedVisionVelocity.getY()) ? acceptedVisionVelocity.getY() : 0.0);
    pub_AcceptedVisionVelocityTrust.set(visionVelocityTrust);
    pub_XAccel.set(filteredFieldX.getAccel());
    pub_YAccel.set(filteredFieldY.getAccel());
    pub_Alpha.set(Units.radiansToDegrees(filteredAlpha.getAccel()));
  }

  public Translation2d getAccel() {
    // The translational kinematic filters are field-relative because they use AprilTag-corrected
    // Pose2d position as the position measurement.
    return new Translation2d(filteredFieldX.getAccel(), filteredFieldY.getAccel());
  }

  public ChassisSpeeds getFilteredSpeeds() {
    Rotation2d poseRotation = getState().Pose.getRotation();
    double fieldVx = filteredFieldX.getVelocity();
    double fieldVy = filteredFieldY.getVelocity();

    return ChassisSpeeds.fromFieldRelativeSpeeds(
        fieldVx, fieldVy, filteredAlpha.getVelocity(), poseRotation);
  }

  public double getRobotRelativeAcceleration() {
    return filteredAlpha.getAccel();
  }

  public void addAcceptedVisionVelocityMeasurement(
      Translation2d fieldVelocity, double timestampSeconds, double quality) {
    if (!Double.isFinite(timestampSeconds)
        || !Double.isFinite(fieldVelocity.getX())
        || !Double.isFinite(fieldVelocity.getY())) {
      return;
    }

    double speed = fieldVelocity.getNorm();
    if (speed > VISION_VELOCITY_MAX_SPEED) {
      return;
    }

    double clampedQuality = MathUtil.clamp(quality, 0.0, 1.0);
    if (!hasAcceptedVisionVelocity) {
      acceptedVisionFieldVelocity = fieldVelocity;
      acceptedVisionVelocityTimestamp = timestampSeconds;
      acceptedVisionVelocityTrust = 0.0;
      hasAcceptedVisionVelocity = true;
      return;
    }

    if (timestampSeconds < acceptedVisionVelocityTimestamp - 1e-3) {
      return;
    }

    if (Math.abs(timestampSeconds - acceptedVisionVelocityTimestamp) < 1e-3) {
      acceptedVisionFieldVelocity =
          acceptedVisionFieldVelocity.interpolate(fieldVelocity, 0.5 * clampedQuality);
      return;
    }

    double dt = timestampSeconds - acceptedVisionVelocityTimestamp;
    boolean continuous = dt > 0.0 && dt <= VISION_VELOCITY_MAX_AGE_SECONDS;
    boolean consistent =
        acceptedVisionFieldVelocity.getDistance(fieldVelocity)
            <= VISION_VELOCITY_CONSISTENCY_TOLERANCE;

    if (continuous && consistent) {
      acceptedVisionVelocityTrust =
          Math.min(
              1.0,
              acceptedVisionVelocityTrust
                  + clampedQuality * dt / VISION_VELOCITY_TRUST_RAMP_SECONDS);
    } else {
      acceptedVisionVelocityTrust =
          Math.min(acceptedVisionVelocityTrust * 0.5, 0.15 * clampedQuality);
    }

    acceptedVisionFieldVelocity = fieldVelocity;
    acceptedVisionVelocityTimestamp = timestampSeconds;
  }

  private double getCurrentAcceptedVisionVelocityTrust(double now, double dt) {
    if (!hasAcceptedVisionVelocity) {
      return 0.0;
    }

    double age = now - acceptedVisionVelocityTimestamp;
    if (!Double.isFinite(age) || age > VISION_VELOCITY_MAX_AGE_SECONDS) {
      acceptedVisionVelocityTrust =
          Math.max(0.0, acceptedVisionVelocityTrust - dt / VISION_VELOCITY_TRUST_DECAY_SECONDS);
    }

    return acceptedVisionVelocityTrust;
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

  public boolean isBeached(double pitchThreshold) {
    double pitch = Math.abs(getPigeon2().getPitch().getValueAsDouble());
    double roll = Math.abs(getPigeon2().getRoll().getValueAsDouble());
    return pitch > pitchThreshold || roll > pitchThreshold;
  }
}
