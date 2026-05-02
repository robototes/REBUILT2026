package frc.robot.subsystems.drivebase;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
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
import frc.robot.util.tuning.NtTunableDouble;
import java.util.function.Supplier;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily
 * be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
  /** Period for the fast prediction filter update loop, in seconds. */
  public static final double PREDICTION_UPDATE_PERIOD_SECONDS = 0.01;

  /** Frequency for prediction-related CTRE status signals, in hertz. */
  public static final double PREDICTION_UPDATE_FREQUENCY_HZ =
      1.0 / PREDICTION_UPDATE_PERIOD_SECONDS;

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
  private static final double VISION_VELOCITY_MAX_SPEED = 6.5;
  private static final double FILTERED_PREDICTION_POSE_MAX_DIVERGENCE = 0.75;
  private static final double STATIONARY_WHEEL_SPEED_MPS = 0.08;
  private static final double STATIONARY_VISION_SPEED_MPS = 0.12;
  private static final double STATIONARY_OMEGA_RAD_PER_SEC = 0.08;
  private static final double STATIONARY_ACCEL_MPS2 = 0.75;
  private static final double STATIONARY_SETTLE_SECONDS = 0.06;
  private static final double STATIONARY_RESET_MAX_POSE_ERROR = 0.30;
  private static final double SOTM_BLEND_EPSILON = 1e-4;
  private static final double SOTM_MAX_PLAUSIBLE_SPEED_MPS = 6.5;
  private static final double SOTM_MAX_PLAUSIBLE_ACCEL_MPS2 = 30.0;
  private static final double SOTM_MAX_VELOCITY_CORRECTION_MPS = 5.5;
  private static final double SOTM_MAX_FILTER_VISION_ERROR_MPS = 1.25;
  private static final double MIN_DERIVED_ACCEL_DT = 1e-4;
  private static final double MAX_DERIVED_ACCEL_DT = kMaxFilterDt;
  private static final double SOTM_WHEEL_ACCEL_FILTER_ALPHA_PER_20MS = 0.9;
  private static final double SOTM_WHEEL_ACCEL_FILTER_PERIOD_SECONDS = 0.02;
  private static final NtTunableDouble SOTM_ACCEL_DIVERGENCE_START =
      new NtTunableDouble("/AutoAim/sotmAccelDivergenceStart", 2.0);
  private static final NtTunableDouble SOTM_VISION_VELOCITY_DIVERGENCE_START =
      new NtTunableDouble("/AutoAim/sotmVisionVelocityDivergenceStart", 0.9);
  private static final NtTunableDouble SOTM_VISION_VELOCITY_DIVERGENCE_FULL =
      new NtTunableDouble("/AutoAim/sotmVisionVelocityDivergenceFull", 2.5);
  private static final NtTunableDouble SOTM_VISION_VELOCITY_DIVERGENCE_MIN_SECONDS =
      new NtTunableDouble("/AutoAim/sotmVisionVelocityDivergenceMinSeconds", 0.06);
  private static final NtTunableDouble SOTM_MIN_VISION_VELOCITY_TRUST =
      new NtTunableDouble("/AutoAim/sotmMinVisionVelocityTrust", 0.25);
  private static final NtTunableDouble SOTM_BLEND_RISE_PER_SECOND =
      new NtTunableDouble("/AutoAim/sotmBlendRisePerSecond", 20.0);
  private static final NtTunableDouble SOTM_BLEND_FALL_PER_SECOND =
      new NtTunableDouble("/AutoAim/sotmBlendFallPerSecond", 10.0);

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
          PREDICTION_UPDATE_PERIOD_SECONDS);
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
          PREDICTION_UPDATE_PERIOD_SECONDS);
  private KinematicFilter filteredAlpha =
      new KinematicFilter(0.005, 0.5, 0.001, PREDICTION_UPDATE_PERIOD_SECONDS);

  private final double nominalDt = PREDICTION_UPDATE_PERIOD_SECONDS;
  private Pigeon2 pigeon;
  private Translation2d lastFieldVelocity = new Translation2d();
  private double lastWheelOmegaForAccel = 0.0;
  private boolean hasWheelAccelBaseline = false;
  private Translation2d filteredWheelFieldAccel = new Translation2d();
  private double filteredWheelAlpha = 0.0;
  private boolean hasFilteredWheelAccel = false;
  private double lastPeriodicTime = 0;
  private boolean predictionFilterRunsInPeriodic = true;
  private double stationaryPredictionTime = 0.0;
  private double simAccelX = 0;
  private double simAccelY = 0;
  private Translation2d sotmPredictionFieldVelocity = new Translation2d();
  private Translation2d sotmPredictionFieldAcceleration = new Translation2d();
  private double sotmPredictionOmega = 0.0;
  private double sotmPredictionAlpha = 0.0;
  private double sotmExternalCorrectionBlend = 0.0;
  private double sotmAccelDisagreement = 0.0;
  private double sotmAccelDivergenceTime = 0.0;
  private double sotmVisionVelocityDisagreement = 0.0;
  private double sotmVisionVelocityDivergenceTime = 0.0;
  private boolean sotmPredictionInitialized = false;
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
  private DoublePublisher pub_FilteredFieldXVelocity;
  private DoublePublisher pub_FilteredFieldYVelocity;
  private DoublePublisher pub_SotmRawFieldXVelocity;
  private DoublePublisher pub_SotmRawFieldYVelocity;
  private DoublePublisher pub_SotmPredictionFieldXVelocity;
  private DoublePublisher pub_SotmPredictionFieldYVelocity;
  private DoublePublisher pub_SotmExternalCorrectionBlend;
  private DoublePublisher pub_SotmAccelDisagreement;
  private DoublePublisher pub_SotmAccelDivergenceTime;
  private DoublePublisher pub_SotmVisionVelocityDisagreement;

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
    BaseStatusSignal.setUpdateFrequencyForAll(
        PREDICTION_UPDATE_FREQUENCY_HZ, ss_XAccel, ss_YAccel, ss_Omega);

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
    pub_FilteredFieldXVelocity =
        nt.getDoubleTopic("/DriveState/Velocities/filteredFieldVelocityX").publish();
    pub_FilteredFieldYVelocity =
        nt.getDoubleTopic("/DriveState/Velocities/filteredFieldVelocityY").publish();
    pub_SotmRawFieldXVelocity =
        nt.getDoubleTopic("/DriveState/Velocities/sotmRawFieldVelocityX").publish();
    pub_SotmRawFieldYVelocity =
        nt.getDoubleTopic("/DriveState/Velocities/sotmRawFieldVelocityY").publish();
    pub_SotmPredictionFieldXVelocity =
        nt.getDoubleTopic("/DriveState/Velocities/sotmPredictionFieldVelocityX").publish();
    pub_SotmPredictionFieldYVelocity =
        nt.getDoubleTopic("/DriveState/Velocities/sotmPredictionFieldVelocityY").publish();
    pub_SotmExternalCorrectionBlend =
        nt.getDoubleTopic("/DriveState/Velocities/sotmExternalCorrectionBlend").publish();
    pub_SotmAccelDisagreement =
        nt.getDoubleTopic("/DriveState/Velocities/sotmAccelDisagreement").publish();
    pub_SotmAccelDivergenceTime =
        nt.getDoubleTopic("/DriveState/Velocities/sotmAccelDivergenceTime").publish();
    pub_SotmVisionVelocityDisagreement =
        nt.getDoubleTopic("/DriveState/Velocities/sotmVisionVelocityDisagreement").publish();
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
    pub_FilteredFieldXVelocity.set(0.0);
    pub_FilteredFieldYVelocity.set(0.0);
    pub_SotmRawFieldXVelocity.set(0.0);
    pub_SotmRawFieldYVelocity.set(0.0);
    pub_SotmPredictionFieldXVelocity.set(0.0);
    pub_SotmPredictionFieldYVelocity.set(0.0);
    pub_SotmExternalCorrectionBlend.set(0.0);
    pub_SotmAccelDisagreement.set(0.0);
    pub_SotmAccelDivergenceTime.set(0.0);
    pub_SotmVisionVelocityDisagreement.set(0.0);
  }

  /**
   * Returns a command that applies the specified control request to this swerve drivetrain.
   *
   * @param requestSupplier Function returning the request to apply
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
    if (predictionFilterRunsInPeriodic) {
      updatePredictionFilter();
    }
  }

  /**
   * Selects whether the prediction filter is updated from this subsystem's normal 20 ms periodic.
   *
   * @param enabled true to use subsystem periodic updates, false when a faster outer loop owns them
   */
  public void setPredictionFilterRunsInPeriodic(boolean enabled) {
    predictionFilterRunsInPeriodic = enabled;
  }

  /** Updates the fused prediction filters from the latest drivetrain, IMU, and vision inputs. */
  public void updatePredictionFilter() {
    double now = Timer.getFPGATimestamp();
    double measuredDt = (lastPeriodicTime > 0) ? (now - lastPeriodicTime) : nominalDt;
    lastPeriodicTime = now;
    boolean measuredDtValid = Double.isFinite(measuredDt) && measuredDt > 0.0;
    double dt = measuredDtValid ? Math.min(measuredDt, kMaxFilterDt) : nominalDt;
    boolean derivedAccelDtValid =
        measuredDtValid && measuredDt >= MIN_DERIVED_ACCEL_DT && measuredDt <= MAX_DERIVED_ACCEL_DT;

    var imuRefreshStatus = BaseStatusSignal.refreshAll(ss_XAccel, ss_YAccel, ss_Omega);
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
    Translation2d wheelFieldAccel = new Translation2d();
    double wheelAlpha = 0.0;
    if (hasWheelAccelBaseline && derivedAccelDtValid) {
      wheelFieldAccel =
          new Translation2d(
              (fieldVelocity.getX() - lastFieldVelocity.getX()) / measuredDt,
              (fieldVelocity.getY() - lastFieldVelocity.getY()) / measuredDt);
      wheelAlpha = (robotSpeeds.omegaRadiansPerSecond - lastWheelOmegaForAccel) / measuredDt;
    }
    lastFieldVelocity = fieldVelocity;
    lastWheelOmegaForAccel = robotSpeeds.omegaRadiansPerSecond;
    hasWheelAccelBaseline = true;

    if (RobotBase.isSimulation()) {
      simAccelX = wheelFieldAccel.getX();
      simAccelY = wheelFieldAccel.getY();
    }
    if (derivedAccelDtValid) {
      updateFilteredWheelAcceleration(wheelFieldAccel, wheelAlpha, dt);
    } else {
      resetFilteredWheelAcceleration();
    }
    Translation2d wheelAccelForDisagreement =
        derivedAccelDtValid && hasFilteredWheelAccel
            ? filteredWheelFieldAccel
            : new Translation2d(Double.NaN, Double.NaN);

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
    Translation2d filterInputFieldAccel = sanitizeSotmAccel(fieldAccel);
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
        filterInputFieldAccel.getX(),
        dt);
    filteredFieldY.update(
        currentPose.getY(),
        fieldVelocity.getY(),
        acceptedVisionVelocity.getY(),
        visionVelocityTrust,
        filterInputFieldAccel.getY(),
        dt);
    filteredAlpha.update(currentOmega, dt);

    settlePredictionFilterIfStationary(
        currentPose,
        fieldVelocity,
        acceptedVisionVelocity,
        visionVelocityTrust,
        filterInputFieldAccel,
        currentOmega,
        dt);
    updateSotmPredictionSource(
        currentPose,
        fieldVelocity,
        robotSpeeds.omegaRadiansPerSecond,
        wheelAccelForDisagreement,
        derivedAccelDtValid ? wheelAlpha : Double.NaN,
        filteredWheelFieldAccel,
        filteredWheelAlpha,
        filterInputFieldAccel,
        acceptedVisionVelocity,
        visionVelocityTrust,
        dt);
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
    pub_FilteredFieldXVelocity.set(filteredFieldX.getVelocity());
    pub_FilteredFieldYVelocity.set(filteredFieldY.getVelocity());
  }

  private void updateFilteredWheelAcceleration(
      Translation2d wheelFieldAccel, double wheelAlpha, double dt) {
    if (!hasFilteredWheelAccel || !isFinite(wheelFieldAccel) || !Double.isFinite(wheelAlpha)) {
      filteredWheelFieldAccel = isFinite(wheelFieldAccel) ? wheelFieldAccel : new Translation2d();
      filteredWheelAlpha = Double.isFinite(wheelAlpha) ? wheelAlpha : 0.0;
      hasFilteredWheelAccel = true;
      return;
    }

    double alpha =
        Math.pow(
            SOTM_WHEEL_ACCEL_FILTER_ALPHA_PER_20MS,
            Math.max(0.0, dt) / SOTM_WHEEL_ACCEL_FILTER_PERIOD_SECONDS);
    double newSampleWeight = 1.0 - MathUtil.clamp(alpha, 0.0, 1.0);
    filteredWheelFieldAccel = filteredWheelFieldAccel.interpolate(wheelFieldAccel, newSampleWeight);
    filteredWheelAlpha += newSampleWeight * (wheelAlpha - filteredWheelAlpha);
  }

  private void resetFilteredWheelAcceleration() {
    filteredWheelFieldAccel = new Translation2d();
    filteredWheelAlpha = 0.0;
    hasFilteredWheelAccel = true;
  }

  private void updateSotmPredictionSource(
      Pose2d currentPose,
      Translation2d rawFieldVelocity,
      double rawOmega,
      Translation2d rawWheelFieldAccel,
      double rawWheelAlpha,
      Translation2d predictionWheelFieldAccel,
      double predictionWheelAlpha,
      Translation2d imuFieldAccel,
      Translation2d acceptedVisionVelocity,
      double visionVelocityTrust,
      double dt) {
    Translation2d filteredFieldVelocity =
        filteredFieldX.isInitialized() && filteredFieldY.isInitialized()
            ? new Translation2d(filteredFieldX.getVelocity(), filteredFieldY.getVelocity())
            : rawFieldVelocity;
    if (!isFinite(filteredFieldVelocity)) {
      filteredFieldVelocity = rawFieldVelocity;
    }

    Translation2d filteredFieldAccel =
        filteredFieldX.isInitialized() && filteredFieldY.isInitialized()
            ? new Translation2d(filteredFieldX.getAccel(), filteredFieldY.getAccel())
            : predictionWheelFieldAccel;
    if (!isFinite(filteredFieldAccel)) {
      filteredFieldAccel = predictionWheelFieldAccel;
    }

    boolean filteredVelocityPlausible = isSotmVelocityPlausible(filteredFieldVelocity);
    boolean filteredAccelPlausible = isSotmAccelPlausible(filteredFieldAccel);
    if (!filteredVelocityPlausible || !filteredAccelPlausible) {
      resetPredictionFilters(currentPose, rawFieldVelocity, predictionWheelFieldAccel, rawOmega);
      filteredFieldVelocity = rawFieldVelocity;
      filteredFieldAccel = predictionWheelFieldAccel;
      filteredVelocityPlausible = isSotmVelocityPlausible(filteredFieldVelocity);
      filteredAccelPlausible = isSotmAccelPlausible(filteredFieldAccel);
    }

    sotmAccelDisagreement = 0.0;
    if (isFinite(rawWheelFieldAccel) && isFinite(imuFieldAccel)) {
      sotmAccelDisagreement = rawWheelFieldAccel.minus(imuFieldAccel).getNorm();
    }
    double accelDivergenceStart = SOTM_ACCEL_DIVERGENCE_START.get();
    if (sotmAccelDisagreement >= accelDivergenceStart) {
      sotmAccelDivergenceTime += Math.max(0.0, dt);
    } else {
      sotmAccelDivergenceTime = 0.0;
    }

    sotmVisionVelocityDisagreement = 0.0;
    double visionScore = 0.0;
    if (visionVelocityTrust >= SOTM_MIN_VISION_VELOCITY_TRUST.get()
        && isFinite(acceptedVisionVelocity)) {
      sotmVisionVelocityDisagreement = rawFieldVelocity.getDistance(acceptedVisionVelocity);
      double visionDivergenceStart = SOTM_VISION_VELOCITY_DIVERGENCE_START.get();
      if (sotmVisionVelocityDisagreement >= visionDivergenceStart) {
        sotmVisionVelocityDivergenceTime += Math.max(0.0, dt);
      } else {
        sotmVisionVelocityDivergenceTime = 0.0;
      }
      visionScore =
          sotmVisionVelocityDivergenceTime
                  >= Math.max(0.0, SOTM_VISION_VELOCITY_DIVERGENCE_MIN_SECONDS.get())
              ? scoreFromThresholds(
                  sotmVisionVelocityDisagreement,
                  visionDivergenceStart,
                  SOTM_VISION_VELOCITY_DIVERGENCE_FULL.get())
              : 0.0;
    } else {
      sotmVisionVelocityDivergenceTime = 0.0;
    }

    Translation2d correctionFieldVelocity = rawFieldVelocity;
    Translation2d correctionFieldAccel = predictionWheelFieldAccel;
    double targetBlend = 0.0;
    // Accel disagreement is useful for diagnostics, but it does not identify absolute chassis
    // velocity. Only sustained trusted vision velocity is allowed to pull SOTM away from wheels.
    if (visionScore > 0.0 && isSotmVelocityPlausible(acceptedVisionVelocity)) {
      correctionFieldVelocity = limitVelocityCorrection(rawFieldVelocity, acceptedVisionVelocity);
      targetBlend = visionScore;
      if (filteredAccelPlausible
          && filteredVelocityPlausible
          && filteredFieldVelocity.getDistance(acceptedVisionVelocity)
              <= SOTM_MAX_FILTER_VISION_ERROR_MPS) {
        correctionFieldAccel = filteredFieldAccel;
      }
    }

    double rate =
        targetBlend > sotmExternalCorrectionBlend
            ? SOTM_BLEND_RISE_PER_SECOND.get()
            : SOTM_BLEND_FALL_PER_SECOND.get();
    sotmExternalCorrectionBlend =
        stepToward(
            sotmExternalCorrectionBlend, targetBlend, Math.max(0.0, rate) * Math.max(0.0, dt));

    if (sotmExternalCorrectionBlend <= SOTM_BLEND_EPSILON) {
      sotmPredictionFieldVelocity = rawFieldVelocity;
      sotmPredictionFieldAcceleration = predictionWheelFieldAccel;
      sotmPredictionOmega = rawOmega;
      sotmPredictionAlpha = predictionWheelAlpha;
    } else {
      sotmPredictionFieldVelocity =
          rawFieldVelocity.interpolate(correctionFieldVelocity, sotmExternalCorrectionBlend);
      sotmPredictionFieldAcceleration =
          predictionWheelFieldAccel.interpolate(correctionFieldAccel, sotmExternalCorrectionBlend);
      sotmPredictionOmega = rawOmega;
      sotmPredictionAlpha = predictionWheelAlpha;
    }

    if (!isFinite(sotmPredictionFieldVelocity)) {
      sotmPredictionFieldVelocity = rawFieldVelocity;
    }
    if (!isFinite(sotmPredictionFieldAcceleration)) {
      sotmPredictionFieldAcceleration = predictionWheelFieldAccel;
    }
    if (!Double.isFinite(sotmPredictionOmega)) {
      sotmPredictionOmega = rawOmega;
    }
    if (!Double.isFinite(sotmPredictionAlpha)) {
      sotmPredictionAlpha = predictionWheelAlpha;
    }

    sotmPredictionInitialized = true;
    pub_SotmRawFieldXVelocity.set(rawFieldVelocity.getX());
    pub_SotmRawFieldYVelocity.set(rawFieldVelocity.getY());
    pub_SotmPredictionFieldXVelocity.set(sotmPredictionFieldVelocity.getX());
    pub_SotmPredictionFieldYVelocity.set(sotmPredictionFieldVelocity.getY());
    pub_SotmExternalCorrectionBlend.set(sotmExternalCorrectionBlend);
    pub_SotmAccelDisagreement.set(sotmAccelDisagreement);
    pub_SotmAccelDivergenceTime.set(sotmAccelDivergenceTime);
    pub_SotmVisionVelocityDisagreement.set(sotmVisionVelocityDisagreement);
  }

  private static double scoreFromThresholds(double value, double start, double full) {
    if (!Double.isFinite(value) || !Double.isFinite(start) || !Double.isFinite(full)) {
      return 0.0;
    }
    if (full <= start) {
      return value >= start ? 1.0 : 0.0;
    }
    return MathUtil.clamp((value - start) / (full - start), 0.0, 1.0);
  }

  private static double stepToward(double current, double target, double maxStep) {
    if (!Double.isFinite(target)) {
      target = 0.0;
    }
    current = MathUtil.clamp(Double.isFinite(current) ? current : 0.0, 0.0, 1.0);
    target = MathUtil.clamp(target, 0.0, 1.0);
    if (target > current) {
      return Math.min(target, current + maxStep);
    }
    return Math.max(target, current - maxStep);
  }

  private static boolean isFinite(Translation2d value) {
    return value != null && Double.isFinite(value.getX()) && Double.isFinite(value.getY());
  }

  private static Translation2d sanitizeSotmAccel(Translation2d accel) {
    if (!isSotmAccelPlausible(accel)) {
      return new Translation2d(Double.NaN, Double.NaN);
    }
    return accel;
  }

  private static boolean isSotmVelocityPlausible(Translation2d velocity) {
    return isFinite(velocity) && velocity.getNorm() <= SOTM_MAX_PLAUSIBLE_SPEED_MPS;
  }

  private static boolean isSotmAccelPlausible(Translation2d accel) {
    return isFinite(accel) && accel.getNorm() <= SOTM_MAX_PLAUSIBLE_ACCEL_MPS2;
  }

  private static Translation2d limitVelocityCorrection(
      Translation2d rawFieldVelocity, Translation2d correctionTarget) {
    if (!isFinite(rawFieldVelocity) || !isFinite(correctionTarget)) {
      return rawFieldVelocity;
    }

    Translation2d correction = correctionTarget.minus(rawFieldVelocity);
    double correctionNorm = correction.getNorm();
    if (correctionNorm <= SOTM_MAX_VELOCITY_CORRECTION_MPS) {
      return correctionTarget;
    }

    return rawFieldVelocity.plus(
        correction.times(SOTM_MAX_VELOCITY_CORRECTION_MPS / correctionNorm));
  }

  private void resetPredictionFilters(
      Pose2d currentPose,
      Translation2d fieldVelocity,
      Translation2d fieldAccel,
      double omegaRadiansPerSecond) {
    Translation2d resetVelocity = isFinite(fieldVelocity) ? fieldVelocity : new Translation2d();
    Translation2d resetAccel = isSotmAccelPlausible(fieldAccel) ? fieldAccel : new Translation2d();
    filteredFieldX.reset(currentPose.getX(), resetVelocity.getX(), resetAccel.getX());
    filteredFieldY.reset(currentPose.getY(), resetVelocity.getY(), resetAccel.getY());
    filteredAlpha.reset(Double.isFinite(omegaRadiansPerSecond) ? omegaRadiansPerSecond : 0.0);
  }

  private void settlePredictionFilterIfStationary(
      Pose2d currentPose,
      Translation2d fieldVelocity,
      Translation2d acceptedVisionVelocity,
      double visionVelocityTrust,
      Translation2d fieldAccel,
      double currentOmega,
      double dt) {
    boolean wheelStationary = fieldVelocity.getNorm() <= STATIONARY_WHEEL_SPEED_MPS;
    boolean omegaStationary =
        Double.isFinite(currentOmega) && Math.abs(currentOmega) <= STATIONARY_OMEGA_RAD_PER_SEC;
    boolean accelStationary =
        Double.isFinite(fieldAccel.getX())
            && Double.isFinite(fieldAccel.getY())
            && fieldAccel.getNorm() <= STATIONARY_ACCEL_MPS2;
    boolean visionStationary =
        visionVelocityTrust <= 0.0
            || acceptedVisionVelocity.getNorm() <= STATIONARY_VISION_SPEED_MPS;
    boolean filterCloseToPose =
        new Translation2d(filteredFieldX.getPosition(), filteredFieldY.getPosition())
                .getDistance(currentPose.getTranslation())
            <= STATIONARY_RESET_MAX_POSE_ERROR;

    if (wheelStationary
        && omegaStationary
        && accelStationary
        && visionStationary
        && filterCloseToPose) {
      stationaryPredictionTime += dt;
    } else {
      stationaryPredictionTime = 0.0;
      return;
    }

    if (stationaryPredictionTime >= STATIONARY_SETTLE_SECONDS) {
      filteredFieldX.reset(currentPose.getX(), 0.0, 0.0);
      filteredFieldY.reset(currentPose.getY(), 0.0, 0.0);
      filteredAlpha.reset(0.0);
    }
  }

  public Translation2d getAccel() {
    // The translational kinematic filters are field-relative because they use AprilTag-corrected
    // Pose2d position as the position measurement.
    return new Translation2d(filteredFieldX.getAccel(), filteredFieldY.getAccel());
  }

  public Pose2d getFilteredPoseForPrediction() {
    Pose2d currentPose = getState().Pose;
    if (!filteredFieldX.isInitialized() || !filteredFieldY.isInitialized()) {
      return currentPose;
    }

    double filteredX = filteredFieldX.getPosition();
    double filteredY = filteredFieldY.getPosition();
    if (!Double.isFinite(filteredX) || !Double.isFinite(filteredY)) {
      return currentPose;
    }

    Translation2d filteredTranslation = new Translation2d(filteredX, filteredY);
    if (filteredTranslation.getDistance(currentPose.getTranslation())
        > FILTERED_PREDICTION_POSE_MAX_DIVERGENCE) {
      return currentPose;
    }

    return new Pose2d(filteredTranslation, currentPose.getRotation());
  }

  /**
   * Returns the pose used by shoot-on-the-move prediction.
   *
   * <p>This is the drivetrain pose during normal driving. When external sensors indicate wheel
   * motion is untrustworthy, translation may blend toward the bounded kinematic filter pose using
   * the same correction blend as velocity.
   *
   * @return pose for launch prediction
   */
  public Pose2d getSotmPoseForPrediction() {
    Pose2d currentPose = getState().Pose;
    if (!sotmPredictionInitialized || sotmExternalCorrectionBlend <= SOTM_BLEND_EPSILON) {
      return currentPose;
    }

    Pose2d filteredPose = getFilteredPoseForPrediction();
    Translation2d blendedTranslation =
        currentPose
            .getTranslation()
            .interpolate(filteredPose.getTranslation(), sotmExternalCorrectionBlend);
    return new Pose2d(blendedTranslation, currentPose.getRotation());
  }

  public ChassisSpeeds getFilteredSpeeds() {
    Rotation2d poseRotation = getState().Pose.getRotation();
    double fieldVx = filteredFieldX.getVelocity();
    double fieldVy = filteredFieldY.getVelocity();

    return ChassisSpeeds.fromFieldRelativeSpeeds(
        fieldVx, fieldVy, filteredAlpha.getVelocity(), poseRotation);
  }

  /**
   * Returns the chassis speeds used by shoot-on-the-move prediction.
   *
   * <p>Normal driving returns raw drivetrain speeds. Sustained trusted-vision disagreement ramps in
   * a bounded correction target from the accepted vision velocity.
   *
   * @return robot-relative chassis speeds for launch prediction
   */
  public ChassisSpeeds getSotmSpeedsForPrediction() {
    var state = getState();
    if (!sotmPredictionInitialized || sotmExternalCorrectionBlend <= SOTM_BLEND_EPSILON) {
      return state.Speeds;
    }

    return ChassisSpeeds.fromFieldRelativeSpeeds(
        sotmPredictionFieldVelocity.getX(),
        sotmPredictionFieldVelocity.getY(),
        sotmPredictionOmega,
        state.Pose.getRotation());
  }

  /**
   * Returns the field-relative acceleration used by shoot-on-the-move phase-delay prediction.
   *
   * @return field-relative translational acceleration in m/s^2
   */
  public Translation2d getSotmFieldAccelerationForPrediction() {
    if (!sotmPredictionInitialized) {
      return new Translation2d();
    }
    return sotmPredictionFieldAcceleration;
  }

  /**
   * Returns the angular acceleration used by shoot-on-the-move phase-delay prediction.
   *
   * @return angular acceleration in rad/s^2
   */
  public double getSotmAngularAccelerationForPrediction() {
    return sotmPredictionInitialized ? sotmPredictionAlpha : 0.0;
  }

  /**
   * Returns how much external correction is blended into the raw drivetrain prediction.
   *
   * @return correction blend from 0.0 (raw drivetrain) to 1.0 (fused prediction)
   */
  public double getSotmExternalCorrectionBlend() {
    return sotmExternalCorrectionBlend;
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
