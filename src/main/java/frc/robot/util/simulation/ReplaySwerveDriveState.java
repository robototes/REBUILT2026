package frc.robot.util.simulation;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.datalog.DataLogRecord;
import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Optional;

/**
 * Maintains a WPILib {@link SwerveDrivePoseEstimator} driven by deserialized DriveState log entries
 * and vision measurements. Produces a {@link SwerveDriveState} that can substitute for the CTRE
 * drivetrain's internal state during log replay.
 */
public class ReplaySwerveDriveState {

  private final SwerveDriveKinematics kinematics;
  private final SwerveDrivePoseEstimator poseEstimator;

  private SwerveModulePosition[] lastModulePositions;
  private SwerveModuleState[] lastModuleStates =
      new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
      };
  private SwerveModuleState[] lastModuleTargets =
      new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
      };
  private ChassisSpeeds lastSpeeds = new ChassisSpeeds();
  private Rotation2d lastHeading = Rotation2d.kZero;
  private double lastTimestamp = 0;
  private double lastOdometryPeriod = 0.02;
  private boolean hasReceivedFirstUpdate = false;

  /**
   * @param kinematics the swerve drive kinematics (module locations)
   */
  public ReplaySwerveDriveState(SwerveDriveKinematics kinematics) {
    this.kinematics = kinematics;
    lastModulePositions =
        new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
        };
    poseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics,
            Rotation2d.kZero,
            lastModulePositions,
            Pose2d.kZero,
            VecBuilder.fill(0.1, 0.1, 0.1), // odometry std devs
            VecBuilder.fill(0.9, 0.9, 0.9) // vision std devs (will be overridden per-call)
            );
  }

  /**
   * Update the pose estimator with a new set of module positions from the log. The heading is
   * extracted from the logged Pose2d (since the gyro angle was baked into the logged pose).
   *
   * @param modulePositions the logged module positions
   * @param heading the gyro heading at this log entry
   * @param timestampSeconds the FPGA timestamp in seconds
   */
  public void updateOdometry(
      SwerveModulePosition[] modulePositions, Rotation2d heading, double timestampSeconds) {
    if (!hasReceivedFirstUpdate) {
      // Reset the estimator to the first pose
      poseEstimator.resetPosition(heading, modulePositions, Pose2d.kZero);
      hasReceivedFirstUpdate = true;
    }
    poseEstimator.updateWithTime(timestampSeconds, heading, modulePositions);
    lastModulePositions = modulePositions;
    lastHeading = heading;
    lastTimestamp = timestampSeconds;
  }

  /** Update with logged Pose2d (used to set the initial pose and heading). */
  public void updatePose(Pose2d pose, double timestampSeconds) {
    if (!hasReceivedFirstUpdate) {
      poseEstimator.resetPosition(pose.getRotation(), lastModulePositions, pose);
      hasReceivedFirstUpdate = true;
    }
    lastHeading = pose.getRotation();
    lastTimestamp = timestampSeconds;
  }

  /** Update with logged ChassisSpeeds. */
  public void updateSpeeds(ChassisSpeeds speeds) {
    lastSpeeds = speeds;
  }

  /** Update with logged module states. */
  public void updateModuleStates(SwerveModuleState[] states) {
    lastModuleStates = states;
  }

  /** Update with logged module targets. */
  public void updateModuleTargets(SwerveModuleState[] targets) {
    lastModuleTargets = targets;
  }

  /** Update with logged odometry period. */
  public void updateOdometryPeriod(double period) {
    lastOdometryPeriod = period;
  }

  /**
   * Add a vision measurement to the pose estimator.
   *
   * @param visionPose the vision-measured pose
   * @param timestampSeconds the timestamp of the vision measurement in seconds
   * @param stdDevs the standard deviations for this measurement
   */
  public void addVisionMeasurement(
      Pose2d visionPose, double timestampSeconds, Matrix<N3, N1> stdDevs) {
    poseEstimator.addVisionMeasurement(visionPose, timestampSeconds, stdDevs);
  }

  /** Add a vision measurement with default std devs. */
  public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds) {
    poseEstimator.addVisionMeasurement(visionPose, timestampSeconds);
  }

  /** Sample the estimated pose at a given timestamp. */
  public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
    return poseEstimator.sampleAt(timestampSeconds);
  }

  /** Build a SwerveDriveState reflecting the current replay state. */
  public SwerveDriveState toSwerveDriveState() {
    var state = new SwerveDriveState();
    state.Pose = poseEstimator.getEstimatedPosition();
    state.Speeds = lastSpeeds;
    state.ModuleStates = lastModuleStates;
    state.ModuleTargets = lastModuleTargets;
    state.ModulePositions = lastModulePositions;
    state.Timestamp = lastTimestamp;
    state.OdometryPeriod = lastOdometryPeriod;
    state.SuccessfulDaqs = 0;
    state.FailedDaqs = 0;
    return state;
  }

  // ---- Static deserialization helpers ----

  /** Deserialize a Pose2d from a wpilog struct record's raw bytes. */
  public static Pose2d deserializePose2d(DataLogRecord record) {
    ByteBuffer bb = ByteBuffer.wrap(record.getRaw());
    bb.order(ByteOrder.LITTLE_ENDIAN);
    return Pose2d.struct.unpack(bb);
  }

  /** Deserialize a ChassisSpeeds from a wpilog struct record's raw bytes. */
  public static ChassisSpeeds deserializeChassisSpeeds(DataLogRecord record) {
    ByteBuffer bb = ByteBuffer.wrap(record.getRaw());
    bb.order(ByteOrder.LITTLE_ENDIAN);
    return ChassisSpeeds.struct.unpack(bb);
  }

  /** Deserialize a SwerveModulePosition[] from a wpilog struct array record's raw bytes. */
  public static SwerveModulePosition[] deserializeModulePositions(DataLogRecord record) {
    ByteBuffer bb = ByteBuffer.wrap(record.getRaw());
    bb.order(ByteOrder.LITTLE_ENDIAN);
    int count = bb.remaining() / SwerveModulePosition.struct.getSize();
    return Struct.unpackArray(bb, count, SwerveModulePosition.struct);
  }

  /** Deserialize a SwerveModuleState[] from a wpilog struct array record's raw bytes. */
  public static SwerveModuleState[] deserializeModuleStates(DataLogRecord record) {
    ByteBuffer bb = ByteBuffer.wrap(record.getRaw());
    bb.order(ByteOrder.LITTLE_ENDIAN);
    int count = bb.remaining() / SwerveModuleState.struct.getSize();
    return Struct.unpackArray(bb, count, SwerveModuleState.struct);
  }
}
