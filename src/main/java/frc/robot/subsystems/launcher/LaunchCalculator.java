package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import frc.robot.util.AllianceUtils;
import frc.robot.util.GetTargetFromPose;
import frc.robot.util.tuning.LauncherConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class LaunchCalculator {
  private static class Holder {
    private static final LaunchCalculator INSTANCE = new LaunchCalculator();
  }

  public static LaunchCalculator getInstance() {
    return Holder.INSTANCE;
  }

  // Cached variables
  private LaunchingParameters cachedParams;
  private Pose2d lastPose = new Pose2d();
  private double lastTurretOmega = 0;
  private double lastTimestamp = 0;

  // Hysteresis state for pose-diff activation.
  // Requires a higher speed to turn ON than to turn OFF, preventing rapid toggling at the
  // boundary and the stale-prevPose spike that would otherwise occur on the first frame after
  // starting to move. prevPose is explicitly reset to the current pose on activation so the
  // first diff frame always computes over a fresh baseline.
  private boolean poseBlendActive = false;
  private Pose2d prevPose = new Pose2d();
  private double prevTimestamp = 0;

  // Throttling constants
  private static final double MIN_DIST_TOLERANCE = Units.inchesToMeters(1); // Meters
  private static final double MIN_ROTATION_TOLERANCE = Units.degreesToRadians(0.5); // Radians
  private static final double MIN_VELOCITY_TOLERANCE = Units.inchesToMeters(0.5); // M/s
  private static final double MIN_OMEGA_TOLERANCE = 0.05; // Radians/s
  // Cache bust uses drivetrain's Kalman-filtered acceleration — already smooth, no need to
  // differentiate wheel speeds ourselves.
  private static final double MIN_ACCEL_TOLERANCE = 0.3; // M/s^2
  private static final double MIN_ANGULAR_ACCEL_TOLERANCE = 0.5; // Rad/s^2

  // Turret transform components — cached at startup to avoid repeated object allocation
  // per loop cycle.
  private static final double TURRET_TX;
  private static final double TURRET_TY;
  private static final Transform2d turretTransform = LauncherConstants.turretTransform();

  private static final double PHASE_DELAY = 0.02;
  private static final double ONE_HALF_PHASE_DELAY_SQUARED = 0.5 * PHASE_DELAY * PHASE_DELAY;
  private static final double CONVERGENCE_TOLERANCE = 0.001;
  private static final double STEP_SIZE = 0.01;
  private static final double MIN_SLOPE = 1e-4;
  private static final int NEWTON_METHOD_MAX_ITERATIONS = 5;
  private static final double VFF_DIST_TOLERANCE = 0.1;
  private static final double MIN_DISTANCE_TO_TARGET = 1e-4;

  // Pose-differentiation velocity blending.
  // Max blend weight at MIN_POSE_DT, fades to 0 at MAX_POSE_DT.
  // Set high because accepted poses are always correct — the dynamic dt scaling handles
  // the fast-movement spottiness by fading to wheel speeds as vision updates become sparse.
  private static final double POSE_VELOCITY_BLEND = 0.9;
  private static final double MIN_POSE_DT = 0.005; // seconds
  private static final double MAX_POSE_DT = 0.05; // seconds

  // Hysteresis thresholds for pose-diff activation.
  // Must reach ON threshold to activate; must drop below OFF threshold to deactivate.
  // The gap prevents rapid toggling and the stale-baseline spike on start/stop.
  private static final double MOVING_SPEED_THRESHOLD_ON = 0.15; // m/s
  private static final double MOVING_SPEED_THRESHOLD_OFF = 0.05; // m/s

  // Trench stuff
  private static final AprilTagFieldLayout field = AllianceUtils.FIELD_LAYOUT;
  private static final double TURRET_TO_TRENCH_TOLERANCE_X = Units.inchesToMeters(12);
  private static final double TURRET_TO_TRENCH_TOLERANCE_Y = Units.inchesToMeters(24.97);
  private static final double TRENCH_LOOKAHEAD = 0.5; // seconds
  private static final int TRENCH_LOOKAHEAD_SAMPLES = 10;
  private static final List<Pose2d> trenchTags = new ArrayList<>();
  private static final int[] tags = {1, 6, 7, 12, 17, 22, 23, 28};
  private static final double TURRET_TO_UNDERCLIMB_TOLERANCE_X = Units.inchesToMeters(47.0 / 2);
  private static final double TURRET_TO_UNDERCLIMB_TOLERANCE_Y = Units.inchesToMeters(11.38);
  private static final List<Pose2d> underclimbTags = new ArrayList<>();
  private static final int[] underclimbTagIds = {15, 31};

  public record LaunchingParameters(
      double targetHood,
      Rotation2d targetTurret,
      double targetFlywheels,
      double targetTurretFeedforward,
      Pose2d turretPose) {}

  static {
    TURRET_TX = LauncherConstants.turretTransform().getTranslation().getX();
    TURRET_TY = LauncherConstants.turretTransform().getTranslation().getY();

    for (int tag : tags) {
      Optional<Pose3d> t = field.getTagPose(tag);
      if (t.isPresent()) {
        trenchTags.add(t.get().toPose2d());
      } else {
        edu.wpi.first.wpilibj.DriverStation.reportWarning(
            "Tag " + tag + " not found in launch calculator", false);
      }
    }
    for (int tag : underclimbTagIds) {
      Optional<Pose3d> t = field.getTagPose(tag);
      if (t.isPresent()) {
        underclimbTags.add(t.get().toPose2d());
      } else {
        edu.wpi.first.wpilibj.DriverStation.reportWarning(
            "Tag " + tag + " not found in launch calculator", false);
      }
    }
  }

  // ------ MAIN LOGIC ------ //

  /**
   * Returns the cached LaunchingParameters, recomputing only when the robot has moved beyond the
   * specified thresholds or is accelerating significantly.
   *
   * <p>Acceleration is sourced directly from the drivetrain's Kalman-filtered IMU+odometry fusion
   * (KinematicFilterInfused), which is already smooth and field-relative. This replaces the
   * previous wheel-speed differentiation approach and eliminates the associated noise and lag.
   *
   * <p>Differentiation state (prevPose, prevTimestamp) is updated at the END of this method in all
   * code paths so that calculate() always sees a real non-zero poseDt, and cache hits don't let the
   * delta go stale.
   *
   * @param drivetrain the drivebase's CommandSwerveDrivetrain object
   * @param turretSubsystem the turretSubsystem object
   * @return LaunchingParameters record holding all the target values.
   */
  public LaunchingParameters getParameters(
      CommandSwerveDrivetrain drivetrain, TurretSubsystem turretSubsystem) {
    SwerveDriveState driveState = drivetrain.getState();
    double timestamp = driveState.Timestamp;

    // Guard against re-entrant calls within the same robot loop cycle.
    if (cachedParams != null && timestamp == lastTimestamp) {
      return cachedParams;
    }

    Pose2d currentPose = driveState.Pose;
    ChassisSpeeds currentSpeeds = driveState.Speeds;
    double currentTurretOmega = turretSubsystem.getOmega();

    // Pull Kalman-filtered acceleration from the drivetrain.
    // getAccel() is already field-relative (robot-relative IMU values rotated by pose rotation).
    // getRobotRelativeAcceleration() is angular acceleration in rad/s² (frame-independent in 2D).
    Translation2d accel = drivetrain.getAccel();
    double ax = accel.getX();
    double ay = accel.getY();
    double alpha = drivetrain.getRobotRelativeAcceleration();

    boolean hasNotMovedSignificantly =
        Math.abs(currentPose.getTranslation().getDistance(lastPose.getTranslation()))
            <= MIN_DIST_TOLERANCE;
    boolean hasNotRotatedSignificantly =
        Math.abs(currentPose.getRotation().getRadians() - lastPose.getRotation().getRadians())
            <= MIN_ROTATION_TOLERANCE;
    boolean isNotMovingFastEnough =
        Math.abs(currentSpeeds.vxMetersPerSecond) <= MIN_VELOCITY_TOLERANCE
            && Math.abs(currentSpeeds.vyMetersPerSecond) <= MIN_VELOCITY_TOLERANCE
            && Math.abs(currentSpeeds.omegaRadiansPerSecond) <= MIN_ROTATION_TOLERANCE;
    boolean isTurretOmegaStable =
        Math.abs(currentTurretOmega - lastTurretOmega) <= MIN_OMEGA_TOLERANCE;
    boolean isNotAcceleratingSignificantly =
        Math.hypot(ax, ay) <= MIN_ACCEL_TOLERANCE && Math.abs(alpha) <= MIN_ANGULAR_ACCEL_TOLERANCE;

    if (hasNotMovedSignificantly
        && hasNotRotatedSignificantly
        && isNotMovingFastEnough
        && isTurretOmegaStable
        && isNotAcceleratingSignificantly
        && cachedParams != null) {
      if (cachedParams.targetTurretFeedforward != 0) {
        cachedParams =
            new LaunchingParameters(
                cachedParams.targetHood,
                cachedParams.targetTurret,
                cachedParams.targetFlywheels,
                0,
                cachedParams.turretPose);
      }
      // Update differentiation state on cache hit so poseDt stays valid next cycle.
      prevPose = currentPose;
      prevTimestamp = timestamp;
      lastTimestamp = timestamp;
      return cachedParams;
    }

    lastPose = currentPose;
    lastTurretOmega = currentTurretOmega;

    cachedParams = calculate(driveState, turretSubsystem, ax, ay, alpha);

    // Update differentiation state AFTER calculate() so calculate() sees the previous cycle's
    // state (real non-zero poseDt), and the next cycle uses this cycle as its baseline.
    prevPose = currentPose;
    prevTimestamp = timestamp;
    lastTimestamp = timestamp;

    return cachedParams;
  }

  /**
   * Computes a fresh set of LaunchingParameters.
   *
   * <p>1. ACCELERATION COMPENSATION (phase delay only): Field-relative acceleration from the
   * drivetrain's IMU+odometry Kalman filter is used to predict robot state at the end of
   * PHASE_DELAY via x = x0 + v*t + 0.5*a*t^2, computed directly in the field frame. The velocity at
   * end of phase delay (launchSpeeds) is treated as constant for the Newton loop — post-launch
   * robot acceleration does not affect ball trajectory.
   *
   * <p>2. DEFENSE/PUSH COMPENSATION: Pose-differentiated velocity blended with wheel speeds behind
   * a hysteresis gate. Requires MOVING_SPEED_THRESHOLD_ON to activate and drops off at
   * MOVING_SPEED_THRESHOLD_OFF. prevPose is reset on activation so the first diff frame computes
   * over a fresh baseline rather than a stale stationary pose.
   *
   * <p>3. TURRET VELOCITY: Computed explicitly in the field frame by rotating the turret offset
   * vector by the predicted robot angle, then adding the tangential velocity from robot rotation.
   * Uses the predicted angle (post-phase-delay) rather than the current angle for accuracy.
   *
   * @param driveState the drivebase's SwerveDriveState
   * @param turretSubsystem the turretSubsystem object
   * @param ax field-relative X acceleration in m/s² (from drivetrain Kalman filter)
   * @param ay field-relative Y acceleration in m/s²
   * @param alpha robot angular acceleration in rad/s²
   * @return LaunchingParameters record holding all the target values
   */
  public LaunchingParameters calculate(
      SwerveDriveState driveState,
      TurretSubsystem turretSubsystem,
      double ax,
      double ay,
      double alpha) {

    Pose2d currentPose = driveState.Pose;
    ChassisSpeeds chassisSpeeds = driveState.Speeds;
    double timestamp = driveState.Timestamp;

    // --- DEFENSE COMPENSATION: Pose-differentiated velocity with hysteresis gate ---
    // Gate prevents toggling at the boundary and the stale-baseline spike on start.
    // When activating, prevPose/prevTimestamp are reset so the first diff frame is clean.
    double wheelSpeedMagnitude =
        Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);

    if (!poseBlendActive && wheelSpeedMagnitude > MOVING_SPEED_THRESHOLD_ON) {
      poseBlendActive = true;
      prevPose = currentPose;
      prevTimestamp = timestamp;
    } else if (poseBlendActive && wheelSpeedMagnitude < MOVING_SPEED_THRESHOLD_OFF) {
      poseBlendActive = false;
    }

    // Convert wheel speeds to field-relative for blending with field-relative pose-diff velocity.
    ChassisSpeeds fieldWheelSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeeds, currentPose.getRotation());
    ChassisSpeeds effectiveFieldSpeeds = fieldWheelSpeeds;

    double poseDt = timestamp - prevTimestamp;
    if (poseBlendActive && poseDt > MIN_POSE_DT && poseDt < MAX_POSE_DT) {
      Twist2d twist = prevPose.log(currentPose);
      ChassisSpeeds poseDerivedSpeeds =
          new ChassisSpeeds(twist.dx / poseDt, twist.dy / poseDt, twist.dtheta / poseDt);
      // Scale blend: full weight at MIN_POSE_DT, 0 at MAX_POSE_DT.
      // Sparse vision updates produce large poseDt and fade naturally to wheel speeds.
      double blendAlpha =
          POSE_VELOCITY_BLEND * (1.0 - (poseDt - MIN_POSE_DT) / (MAX_POSE_DT - MIN_POSE_DT));
      effectiveFieldSpeeds =
          new ChassisSpeeds(
              blendAlpha * poseDerivedSpeeds.vxMetersPerSecond
                  + (1.0 - blendAlpha) * fieldWheelSpeeds.vxMetersPerSecond,
              blendAlpha * poseDerivedSpeeds.vyMetersPerSecond
                  + (1.0 - blendAlpha) * fieldWheelSpeeds.vyMetersPerSecond,
              blendAlpha * poseDerivedSpeeds.omegaRadiansPerSecond
                  + (1.0 - blendAlpha) * fieldWheelSpeeds.omegaRadiansPerSecond);
    }

    // --- PHASE DELAY PREDICTION (second-order, field frame) ---
    // x = x0 + v*t + 0.5*a*t^2, computed directly in the field frame.
    // This avoids Twist2d's robot-frame curved path integration, which is less accurate
    // when acceleration is known in the field frame.
    // ax/ay are already field-relative from drivetrain.getAccel().
    double dxField =
        effectiveFieldSpeeds.vxMetersPerSecond * PHASE_DELAY + ONE_HALF_PHASE_DELAY_SQUARED * ax;
    double dyField =
        effectiveFieldSpeeds.vyMetersPerSecond * PHASE_DELAY + ONE_HALF_PHASE_DELAY_SQUARED * ay;
    double dTheta =
        effectiveFieldSpeeds.omegaRadiansPerSecond * PHASE_DELAY
            + ONE_HALF_PHASE_DELAY_SQUARED * alpha;

    Rotation2d predictedAngle = currentPose.getRotation().plus(new Rotation2d(dTheta));
    Pose2d estimatedPose =
        new Pose2d(currentPose.getX() + dxField, currentPose.getY() + dyField, predictedAngle);

    // Predicted field-relative velocity at the moment of launch (end of phase delay).
    // This is what the ball actually inherits — treated as constant for the Newton loop.
    double launchVX = effectiveFieldSpeeds.vxMetersPerSecond + ax * PHASE_DELAY;
    double launchVY = effectiveFieldSpeeds.vyMetersPerSecond + ay * PHASE_DELAY;
    double launchOmega = effectiveFieldSpeeds.omegaRadiansPerSecond + alpha * PHASE_DELAY;

    // --- TURRET VELOCITY (field-relative, at launch time) ---
    // Rotate the turret offset vector into the field frame using the predicted robot angle.
    // Using predictedAngle (not current) keeps this consistent with estimatedPose.
    double cos = predictedAngle.getCos();
    double sin = predictedAngle.getSin();
    double turretOffsetFieldX = TURRET_TX * cos - TURRET_TY * sin;
    double turretOffsetFieldY = TURRET_TX * sin + TURRET_TY * cos;
    // Total angular rate at the turret pivot = robot omega + turret's own spin.
    // Both contribute tangential velocity to the turret point: v = omega x r.
    double totalOmega = launchOmega + turretSubsystem.getOmega();
    double turretVelocityX = launchVX + (-turretOffsetFieldY * totalOmega);
    double turretVelocityY = launchVY + (turretOffsetFieldX * totalOmega);

    // Target translation
    Pose2d turretPose = estimatedPose.transformBy(turretTransform);
    Translation2d target = GetTargetFromPose.getTargetLocation(turretPose);

    double distanceX = target.getX() - turretPose.getX();
    double distanceY = target.getY() - turretPose.getY();
    double distance = Math.hypot(distanceX, distanceY);

    // Newton-Raphson TOF convergence.
    // turretVelocityX/Y are constant — post-launch robot acceleration doesn't affect the ball.
    double trueDistance = distance;
    double trueDistanceX = distanceX;
    double trueDistanceY = distanceY;
    double t = LauncherConstants.getTimeFromDistance(distance);
    for (int i = 0; i < NEWTON_METHOD_MAX_ITERATIONS; i++) {
      double prevT = t;

      trueDistanceX = distanceX - turretVelocityX * t;
      trueDistanceY = distanceY - turretVelocityY * t;
      trueDistance = Math.hypot(trueDistanceX, trueDistanceY);

      double lookupT = LauncherConstants.getTimeFromDistance(trueDistance);
      double f = lookupT - t;

      double dDist_Dt =
          -(trueDistanceX * turretVelocityX + trueDistanceY * turretVelocityY) / trueDistance;
      double fPrime = (derivativeOfTOF(trueDistance) * dDist_Dt) - 1.0;

      if (Math.abs(fPrime) > MIN_SLOPE) {
        t = t - f / fPrime;
      } else {
        t = lookupT;
      }

      if (Math.abs(t - prevT) < CONVERGENCE_TOLERANCE) break;
    }

    // Velocity feedforward angular velocity
    double feedforwardAngularVelocity = 0;
    if (trueDistance > VFF_DIST_TOLERANCE) {
      double tangentialVel =
          (-trueDistanceY * turretVelocityX + trueDistanceX * turretVelocityY) / trueDistance;
      feedforwardAngularVelocity = (tangentialVel / trueDistance) - launchOmega;
    }

    Rotation2d targetAngleFieldRelative;
    if (trueDistance < MIN_DISTANCE_TO_TARGET) {
      targetAngleFieldRelative = Rotation2d.kZero;
    } else {
      targetAngleFieldRelative = new Rotation2d(trueDistanceX, trueDistanceY);
    }

    // Use launchSpeeds (robot-relative) for trench check since isApproachingTrench uses
    // ChassisSpeeds in robot-relative form via Twist2d.exp.
    ChassisSpeeds launchSpeedsRobotRelative =
        ChassisSpeeds.fromFieldRelativeSpeeds(launchVX, launchVY, launchOmega, predictedAngle);
    double targetHood = getHoodAngle(estimatedPose, trueDistance, launchSpeedsRobotRelative);
    double targetFlywheels = LauncherConstants.getFlywheelSpeedFromDistance(trueDistance);
    Rotation2d targetTurret =
        targetAngleFieldRelative.minus(predictedAngle).rotateBy(Rotation2d.k180deg);

    return new LaunchingParameters(
        targetHood, targetTurret, targetFlywheels, feedforwardAngularVelocity, turretPose);
  }

  /**
   * Calculates the derivative of TOF with respect to distance at a given distance.
   *
   * @param distance instantaneous distance
   * @return derivative of the TOF lookup table at the given distance
   */
  public double derivativeOfTOF(double distance) {
    double min = LauncherConstants.getTimeFromDistance(distance - STEP_SIZE);
    double max = LauncherConstants.getTimeFromDistance(distance + STEP_SIZE);
    return (max - min) / (2 * STEP_SIZE);
  }

  /**
   * Returns the hood angle. Returns 0 if approaching the trench to protect the hood.
   *
   * @param robotPose the robot's estimated pose
   * @param trueDist distance from turret to hub
   * @param speeds robot-relative chassis speeds at launch time
   * @return hood angle (unitless, defined by LauncherConstants interpolating map)
   */
  public double getHoodAngle(Pose2d robotPose, double trueDist, ChassisSpeeds speeds) {
    if (isApproachingTrench(robotPose, speeds)) return 0;
    return LauncherConstants.getHoodAngleFromDistance(trueDist);
  }

  public static boolean isApproachingTrench(Pose2d robotPose, ChassisSpeeds speeds) {
    for (int i = 0; i <= TRENCH_LOOKAHEAD_SAMPLES; i++) {
      double t = TRENCH_LOOKAHEAD * i / TRENCH_LOOKAHEAD_SAMPLES;
      Pose2d sampledRobotPose =
          robotPose.exp(
              new Twist2d(
                  speeds.vxMetersPerSecond * t,
                  speeds.vyMetersPerSecond * t,
                  speeds.omegaRadiansPerSecond * t));
      Pose2d sampledTurretPose = sampledRobotPose.transformBy(turretTransform);
      if (isCloseToTrench(sampledTurretPose)) return true;
    }
    return false;
  }

  public static boolean isCloseToTrench(Pose2d pose) {
    Pose2d nearestTag = pose.nearest(trenchTags);
    double dx = Math.abs(pose.getX() - nearestTag.getX());
    double dy = Math.abs(pose.getY() - nearestTag.getY());
    return dx < TURRET_TO_TRENCH_TOLERANCE_X && dy < TURRET_TO_TRENCH_TOLERANCE_Y;
  }

  public static boolean isUnderClimb(Pose2d turretPose) {
    Pose2d nearestTag = turretPose.nearest(underclimbTags);
    double dx = Math.abs(turretPose.getX() - nearestTag.getX());
    double dy = Math.abs(turretPose.getY() - nearestTag.getY());
    return dx < TURRET_TO_UNDERCLIMB_TOLERANCE_X && dy < TURRET_TO_UNDERCLIMB_TOLERANCE_Y;
  }
}
