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

  // Cached variables (mostly for throttling method)
  private LaunchingParameters cachedParams;
  private Pose2d lastPose = new Pose2d();
  private double lastTurretOmega = 0;

  // Pose-differentiation state (for defense/push compensation)
  private Pose2d prevPose = new Pose2d();
  private double prevTimestamp = 0;

  // Acceleration tracking state
  private ChassisSpeeds lastSpeeds = new ChassisSpeeds();
  private double lastSpeedsTimestamp = 0;

  // Throttling Magic numbers
  private static final double MIN_DIST_TOLERANCE = Units.inchesToMeters(1); // Meters
  private static final double MIN_ROTATION_TOLERANCE = Units.degreesToRadians(0.5); // Radians
  private static final double MIN_VELOCITY_TOLERANCE = Units.inchesToMeters(0.5); // M/s
  private static final double MIN_OMEGA_TOLERANCE = 0.05; // Radians/s
  private static final double MIN_ACCEL_TOLERANCE =
      0.3; // M/s^2 - bypass cache if accelerating hard

  // Transforms and pose2ds
  private static final Transform2d turretTransform = LauncherConstants.turretTransform();

  private static final double PHASE_DELAY = 0.02;
  private static final double CONVERGENCE_TOLERANCE = 0.001;
  private static final double STEP_SIZE = 0.01; // Instantaneous rate of change step size in meters
  private static final double MIN_SLOPE = 1e-4;
  private static final int NEWTON_METHOD_MAX_ITERATIONS = 5;
  private static final double VFF_DIST_TOLERANCE = 0.1;
  private static final double MIN_DISTANCE_TO_TARGET = 1e-4;

  // Pose-differentiation velocity blending
  // Maximum blend weight applied when poseDt is at its minimum (most trustworthy).
  // Set high (0.9) because accepted poses are always correct — the dynamic scaling
  // below handles the fast-movement spottiness by fading to wheel speeds as poseDt grows.
  private static final double POSE_VELOCITY_BLEND = 0.9;

  // Sanity bounds for pose-differentiated velocity. If the timestamp delta is outside
  // this range, the pose diff is stale or garbage (e.g. first loop), so we fall back
  // to wheel-reported speeds entirely.
  // MAX_POSE_DT is intentionally tight (0.05s) because at high speed, vision updates
  // become sparse and a large dt produces a coarse, unreliable velocity derivative.
  // When updates are sparse we want wheel speeds, not a bad pose-diff estimate.
  private static final double MIN_POSE_DT = 0.005; // seconds
  private static final double MAX_POSE_DT =
      0.05; // seconds - tightened for fast-movement spottiness

  // Trench stuff
  private static final AprilTagFieldLayout field = AllianceUtils.FIELD_LAYOUT;
  private static final double TURRET_TO_TRENCH_TOLERANCE_X = Units.inchesToMeters(12);
  private static final double TURRET_TO_TRENCH_TOLERANCE_Y = Units.inchesToMeters(24.97);
  private static final double TRENCH_LOOKAHEAD = 0.5; // seconds
  private static final int TRENCH_LOOKAHEAD_SAMPLES = 10;
  private static final List<Pose2d> trenchTags = new ArrayList<>();
  private static final int[] tags = {1, 6, 7, 12, 17, 22, 23, 28}; // Trench tags
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
   * This method returns the cached LaunchingParameter. It updates this cache only if the robot has
   * moved, and is moving within a specified threshold of values. This includes minimum distance,
   * velocity, rotation, and angular velocity. Also bypasses the cache if the robot is accelerating
   * significantly (e.g. being pushed by a defender).
   *
   * @param drivetrain the drivebase's CommandSwerveDrivetrain object
   * @param turretSubsystem the turretSubsystem object. There should only be one instance throughout
   *     the entirety of run time
   * @return LaunchingParameters record holding all the target values.
   */
  public LaunchingParameters getParameters(
      CommandSwerveDrivetrain drivetrain, TurretSubsystem turretSubsystem) {
    SwerveDriveState driveState = drivetrain.getState();
    Pose2d currentPose = driveState.Pose;
    ChassisSpeeds currentSpeeds = driveState.Speeds;
    double currentTurretOmega = turretSubsystem.getOmega();

    // Compute acceleration from wheel-reported speeds for cache invalidation.
    // Even if we end up blending in pose-derived velocity for the shot calc,
    // sudden acceleration (being pushed, fast direction change) should always
    // bust the cache immediately.
    double speedsDt = driveState.Timestamp - lastSpeedsTimestamp;
    double accelMagnitude = 0;
    if (speedsDt > MIN_POSE_DT && speedsDt < MAX_POSE_DT) {
      double ax = (currentSpeeds.vxMetersPerSecond - lastSpeeds.vxMetersPerSecond) / speedsDt;
      double ay = (currentSpeeds.vyMetersPerSecond - lastSpeeds.vyMetersPerSecond) / speedsDt;
      accelMagnitude = Math.hypot(ax, ay);
    }

    boolean hasNotMovedSignificantly =
        Math.abs(currentPose.getTranslation().getDistance(lastPose.getTranslation()))
            <= MIN_DIST_TOLERANCE;
    boolean hasNotRotatedSigificantly =
        Math.abs(currentPose.getRotation().getRadians() - lastPose.getRotation().getRadians())
            <= MIN_ROTATION_TOLERANCE;
    boolean isNotMovingFastEnough =
        Math.abs(currentSpeeds.vxMetersPerSecond) <= MIN_VELOCITY_TOLERANCE
            && Math.abs(currentSpeeds.vyMetersPerSecond) <= MIN_VELOCITY_TOLERANCE
            && Math.abs(currentSpeeds.omegaRadiansPerSecond) <= MIN_ROTATION_TOLERANCE;
    boolean isTurretOmegaStable =
        Math.abs(currentTurretOmega - lastTurretOmega) <= MIN_OMEGA_TOLERANCE;
    // Bypass cache if robot is accelerating hard - covers being pushed by a defender
    // even if wheel-reported velocity hasn't changed much yet (wheel slip case)
    boolean isNotAcceleratingSignificantly = accelMagnitude <= MIN_ACCEL_TOLERANCE;

    if (hasNotMovedSignificantly
        && hasNotRotatedSigificantly
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
      return cachedParams;
    }

    lastPose = currentPose;
    lastTurretOmega = currentTurretOmega;

    cachedParams = calculate(driveState, turretSubsystem);
    return cachedParams;
  }

  /**
   * Returns a new record of all the numbers required to shoot. Improvements over the original:
   *
   * <p>1. ACCELERATION COMPENSATION: The phase-delay pose prediction is now second-order (x = x0 +
   * v*t + 0.5*a*t^2). Acceleration is also applied inside the Newton TOF loop so that the effective
   * turret velocity at ball release time accounts for how the robot is speeding up/slowing down
   * during flight.
   *
   * <p>2. DEFENSE/PUSH COMPENSATION: Instead of using only wheel-reported ChassisSpeeds (which
   * reflect motor output and can lie when a defender shoves the robot and wheels slip), we compute
   * a pose-differentiated velocity from consecutive estimated poses. The estimated pose comes from
   * the full state estimator (wheel odometry + vision + IMU), so it reflects actual displacement.
   * The blend weight scales dynamically with poseDt: high trust (0.9) when vision updates are
   * frequent (low speed), fading to 0 when updates are sparse (high speed), which gracefully
   * handles the "good when still, spotty when fast" vision characteristic.
   *
   * @param driveState the drivebase's SwerveDriveState
   * @param turretSubsystem the turretSubsystem object
   * @return LaunchingParameters record holding all the target values
   */
  public LaunchingParameters calculate(
      SwerveDriveState driveState, TurretSubsystem turretSubsystem) {

    Pose2d estimatedPose = driveState.Pose;
    ChassisSpeeds wheelSpeeds = driveState.Speeds;
    double timestamp = driveState.Timestamp;

    // --- DEFENSE COMPENSATION: Pose-differentiated velocity with dynamic blending ---
    // Compute actual robot velocity from consecutive pose estimates. This is ground-truth
    // displacement regardless of what the wheels are doing (handles defender push/slip).
    //
    // The blend weight scales linearly with poseDt:
    //   - Small poseDt (frequent vision updates, low speed) -> weight near POSE_VELOCITY_BLEND
    // (0.9)
    //   - Large poseDt (sparse vision updates, high speed)  -> weight near 0 (fall back to wheels)
    // This naturally handles the "good when still, spotty when fast" vision characteristic:
    // at high speed where updates are sparse, poseDt grows past MAX_POSE_DT and we fall back
    // entirely to wheel speeds.
    ChassisSpeeds effectiveSpeeds = wheelSpeeds; // default: fall back to wheel speeds
    double poseDt = timestamp - prevTimestamp;
    if (poseDt > MIN_POSE_DT && poseDt < MAX_POSE_DT) {
      // se2 log map gives us the body-frame twist between the two poses
      Twist2d twist = prevPose.log(estimatedPose);
      ChassisSpeeds poseDerivedSpeeds =
          new ChassisSpeeds(twist.dx / poseDt, twist.dy / poseDt, twist.dtheta / poseDt);
      // Scale blend linearly: full POSE_VELOCITY_BLEND at MIN_POSE_DT, 0.0 at MAX_POSE_DT
      double blendAlpha =
          POSE_VELOCITY_BLEND * (1.0 - (poseDt - MIN_POSE_DT) / (MAX_POSE_DT - MIN_POSE_DT));
      effectiveSpeeds =
          new ChassisSpeeds(
              blendAlpha * poseDerivedSpeeds.vxMetersPerSecond
                  + (1.0 - blendAlpha) * wheelSpeeds.vxMetersPerSecond,
              blendAlpha * poseDerivedSpeeds.vyMetersPerSecond
                  + (1.0 - blendAlpha) * wheelSpeeds.vyMetersPerSecond,
              blendAlpha * poseDerivedSpeeds.omegaRadiansPerSecond
                  + (1.0 - blendAlpha) * wheelSpeeds.omegaRadiansPerSecond);
    }
    prevPose = estimatedPose;
    prevTimestamp = timestamp;

    // --- ACCELERATION COMPENSATION: Compute chassis acceleration ---
    // Used for second-order pose prediction and effective velocity at ball-release time.
    ChassisSpeeds acceleration = new ChassisSpeeds(0, 0, 0);
    double speedsDt = timestamp - lastSpeedsTimestamp;
    if (speedsDt > MIN_POSE_DT && speedsDt < MAX_POSE_DT) {
      acceleration =
          new ChassisSpeeds(
              (effectiveSpeeds.vxMetersPerSecond - lastSpeeds.vxMetersPerSecond) / speedsDt,
              (effectiveSpeeds.vyMetersPerSecond - lastSpeeds.vyMetersPerSecond) / speedsDt,
              (effectiveSpeeds.omegaRadiansPerSecond - lastSpeeds.omegaRadiansPerSecond)
                  / speedsDt);
    }
    lastSpeeds = effectiveSpeeds;
    lastSpeedsTimestamp = timestamp;

    // --- SECOND-ORDER PHASE DELAY PREDICTION ---
    // x = x0 + v*t + 0.5*a*t^2  (previously only v*t)
    double pdt = PHASE_DELAY;
    estimatedPose =
        estimatedPose.exp(
            new Twist2d(
                effectiveSpeeds.vxMetersPerSecond * pdt
                    + 0.5 * acceleration.vxMetersPerSecond * pdt * pdt,
                effectiveSpeeds.vyMetersPerSecond * pdt
                    + 0.5 * acceleration.vyMetersPerSecond * pdt * pdt,
                effectiveSpeeds.omegaRadiansPerSecond * pdt
                    + 0.5 * acceleration.omegaRadiansPerSecond * pdt * pdt));

    Rotation2d robotAngle = estimatedPose.getRotation();

    // Turret velocity (robot-relative), then field-relative
    double totalOmega = effectiveSpeeds.omegaRadiansPerSecond + turretSubsystem.getOmega();
    ChassisSpeeds turretRobotRelativeSpeeds =
        new ChassisSpeeds(
            effectiveSpeeds.vxMetersPerSecond
                - effectiveSpeeds.omegaRadiansPerSecond * turretTransform.getY(),
            effectiveSpeeds.vyMetersPerSecond
                + effectiveSpeeds.omegaRadiansPerSecond * turretTransform.getX(),
            totalOmega);
    ChassisSpeeds turretFieldRelativeSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(turretRobotRelativeSpeeds, robotAngle);
    double turretVelocityX = turretFieldRelativeSpeeds.vxMetersPerSecond;
    double turretVelocityY = turretFieldRelativeSpeeds.vyMetersPerSecond;

    // Turret acceleration field-relative (for effective velocity at TOF time t)
    ChassisSpeeds turretAccelRobotRelative =
        new ChassisSpeeds(
            acceleration.vxMetersPerSecond
                - acceleration.omegaRadiansPerSecond * turretTransform.getY(),
            acceleration.vyMetersPerSecond
                + acceleration.omegaRadiansPerSecond * turretTransform.getX(),
            acceleration.omegaRadiansPerSecond);
    ChassisSpeeds turretAccelFieldRelative =
        ChassisSpeeds.fromRobotRelativeSpeeds(turretAccelRobotRelative, robotAngle);
    double turretAccelX = turretAccelFieldRelative.vxMetersPerSecond;
    double turretAccelY = turretAccelFieldRelative.vyMetersPerSecond;

    // Target translation
    Pose2d turretPose = estimatedPose.transformBy(turretTransform);
    Translation2d target = GetTargetFromPose.getTargetLocation(turretPose);

    // Initial distances
    double distanceX = target.getX() - turretPose.getX();
    double distanceY = target.getY() - turretPose.getY();
    double distance = Math.hypot(distanceX, distanceY);

    // Newton-Raphson TOF convergence
    // Now uses effective velocity at time t (v + a*t) so the ball's predicted landing spot
    // accounts for how the robot is accelerating during flight.
    double trueDistance = distance;
    double trueDistanceX = distanceX;
    double trueDistanceY = distanceY;
    double t = LauncherConstants.getTimeFromDistance(distance);
    for (int i = 0; i < NEWTON_METHOD_MAX_ITERATIONS; i++) {
      double prevT = t;

      // Effective turret velocity at time t: v + a*t
      // The ball inherits the turret's velocity at launch, but if the robot is accelerating,
      // the effective launch velocity changes with the converged t.
      double effectiveTurretVX = turretVelocityX + turretAccelX * t;
      double effectiveTurretVY = turretVelocityY + turretAccelY * t;

      trueDistanceX = distanceX - effectiveTurretVX * t;
      trueDistanceY = distanceY - effectiveTurretVY * t;
      trueDistance = Math.hypot(trueDistanceX, trueDistanceY);

      double lookupT = LauncherConstants.getTimeFromDistance(trueDistance);
      double f = lookupT - t;

      // d(trueDistance)/dt accounts for the acceleration term:
      // trueDistanceX(t) = distanceX - (vx + ax*t)*t = distanceX - vx*t - ax*t^2
      // d/dt = -(vx + 2*ax*t), similarly for Y, then chain rule.
      double dDist_Dt =
          -(trueDistanceX * (turretVelocityX + 2 * turretAccelX * t)
                  + trueDistanceY * (turretVelocityY + 2 * turretAccelY * t))
              / trueDistance;
      double fPrime = (derivativeOfTOF(trueDistance) * dDist_Dt) - 1.0;

      if (Math.abs(fPrime) > MIN_SLOPE) {
        t = t - f / fPrime;
      } else {
        t = lookupT;
      }

      if (Math.abs(t - prevT) < CONVERGENCE_TOLERANCE) break;
    }

    // Velocity feedforward (uses final converged effective velocity)
    double effectiveTurretVXFinal = turretVelocityX + turretAccelX * t;
    double effectiveTurretVYFinal = turretVelocityY + turretAccelY * t;
    double feedforwardAngularVelocity = 0;
    if (trueDistance > VFF_DIST_TOLERANCE) {
      double tangentialVel =
          (-trueDistanceY * effectiveTurretVXFinal + trueDistanceX * effectiveTurretVYFinal)
              / trueDistance;
      feedforwardAngularVelocity =
          (tangentialVel / trueDistance) - effectiveSpeeds.omegaRadiansPerSecond;
    }

    Rotation2d targetAngleFieldRelative;
    if (trueDistance < MIN_DISTANCE_TO_TARGET) {
      targetAngleFieldRelative = Rotation2d.kZero;
    } else {
      targetAngleFieldRelative = new Rotation2d(trueDistanceX, trueDistanceY);
    }

    double targetHood = getHoodAngle(estimatedPose, trueDistance, effectiveSpeeds);
    double targetFlywheels = LauncherConstants.getFlywheelSpeedFromDistance(trueDistance);
    Rotation2d targetTurret =
        targetAngleFieldRelative.minus(robotAngle).rotateBy(Rotation2d.k180deg);

    return new LaunchingParameters(
        targetHood, targetTurret, targetFlywheels, feedforwardAngularVelocity, turretPose);
  }

  /**
   * Calculates the derivative of Time with respect to distance, at a given distance.
   *
   * @param distance instantaneous distance
   * @return the derivative of the TOF lookup table at the given distance
   */
  public double derivativeOfTOF(double distance) {
    double min = LauncherConstants.getTimeFromDistance(distance - STEP_SIZE);
    double max = LauncherConstants.getTimeFromDistance(distance + STEP_SIZE);
    return (max - min) / (2 * STEP_SIZE);
  }

  /**
   * Returns the hood angle. Checks isApproachingTrench(); if true, returns 0 to protect the hood.
   *
   * @param robotPose the robot's estimated pose
   * @param trueDist distance from turret to hub
   * @param speeds effective chassis speeds (blended pose-derived + wheel)
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
