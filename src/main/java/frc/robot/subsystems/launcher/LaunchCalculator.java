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

  // Pose-differentiation state (for defense/push compensation).
  // Updated every cycle in getParameters() so poseDt stays valid even during cache hits.
  private Pose2d prevPose = new Pose2d();
  private double prevTimestamp = 0;

  // Acceleration tracking state — always derived from raw wheel speeds, never from
  // the blended effectiveSpeeds, to avoid amplifying pose-diff noise.
  // Updated every cycle in getParameters() so speedsDt stays valid during cache hits.
  private ChassisSpeeds lastWheelSpeeds = new ChassisSpeeds();
  private double lastWheelSpeedsTimestamp = 0;
  private ChassisSpeeds filteredAcceleration = new ChassisSpeeds(0, 0, 0);

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

  // Pose-differentiation velocity blending.
  // Maximum blend weight when poseDt is at its minimum (most trustworthy).
  // Only active when the robot is actually moving — see robotIsMoving gate below.
  private static final double POSE_VELOCITY_BLEND = 0.9;

  // Sanity bounds for pose-differentiated velocity. If the timestamp delta is outside
  // this range, the pose diff is stale or garbage (e.g. first loop), so we fall back
  // to wheel-reported speeds entirely. MAX_POSE_DT is tight because at high speed,
  // vision updates become sparse and a large dt produces an unreliable velocity derivative.
  private static final double MIN_POSE_DT = 0.005; // seconds
  private static final double MAX_POSE_DT = 0.05; // seconds

  // Gate for pose-derived velocity. Below this wheel speed magnitude we consider the
  // robot stationary and skip pose-diff entirely. When still, dividing tiny estimator
  // corrections (vision noise, IMU drift) by a small dt produces large fake velocities
  // that jitter the turret. Wheel speeds near zero are the correct signal in that case.
  private static final double MIN_MOVING_SPEED = 0.05; // m/s

  // Low-pass filter for acceleration. Differentiating velocity is inherently noisy;
  // this smooths the raw derivative before it is used for phase-delay prediction.
  // 0 = no filtering, 1 = effectively disabled. Tune up if jitter persists,
  // down if phase-delay prediction feels sluggish during fast direction changes.
  private static final double ACCEL_FILTER_ALPHA = 0.8;

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
   * <p>State variables used for velocity/acceleration differentiation (prevPose, prevTimestamp,
   * lastWheelSpeeds, lastWheelSpeedsTimestamp) are updated every cycle here regardless of whether
   * the cache is hit. This keeps time deltas valid so the acceleration cache-bust logic doesn't
   * silently disable itself during stable periods when calculate() is being skipped.
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
    double timestamp = driveState.Timestamp;

    // --- Update differentiation state every cycle, even on cache hits ---
    // If we only updated inside calculate(), speedsDt and poseDt would grow past MAX_POSE_DT
    // during stable periods and the acceleration cache-bust would silently stop working.
    double speedsDt = timestamp - lastWheelSpeedsTimestamp;
    double accelMagnitude = 0;
    if (speedsDt > MIN_POSE_DT && speedsDt < MAX_POSE_DT) {
      double ax = (currentSpeeds.vxMetersPerSecond - lastWheelSpeeds.vxMetersPerSecond) / speedsDt;
      double ay = (currentSpeeds.vyMetersPerSecond - lastWheelSpeeds.vyMetersPerSecond) / speedsDt;
      accelMagnitude = Math.hypot(ax, ay);

      double rawAOmega =
          (currentSpeeds.omegaRadiansPerSecond - lastWheelSpeeds.omegaRadiansPerSecond) / speedsDt;
      filteredAcceleration =
          new ChassisSpeeds(
              ACCEL_FILTER_ALPHA * filteredAcceleration.vxMetersPerSecond
                  + (1 - ACCEL_FILTER_ALPHA) * ax,
              ACCEL_FILTER_ALPHA * filteredAcceleration.vyMetersPerSecond
                  + (1 - ACCEL_FILTER_ALPHA) * ay,
              ACCEL_FILTER_ALPHA * filteredAcceleration.omegaRadiansPerSecond
                  + (1 - ACCEL_FILTER_ALPHA) * rawAOmega);
    }
    lastWheelSpeeds = currentSpeeds;
    lastWheelSpeedsTimestamp = timestamp;
    prevPose = currentPose;
    prevTimestamp = timestamp;

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
   * Returns a new record of all the numbers required to shoot. Key design decisions:
   *
   * <p>1. ACCELERATION COMPENSATION (phase delay only): Acceleration is used to predict the robot's
   * velocity at the end of PHASE_DELAY via x = x0 + v*t + 0.5*a*t^2. Once the ball leaves the
   * shooter, the robot's acceleration no longer affects its trajectory, so the velocity predicted
   * at the end of PHASE_DELAY is treated as constant throughout the Newton loop. Acceleration is
   * always derived from raw wheel speeds (not the blended effectiveSpeeds) and low-pass filtered to
   * avoid amplifying noise.
   *
   * <p>2. DEFENSE/PUSH COMPENSATION: Pose-differentiated velocity blended with wheel speeds. The
   * blend weight scales dynamically with poseDt and is gated off when stationary to prevent
   * estimator noise from being amplified into a fake jittery velocity signal.
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
    // Gate: only use pose-derived velocity when the robot is actually moving.
    // When still, tiny estimator corrections (vision noise, IMU drift) divided by a small dt
    // produce large fake velocities. Wheel speeds near zero are the correct signal when stationary.
    double wheelSpeedMagnitude =
        Math.hypot(wheelSpeeds.vxMetersPerSecond, wheelSpeeds.vyMetersPerSecond);
    boolean robotIsMoving = wheelSpeedMagnitude > MIN_MOVING_SPEED;

    ChassisSpeeds effectiveSpeeds = wheelSpeeds; // default: use wheel speeds
    double poseDt = timestamp - prevTimestamp;
    if (robotIsMoving && poseDt > MIN_POSE_DT && poseDt < MAX_POSE_DT) {
      Twist2d twist = prevPose.log(estimatedPose);
      ChassisSpeeds poseDerivedSpeeds =
          new ChassisSpeeds(twist.dx / poseDt, twist.dy / poseDt, twist.dtheta / poseDt);
      // Scale blend linearly: full POSE_VELOCITY_BLEND at MIN_POSE_DT, 0.0 at MAX_POSE_DT.
      // Sparse vision updates (high speed) produce a large poseDt and fade naturally to wheels.
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

    // --- PHASE DELAY PREDICTION (second-order) ---
    // Acceleration is used here to predict the robot's state at the end of PHASE_DELAY:
    //   x = x0 + v*t + 0.5*a*t^2
    // The velocity at the end of PHASE_DELAY (v + a*PHASE_DELAY) is what the turret
    // will actually be doing when the ball leaves the shooter. That predicted velocity
    // is then treated as CONSTANT for the Newton-Raphson loop below, because once the
    // ball is in flight the robot's acceleration no longer affects its trajectory.
    ChassisSpeeds acceleration = filteredAcceleration;
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

    // Velocity at the end of PHASE_DELAY — constant for the remainder of the calculation.
    ChassisSpeeds launchSpeeds =
        new ChassisSpeeds(
            effectiveSpeeds.vxMetersPerSecond + acceleration.vxMetersPerSecond * pdt,
            effectiveSpeeds.vyMetersPerSecond + acceleration.vyMetersPerSecond * pdt,
            effectiveSpeeds.omegaRadiansPerSecond + acceleration.omegaRadiansPerSecond * pdt);

    Rotation2d robotAngle = estimatedPose.getRotation();

    // Turret velocity at launch time (robot-relative), then field-relative.
    // Uses launchSpeeds (not effectiveSpeeds) because this is the velocity the ball inherits.
    double totalOmega = launchSpeeds.omegaRadiansPerSecond + turretSubsystem.getOmega();
    ChassisSpeeds turretRobotRelativeSpeeds =
        new ChassisSpeeds(
            launchSpeeds.vxMetersPerSecond
                - launchSpeeds.omegaRadiansPerSecond * turretTransform.getY(),
            launchSpeeds.vyMetersPerSecond
                + launchSpeeds.omegaRadiansPerSecond * turretTransform.getX(),
            totalOmega);
    ChassisSpeeds turretFieldRelativeSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(turretRobotRelativeSpeeds, robotAngle);

    // These are constant for the Newton loop — ball velocity is fixed at launch.
    double turretVelocityX = turretFieldRelativeSpeeds.vxMetersPerSecond;
    double turretVelocityY = turretFieldRelativeSpeeds.vyMetersPerSecond;

    // Target translation
    Pose2d turretPose = estimatedPose.transformBy(turretTransform);
    Translation2d target = GetTargetFromPose.getTargetLocation(turretPose);

    // Initial distances
    double distanceX = target.getX() - turretPose.getX();
    double distanceY = target.getY() - turretPose.getY();
    double distance = Math.hypot(distanceX, distanceY);

    // Newton-Raphson TOF convergence.
    // turretVelocityX/Y are constant here — post-launch robot acceleration doesn't
    // affect the ball. The derivative is the same as the original (no acceleration terms).
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

    // Velocity feedforward
    double feedforwardAngularVelocity = 0;
    if (trueDistance > VFF_DIST_TOLERANCE) {
      double tangentialVel =
          (-trueDistanceY * turretVelocityX + trueDistanceX * turretVelocityY) / trueDistance;
      feedforwardAngularVelocity =
          (tangentialVel / trueDistance) - launchSpeeds.omegaRadiansPerSecond;
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
