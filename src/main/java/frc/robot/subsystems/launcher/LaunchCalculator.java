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
  // Updated at the END of getParameters() — after calculate() has consumed the previous
  // state — so that poseDt and speedsDt reflect a real elapsed interval, not zero.
  private Pose2d prevPose = new Pose2d();
  private double prevTimestamp = 0;

  // Acceleration tracking state — always derived from raw wheel speeds.
  // Also updated at the end of getParameters(), including on cache hits, so that
  // the filtered acceleration and time deltas stay valid every cycle.
  private ChassisSpeeds lastWheelSpeeds = new ChassisSpeeds();
  private double lastWheelSpeedsTimestamp = 0;
  private ChassisSpeeds filteredAcceleration = new ChassisSpeeds(0, 0, 0);

  // Hysteresis state for robotIsMoving gate.
  // Requires a higher speed to turn ON than to turn OFF, preventing rapid toggling at the
  // boundary and the stale-baseline spike on start/stop.
  private boolean robotIsMoving = false;

  // Throttling Magic numbers
  private static final double MIN_DIST_TOLERANCE = Units.inchesToMeters(1); // Meters
  private static final double MIN_ROTATION_TOLERANCE = Units.degreesToRadians(0.5); // Radians
  private static final double MIN_VELOCITY_TOLERANCE = Units.inchesToMeters(0.5); // M/s
  private static final double MIN_OMEGA_TOLERANCE = 0.05; // Radians/s
  // Cache bust threshold uses filtered acceleration magnitude (linear + angular).
  // Using raw acceleration here would cause frequent unnecessary cache misses from noise.
  private static final double MIN_ACCEL_TOLERANCE = 0.3; // M/s^2
  private static final double MIN_ANGULAR_ACCEL_TOLERANCE = 0.5; // Rad/s^2

  // Transforms and pose2ds
  private static final Transform2d turretTransform = LauncherConstants.turretTransform();

  private static final double PHASE_DELAY = 0.02;
  private static final double CONVERGENCE_TOLERANCE = 0.001;
  private static final double STEP_SIZE = 0.01;
  private static final double MIN_SLOPE = 1e-4;
  private static final int NEWTON_METHOD_MAX_ITERATIONS = 5;
  private static final double VFF_DIST_TOLERANCE = 0.1;
  private static final double MIN_DISTANCE_TO_TARGET = 1e-4;

  // Sanity bounds for pose-differentiated velocity.
  private static final double MIN_POSE_DT = 0.005; // seconds
  private static final double MAX_POSE_DT = 0.05; // seconds

  // Hysteresis thresholds for the robotIsMoving gate.
  // Must reach ENABLE threshold to activate; must drop below DISABLE threshold to deactivate.
  // The gap prevents rapid toggling and the stale-baseline spike on start/stop.
  private static final double MIN_MOVING_SPEED_ENABLE = 0.1; // m/s
  private static final double MIN_MOVING_SPEED_DISABLE = 0.03; // m/s

  // Low-pass filter for acceleration. 0 = no filtering, 1 = effectively disabled.
  private static final double ACCEL_FILTER_ALPHA = 0.9;

  // Variable-alpha low-pass filter for the slip estimate. Vision-correction noise and
  // real wheel slip live in different magnitude regimes:
  //   - Vision noise: <1 cm correction over 20 ms, single-frame transients.
  //   - Real stuck/push: wheels at 2 m/s vs robot at 0 = 2+ m/s, sustained over many frames.
  // We use a heavy filter (slow) below the threshold to reject noise, and a light filter
  // (fast) above it to track real events with minimal lag. Hysteresis on the threshold
  // prevents flicker between modes.
  private static final double SLIP_FILTER_ALPHA_SLOW = 0.85; // noise rejection
  private static final double SLIP_FILTER_ALPHA_FAST = 0.20; // fast tracking
  private static final double FAST_SLIP_THRESHOLD_ENTER = 2.0; // m/s
  private static final double FAST_SLIP_THRESHOLD_EXIT = 0.5; // m/s (hysteresis)
  private boolean slipFastMode = false;

  // Safety caps on the filtered slip. Prevents a runaway pose-estimator glitch, such as
  // a bad vision measurement that spikes the derivative, from injecting nonsense into the
  // calculation. Real robot pushes and rotational disturbances stay well below these.
  private static final double MAX_SLIP_MAGNITUDE = 5.0; // m/s
  private static final double MAX_SLIP_OMEGA = 8.0; // rad/s (~1.3 rotations/s)

  // Filtered slip state (instance, persists across cycles)
  private ChassisSpeeds filteredSlip = new ChassisSpeeds(0, 0, 0);

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
   * moved, and is moving within a specified threshold of values.
   *
   * <p>Differentiation state (prevPose, prevTimestamp, lastWheelSpeeds, lastWheelSpeedsTimestamp,
   * filteredAcceleration) is updated at the END of this method in all code paths — both on cache
   * hits and after calculate(). This is critical: updating them BEFORE calculate() would cause
   * poseDt = 0 inside calculate(), disabling pose-diff entirely. Updating them AFTER ensures
   * calculate() always sees a real elapsed interval, and that cache hits don't let the deltas go
   * stale over time.
   *
   * @param drivetrain the drivebase's CommandSwerveDrivetrain object
   * @param turretSubsystem the turretSubsystem object
   * @return LaunchingParameters record holding all the target values.
   */
  public LaunchingParameters getParameters(
      CommandSwerveDrivetrain drivetrain, TurretSubsystem turretSubsystem) {
    SwerveDriveState driveState = drivetrain.getState();
    Pose2d currentPose = driveState.Pose;
    ChassisSpeeds currentSpeeds = driveState.Speeds;
    double currentTurretOmega = turretSubsystem.getOmega();
    double timestamp = driveState.Timestamp;

    if (cachedParams != null && timestamp == prevTimestamp) {
      return cachedParams;
    }

    // Compute filtered acceleration for cache-bust check.
    // Uses filteredAcceleration (not raw delta) to avoid noise-driven unnecessary cache misses.
    // Includes angular acceleration so aggressive rotation also busts the cache.
    double speedsDt = timestamp - lastWheelSpeedsTimestamp;
    if (speedsDt > MIN_POSE_DT && speedsDt < MAX_POSE_DT) {
      double rawAX =
          (currentSpeeds.vxMetersPerSecond - lastWheelSpeeds.vxMetersPerSecond) / speedsDt;
      double rawAY =
          (currentSpeeds.vyMetersPerSecond - lastWheelSpeeds.vyMetersPerSecond) / speedsDt;
      double rawAOmega =
          (currentSpeeds.omegaRadiansPerSecond - lastWheelSpeeds.omegaRadiansPerSecond) / speedsDt;
      filteredAcceleration =
          new ChassisSpeeds(
              ACCEL_FILTER_ALPHA * filteredAcceleration.vxMetersPerSecond
                  + (1 - ACCEL_FILTER_ALPHA) * rawAX,
              ACCEL_FILTER_ALPHA * filteredAcceleration.vyMetersPerSecond
                  + (1 - ACCEL_FILTER_ALPHA) * rawAY,
              ACCEL_FILTER_ALPHA * filteredAcceleration.omegaRadiansPerSecond
                  + (1 - ACCEL_FILTER_ALPHA) * rawAOmega);
    } else if (speedsDt >= MAX_POSE_DT) {
      // speedsDt outside valid range means the system was disabled, restarted, or had a
      // significant loop overrun. Reset to zero so stale acceleration from before the gap
      // doesn't corrupt the phase delay prediction on the first active cycle.
      filteredAcceleration = new ChassisSpeeds(0, 0, 0);
    }

    double filteredAccelMagnitude =
        Math.hypot(filteredAcceleration.vxMetersPerSecond, filteredAcceleration.vyMetersPerSecond);

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
    boolean isNotAcceleratingSignificantly =
        filteredAccelMagnitude <= MIN_ACCEL_TOLERANCE
            && Math.abs(filteredAcceleration.omegaRadiansPerSecond) <= MIN_ANGULAR_ACCEL_TOLERANCE;

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
      // Update differentiation state on cache hit so deltas stay valid next cycle.
      lastWheelSpeeds = currentSpeeds;
      lastWheelSpeedsTimestamp = timestamp;
      prevPose = currentPose;
      prevTimestamp = timestamp;
      return cachedParams;
    }

    lastPose = currentPose;
    lastTurretOmega = currentTurretOmega;

    cachedParams = calculate(driveState, turretSubsystem);

    // Update differentiation state AFTER calculate() so that calculate() sees the previous
    // cycle's state (giving a real non-zero poseDt / speedsDt), and the next cycle sees
    // this cycle's state as its baseline.
    lastWheelSpeeds = currentSpeeds;
    lastWheelSpeedsTimestamp = timestamp;
    prevPose = currentPose;
    prevTimestamp = timestamp;

    return cachedParams;
  }

  /**
   * Returns a new record of all the numbers required to shoot. Key design decisions:
   *
   * <p>1. ACCELERATION COMPENSATION (phase delay only): Acceleration is used to predict the robot's
   * velocity at the end of PHASE_DELAY via x = x0 + v*t + 0.5*a*t^2. The velocity predicted at end
   * of phase delay (launchSpeeds) is treated as constant throughout the Newton loop — post-launch
   * robot acceleration does not affect ball trajectory.
   *
   * <p>2. DEFENSE/PUSH COMPENSATION: Slip filter blended on top of wheel speeds. The slip is the
   * difference between pose-derived velocity and wheel velocity. A variable-alpha low-pass filter
   * separates real push events (large, sustained slip) from vision noise (small, transient). Gated
   * off when stationary to prevent estimator noise from being amplified into jitter.
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

    // --- DEFENSE COMPENSATION: Slip filter with hysteresis gate ---
    // When still, tiny estimator corrections divided by a small dt produce large fake velocities.
    double wheelSpeedMagnitude =
        Math.hypot(wheelSpeeds.vxMetersPerSecond, wheelSpeeds.vyMetersPerSecond);

    if (robotIsMoving) {
      robotIsMoving = wheelSpeedMagnitude > MIN_MOVING_SPEED_DISABLE;
    } else {
      robotIsMoving = wheelSpeedMagnitude > MIN_MOVING_SPEED_ENABLE;
    }

    ChassisSpeeds effectiveSpeeds = wheelSpeeds;
    double poseDt = timestamp - prevTimestamp;
    if (robotIsMoving && poseDt > MIN_POSE_DT && poseDt < MAX_POSE_DT) {
      Twist2d twist = prevPose.log(estimatedPose);

      // Slip = (pose-derived velocity) - (wheel velocity). When wheels match pose,
      // slip is ~0; when the robot is pushed or wheels slip, slipRaw is the motion delta.
      double slipRawX = twist.dx / poseDt - wheelSpeeds.vxMetersPerSecond;
      double slipRawY = twist.dy / poseDt - wheelSpeeds.vyMetersPerSecond;
      double slipRawOmega = twist.dtheta / poseDt - wheelSpeeds.omegaRadiansPerSecond;

      // Switch to fast tracking when slip is large enough to be a real event
      // rather than vision noise. Hysteresis prevents flicker.
      double slipRawMag = Math.hypot(slipRawX, slipRawY);
      slipFastMode =
          slipRawMag > FAST_SLIP_THRESHOLD_EXIT
              ? slipRawMag > FAST_SLIP_THRESHOLD_ENTER
              : slipFastMode;
      double filterAlpha = slipFastMode ? SLIP_FILTER_ALPHA_FAST : SLIP_FILTER_ALPHA_SLOW;

      double newSlipX = filterAlpha * filteredSlip.vxMetersPerSecond + (1 - filterAlpha) * slipRawX;
      double newSlipY = filterAlpha * filteredSlip.vyMetersPerSecond + (1 - filterAlpha) * slipRawY;
      double newSlipOmega =
          filterAlpha * filteredSlip.omegaRadiansPerSecond + (1 - filterAlpha) * slipRawOmega;

      // Cap the filtered slip magnitude
      double newSlipMag = Math.hypot(newSlipX, newSlipY);
      if (newSlipMag > MAX_SLIP_MAGNITUDE) {
        double scale = MAX_SLIP_MAGNITUDE / newSlipMag;
        newSlipX *= scale;
        newSlipY *= scale;
      }
      newSlipOmega = Math.max(-MAX_SLIP_OMEGA, Math.min(MAX_SLIP_OMEGA, newSlipOmega));
      filteredSlip = new ChassisSpeeds(newSlipX, newSlipY, newSlipOmega);

      effectiveSpeeds =
          new ChassisSpeeds(
              wheelSpeeds.vxMetersPerSecond + filteredSlip.vxMetersPerSecond,
              wheelSpeeds.vyMetersPerSecond + filteredSlip.vyMetersPerSecond,
              wheelSpeeds.omegaRadiansPerSecond + filteredSlip.omegaRadiansPerSecond);
    } else {
      // Reset slip filter when robot stops or poseDt is out of range
      filteredSlip = new ChassisSpeeds(0, 0, 0);
      slipFastMode = false;
    }

    // --- PHASE DELAY PREDICTION (second-order) ---
    // Predict robot state at end of PHASE_DELAY: x = x0 + v*t + 0.5*a*t^2.
    // launchSpeeds = velocity at end of phase delay = what the ball actually inherits.
    // This is treated as constant for all subsequent calculations.
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

    ChassisSpeeds launchSpeeds =
        new ChassisSpeeds(
            effectiveSpeeds.vxMetersPerSecond + acceleration.vxMetersPerSecond * pdt,
            effectiveSpeeds.vyMetersPerSecond + acceleration.vyMetersPerSecond * pdt,
            effectiveSpeeds.omegaRadiansPerSecond + acceleration.omegaRadiansPerSecond * pdt);

    Rotation2d robotAngle = estimatedPose.getRotation();

    // Turret velocity at launch (robot-relative then field-relative).
    // Uses launchSpeeds — this is the velocity the ball inherits at the moment of release.
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

    // Constant for the Newton loop — ball velocity is fixed at launch.
    double turretVelocityX = turretFieldRelativeSpeeds.vxMetersPerSecond;
    double turretVelocityY = turretFieldRelativeSpeeds.vyMetersPerSecond;

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

    // Use launchSpeeds for trench check — consistent with the velocity used for all other
    // calculations and represents the predicted state at the moment of launch.
    double targetHood = getHoodAngle(estimatedPose, trueDistance, launchSpeeds);
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
   * @param speeds chassis speeds at launch time
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
