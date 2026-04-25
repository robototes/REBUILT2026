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
  private double lastTimeStamp = 0;

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

  // Throttling Magic numbers
  private static final double MIN_DIST_TOLERANCE = Units.inchesToMeters(1); // Meters
  private static final double MIN_ROTATION_TOLERANCE = Units.degreesToRadians(0.5); // Radians
  private static final double MIN_VELOCITY_TOLERANCE = Units.inchesToMeters(0.5); // M/s
  private static final double MIN_OMEGA_TOLERANCE = 0.05; // Radians/s
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
  private static final double TURRET_TX;
  private static final double TURRET_TY;

  // Pose-differentiation velocity blending.
  // Maximum blend weight when poseDt is at its minimum (most trustworthy).
  // Only active when the robot is actually moving — see robotIsMoving gate below.
  private static final double POSE_VELOCITY_BLEND = 0.9;

  // Sanity bounds for pose-differentiated velocity.
  private static final double MIN_POSE_DT = 0.005; // seconds
  private static final double MAX_POSE_DT = 0.05; // seconds

  // Gate for pose-derived velocity. Below this wheel speed magnitude we consider the
  // robot stationary and skip pose-diff to avoid amplifying estimator noise into jitter.
  private static final double MIN_MOVING_SPEED = 0.05; // m/s

  // Low-pass filter for acceleration. 0 = no filtering, 1 = effectively disabled.
  private static final double ACCEL_FILTER_ALPHA = 0.9;

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

  private static final double PHASE_DELAY_SQUARED = PHASE_DELAY * PHASE_DELAY;

  // Precalculated: PHASEDELAY ^ 2 * 0.5
  private static final double ONE_HALF_X_PHASE_D_SQUARED = 0.5 * PHASE_DELAY_SQUARED;

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
    double currentTimeStamp = driveState.Timestamp;
    if (currentTimeStamp == lastTimeStamp && cachedParams != null) {
      return cachedParams;
    }

    Pose2d currentPose = driveState.Pose;
    ChassisSpeeds currentSpeeds = driveState.Speeds;
    double currentTurretOmega = turretSubsystem.getOmega();
    // If the robot has moved within a certain threshold

    Translation2d accel = drivetrain.getAccel();
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
        Math.hypot(accel.getX(), accel.getY()) <= MIN_ACCEL_TOLERANCE
            && Math.abs(alpha) <= MIN_ANGULAR_ACCEL_TOLERANCE;

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
      // Update differentiation state on cache hit so deltas stay valid next cycle.
      lastWheelSpeeds = currentSpeeds;
      lastWheelSpeedsTimestamp = timestamp;
      prevPose = currentPose;
      prevTimestamp = timestamp;
      return cachedParams;
    }

    lastPose = currentPose;
    lastTurretOmega = currentTurretOmega;
    lastTimeStamp = currentTimeStamp;

    cachedParams = calculate(driveState, turretSubsystem, accel.getX(), accel.getY(), alpha);
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
   * <p>2. DEFENSE/PUSH COMPENSATION: Pose-differentiated velocity blended with wheel speeds. Blend
   * weight scales dynamically with poseDt and is gated off when stationary to prevent estimator
   * noise from being amplified into a fake jittery velocity signal.
   *
   * @param driveState the drivebase's SwerveDriveState
   * @param turretSubsystem the turretSubsystem object
   * @return LaunchingParameters record holding all the target values
   */
  public LaunchingParameters calculate(
      SwerveDriveState driveState,
      TurretSubsystem turretSubsystem,
      double ax,
      double ay,
      double alpha) {

    ChassisSpeeds chassisSpeeds = driveState.Speeds;
    Pose2d currentPose = driveState.Pose;

    // 1. Convert current speeds to pure Field-Relative values
    ChassisSpeeds fieldSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeeds, currentPose.getRotation());

    // 2. Predict Field-Relative Pose using Acceleration (PHASE_DELAY)

    // Field-relative displacement
    double dxField =
        (fieldSpeeds.vxMetersPerSecond * PHASE_DELAY) + (ONE_HALF_X_PHASE_D_SQUARED * ax);
    double dyField =
        (fieldSpeeds.vyMetersPerSecond * PHASE_DELAY) + (ONE_HALF_X_PHASE_D_SQUARED * ay);

    // Angular changes are frame-independent in 2D
    double dTheta =
        (chassisSpeeds.omegaRadiansPerSecond * PHASE_DELAY) + (ONE_HALF_X_PHASE_D_SQUARED * alpha);
    Rotation2d predictedAngle = currentPose.getRotation().plus(new Rotation2d(dTheta));

    // Apply translation directly in the field frame (bypassing Twist2d's curved robot-frame path)
    Pose2d estimatedPose =
        new Pose2d(currentPose.getX() + dxField, currentPose.getY() + dyField, predictedAngle);
    Rotation2d robotAngle = estimatedPose.getRotation();

    // 3. Predict Field-Relative Velocities at the moment of launch
    double predicted_vx_field = fieldSpeeds.vxMetersPerSecond + (ax * PHASE_DELAY);
    double predicted_vy_field = fieldSpeeds.vyMetersPerSecond + (ay * PHASE_DELAY);
    double predicted_omega_robot = chassisSpeeds.omegaRadiansPerSecond + (alpha * PHASE_DELAY);
    // 4. Calculate final Field-Relative Turret speeds
    // Find the turret's translation vector rotated into the field frame
    double cos = predictedAngle.getCos(), sin = predictedAngle.getSin();
    double turretOffsetFieldX = TURRET_TX * cos - TURRET_TY * sin;
    double turretOffsetFieldY = TURRET_TX * sin + TURRET_TY * cos;
    // Tangential velocity of the turret axis due to robot rotation
    double tangential_vx_field = -turretOffsetFieldY * predicted_omega_robot;
    double tangential_vy_field = turretOffsetFieldX * predicted_omega_robot;
    // Combine robot translational velocity with turret tangential velocity
    double turretVelocityX = predicted_vx_field + tangential_vx_field;
    double turretVelocityY = predicted_vy_field + tangential_vy_field;

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
      trueDistance = Math.sqrt(trueDistanceX * trueDistanceX + trueDistanceY * trueDistanceY);

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

      // Calculated using the standard angular velocity formula (linear velocity / radius). We
      // offset it with the robot's field angular velocity to get the true angular velocity in
      // radians per second
      feedforwardAngularVelocity = (tangentialVel / trueDistance) - predicted_omega_robot; // RAD/S
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
