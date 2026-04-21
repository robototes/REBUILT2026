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

  // Throttling Magic numbers
  private static final double MIN_DIST_TOLERANCE = Units.inchesToMeters(1); // Meters
  private static final double MIN_ROTATION_TOLERANCE = Units.degreesToRadians(0.5); // Radians
  private static final double MIN_VELOCITY_TOLERANCE = Units.inchesToMeters(0.5); // M/s
  private static final double MIN_OMEGA_TOLERANCE = 0.05; // Radians/s

  // Transforms and pose2ds
  private static final Transform2d turretTransform = LauncherConstants.turretTransform();

  private static final double PHASE_DELAY = 0.02;
  private static final double CONVERGENCE_TOLERANCE = 0.001;
  private static final double STEP_SIZE = 0.01; // Instantaneous rate of change step size in meters
  private static final double MIN_SLOPE = 1e-4;
  private static final int NEWTON_METHOD_MAX_ITERATIONS = 5;
  private static final double VFF_DIST_TOLERANCE = 0.1;
  private static final double MIN_DISTANCE_TO_TARGET = 1e-4;
  private static final double TURRET_TX;
  private static final double TURRET_TY;

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
   * moved, and is moving within a specified threshold of values. This includes minimum distance,
   * velocity, rotation, and angular velocity
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
    // If the robot has moved within a certain threshold
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
    // Has turretSubsystem.getOmega() not changed much
    boolean isTurretOmegaStable =
        Math.abs(currentTurretOmega - lastTurretOmega) <= MIN_OMEGA_TOLERANCE;

    // Check to see if all conditions are met
    if (hasNotMovedSignificantly
        && hasNotRotatedSigificantly
        && isNotMovingFastEnough
        && isTurretOmegaStable
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
    // cache the pose and chassis speeds
    lastPose = currentPose;
    lastTurretOmega = currentTurretOmega;

    // Recalcualate
    Translation2d accel = drivetrain.getAccel();
    cachedParams =
        calculate(
            driveState,
            turretSubsystem,
            accel.getX(),
            accel.getY(),
            drivetrain.getRobotRelativeAcceleration());
    return cachedParams;
  }

  /**
   * This method returns a new record of all the numbers required to shoot a ball the moment it's
   * done with one code loop. Its' calculation heavy nature comes from the turret's target angle
   * calculation. It uses newton's method (f(x)/f'(x)) to find the root, and calculate the converged
   * TOF (time of flight) iteratively. TOF Converges quickly, often within 5 iterations.
   *
   * @param SwerveDriveState the drivebase's swervedrivestate. It's only called in getParameters()
   *     and should not use any other drive state
   * @param turretSubsystem the turretSubsystem object. There should only be one instance throughout
   *     the entirety of run time
   * @return LaunchingParameters record holding all the target values. Record is defined in the
   *     LaunchCalculator class
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
    double omega_at_launch = chassisSpeeds.omegaRadiansPerSecond + (alpha * PHASE_DELAY);

    // 4. Calculate final Field-Relative Turret speeds
    // Find the turret's translation vector rotated into the field frame
    double cos = predictedAngle.getCos(), sin = predictedAngle.getSin();
    double turretOffsetFieldX = TURRET_TX * cos - TURRET_TY * sin;
    double turretOffsetFieldY = TURRET_TX * sin + TURRET_TY * cos;

    // Cross product (omega x r) to find the tangential velocity of the turret in the field frame
    double tangential_vx_field = -turretOffsetFieldY * omega_at_launch;
    double tangential_vy_field = turretOffsetFieldX * omega_at_launch;

    // Combine robot translational velocity with turret tangential velocity
    double turretVelocityX = predicted_vx_field + tangential_vx_field;
    double turretVelocityY = predicted_vy_field + tangential_vy_field;

    // Target translation
    Pose2d turretPose = estimatedPose.transformBy(turretTransform);
    Translation2d target = GetTargetFromPose.getTargetLocation(turretPose);

    // Initial distances
    double distanceX = target.getX() - turretPose.getX();
    double distanceY = target.getY() - turretPose.getY();
    double distance = Math.hypot(distanceX, distanceY);

    // Final distances
    double trueDistance = distance;
    double trueDistanceX = distanceX;
    double trueDistanceY = distanceY;
    double t = LauncherConstants.getTimeFromDistance(distance);
    for (int i = 0; i < NEWTON_METHOD_MAX_ITERATIONS; i++) {
      double prevT = t;

      // true distance is calculated by subtracting the displacement of the ball to the initial
      // calculated distance of the hub. We're essentially trying to find the distance of the ball's
      // landing spot and the hub.
      // NOTE: Drag compensation removed as it is accounted for in the interpolating map.
      trueDistanceX = distanceX - turretVelocityX * t;
      trueDistanceY = distanceY - turretVelocityY * t;
      trueDistance = Math.sqrt(trueDistanceX * trueDistanceX + trueDistanceY * trueDistanceY);

      // begin newton raphson's method to find the converged time of flight
      double lookupT = LauncherConstants.getTimeFromDistance(trueDistance);
      // calculate time error
      double f = lookupT - t;
      // Rate of change of the distance from turret to hub with respect to time (derivative of
      // (pythagorean formula: sqrt((X_distance)^2+(Y_distance)^2) )
      double dDist_Dt =
          -(trueDistanceX * turretVelocityX + trueDistanceY * turretVelocityY) / trueDistance;
      // Slope of error, or the derivative of f as instantiated above
      double fPrime = (derivativeOfTOF(trueDistance) * dDist_Dt) - 1.0;

      // If the derivative is big enough, calculate
      if (Math.abs(fPrime) > MIN_SLOPE) { // Prevent divide by zero error
        t = t - f / fPrime;
      } else {
        t = lookupT; // Fallback to fixed-point
      }

      if (Math.abs(t - prevT) < CONVERGENCE_TOLERANCE) break;
    }
    // Velocity feedforward
    double feedforwardAngularVelocity = 0;
    if (trueDistance > VFF_DIST_TOLERANCE) {
      // Calculated using the 2D cross product. We find the tangential velocity by taking the dot
      // product of our velocity vector and a vector perpendicular to our target direction (swapped
      // x and y) to get the velocity component that is tangent to the target. then divide by the
      // distance to normalize the magnitude in m/s
      double tangentialVel =
          (-trueDistanceY * turretVelocityX + trueDistanceX * turretVelocityY) / trueDistance;

      // Calculated using the standard angular velocity formula (linear velocity / radius). We
      // offset it with the robot's field angular velocity to get the true angular velocity in
      // radians per second
      feedforwardAngularVelocity =
          (tangentialVel / trueDistance) - chassisSpeeds.omegaRadiansPerSecond; // RAD/S
    }

    Rotation2d targetAngleFieldRelative;
    if (trueDistance < MIN_DISTANCE_TO_TARGET) {
      targetAngleFieldRelative = Rotation2d.kZero;
    } else {
      // We still pass dx and dy to the constructor so it stays "linked" to the vector
      targetAngleFieldRelative = new Rotation2d(trueDistanceX, trueDistanceY);
    }
    // FINAL NUMS
    double targetHood = getHoodAngle(estimatedPose, trueDistance, chassisSpeeds);
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
   * @return returns returns the derivative of the Time of flight from the interpolating map, with
   *     the change in time being 2*STEP_SIZE
   */
  public double derivativeOfTOF(double distance) {
    double min = LauncherConstants.getTimeFromDistance(distance - STEP_SIZE);
    double max = LauncherConstants.getTimeFromDistance(distance + STEP_SIZE);
    return (max - min) / (2 * STEP_SIZE);
  }

  /**
   * returns the hood angle, as defined in launcherConstants's interpolating map. Checks
   * isCloseToTrench(). If true, set it to 0 so that we don't rip our hood.
   *
   * @param lookaheadPose currentPose. This should be the robot's pose, or a point on the robot's
   *     pose
   * @param trueDist The distance from the curretPose to the hub
   * @return returns a double representing the hood angle (should be tuned in launcher constants).
   *     Returned value does not have an apparant unit.
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
      // Transform by turret transform because we are checking to see if the turret is close to the
      // trench not the center of the robot
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
