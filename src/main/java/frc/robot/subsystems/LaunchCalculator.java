package frc.robot.subsystems;

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
import frc.robot.subsystems.launcher.TurretSubsystem;
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
  private static LaunchingParameters cachedParams;
  private static ChassisSpeeds lastSpeeds = new ChassisSpeeds();
  private static Pose2d lastPose = new Pose2d();

  // Throttling Magic numbers
  private static final double MIN_DIST_TOLERANCE = Units.inchesToMeters(3); // Meters
  private static final double MIN_ROTATION_TOLERANCE = Units.degreesToRadians(1); // Radians
  private static final double MIN_VELOCITY_TOLERANCE = Units.inchesToMeters(2); // M/s

  // Transforms and pose2ds
  private static final Transform2d turretTransform = LauncherConstants.turretTransform();

  private static final double DRAG_COEFFICIENT = 0.48;
  private static final double PHASE_DELAY = 0.02;
  private static final double CONVERGENCE_TOLERANCE = 0.001;
  private static final double DRAG_TOLERANCE =
      1e-3; // Drag tolerance. This should never really be used anyways because our interpolating
  // map isn't accurate enough to have a delta drag of near 0, but it's just here to future proof
  private static final double STEP_SIZE = 0.01; // Instantaneous rate of change step size in meters
  private static final double MIN_SLOPE = 1e-4;
  private static final int NEWTON_METHOD_MAX_ITERATIONS = 5;
  private static final double VFF_DIST_TOLERANCE = 0.1;

  // Trench stuff
  private static final AprilTagFieldLayout field = AllianceUtils.FIELD_LAYOUT;
  private static final double TURRET_TO_TRENCH_TOLERANCE = Units.inchesToMeters(12);
  private static final List<Pose2d> trenchTags = new ArrayList<>();
  private static final int[] tags = {1, 6, 7, 12, 17, 22, 23, 28}; // Trench tags

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
    Pose2d currentPose = drivetrain.getState().Pose;
    ChassisSpeeds currentSpeeds = driveState.Speeds;
    // If the robot has moved within a certain threshold
    boolean hasMovedSignificantly =
        Math.abs(currentPose.getTranslation().getDistance(lastPose.getTranslation()))
            <= MIN_DIST_TOLERANCE;
    boolean hasRotatedSigificantly =
        Math.abs(currentPose.getRotation().getRadians() - lastPose.getRotation().getRadians())
            <= MIN_ROTATION_TOLERANCE;
    boolean isMovingFastEnough =
        Math.abs(currentSpeeds.vxMetersPerSecond - lastSpeeds.vxMetersPerSecond)
                >= MIN_VELOCITY_TOLERANCE
            && Math.abs(currentSpeeds.vyMetersPerSecond - lastSpeeds.vyMetersPerSecond)
                >= MIN_VELOCITY_TOLERANCE
            && Math.abs(currentSpeeds.omegaRadiansPerSecond - lastSpeeds.omegaRadiansPerSecond)
                >= MIN_ROTATION_TOLERANCE;
    // Check to see if all conditions are met
    if (hasMovedSignificantly
        && hasRotatedSigificantly
        && isMovingFastEnough
        && cachedParams != null) {
      return cachedParams;
    }
    // cache the pose and chassis speeds
    lastPose = currentPose;
    lastSpeeds = currentSpeeds;

    // Recalcualate
    LaunchingParameters cachedParams = calculate(drivetrain, turretSubsystem);
    return cachedParams;
  }

  /**
   * This method returns a new record of all the numbers calculation heavy nature comes from the
   * turret's target angle calculation. It uses newton's method (f(x)/f'(x)) to find the root, and
   * calculate the converged TOF (time of flight) iteratively. TOF Converges quickly, often within 5
   * iterations.
   *
   * @param drivetrain the drivebase's CommandSwerveDrivetrain object
   * @param turretSubsystem the turretSubsystem object. There should only be one instance throughout
   *     the entirety of run time
   * @return LaunchingParameters record holding all the target values. Record is defined in the
   *     LaunchCalculator class
   */
  public LaunchingParameters calculate(
      CommandSwerveDrivetrain driveTrain, TurretSubsystem turretSubsystem) {

    // Grab current pose
    Pose2d estimatedPose = driveTrain.getState().Pose;

    // Predicted robot pose after calculations have finished
    ChassisSpeeds chassisSpeeds = driveTrain.getState().Speeds;
    estimatedPose =
        estimatedPose.exp(
            new Twist2d(
                chassisSpeeds.vxMetersPerSecond * PHASE_DELAY,
                chassisSpeeds.vyMetersPerSecond * PHASE_DELAY,
                chassisSpeeds.omegaRadiansPerSecond * PHASE_DELAY));
    Rotation2d robotAngle = estimatedPose.getRotation();
    // Turret VX robot relative is calculated using this formula: V_x_point = V_x_center + (-omega *
    // offset_y)
    // Turret VY robot relative is calculated using this formula: V_y_point= V_y_center + (omega *
    // offset_x)
    double totalOmega = chassisSpeeds.omegaRadiansPerSecond + turretSubsystem.getOmega();
    ChassisSpeeds turretRobotRelativeSpeeds =
        new ChassisSpeeds(
            chassisSpeeds.vxMetersPerSecond
                - chassisSpeeds.omegaRadiansPerSecond * turretTransform.getY(),
            chassisSpeeds.vyMetersPerSecond
                + chassisSpeeds.omegaRadiansPerSecond * turretTransform.getX(),
            totalOmega);
    // Let chassisspeeds built in methods handle the conversion from robot relative to field
    // relative
    ChassisSpeeds turretFieldRelativeSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(turretRobotRelativeSpeeds, robotAngle);
    double turretVelocityX = turretFieldRelativeSpeeds.vxMetersPerSecond;
    double turretVelocityY = turretFieldRelativeSpeeds.vyMetersPerSecond;

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

      // Get the true time
      double driftT = getDragCompensatedTOF(t);
      // true distance is calculated by substracting the displacement of the ball to the initial
      // calculated distance of the hub. We're essentially trying to find the distance of the ball's
      // landing spot and the hub.
      trueDistanceX = distanceX - turretVelocityX * driftT;
      trueDistanceY = distanceY - turretVelocityY * driftT;
      trueDistance = Math.hypot(trueDistanceX, trueDistanceY);

      // begin newton raphson's method to find the converged time of flight
      double lookupT = LauncherConstants.getTimeFromDistance(trueDistance);
      // calculate time error
      double f = lookupT - t;
      // Rate of change of the distance from turret to hub with respect to time (derivative of
      // (pythagorean formula: sqrt((X_distance)^2+(Y_distance)^2) )
      double dDist_Dt =
          -(trueDistanceX * turretVelocityX + trueDistanceY * turretVelocityY) / trueDistance;
      // Slope of error, or the derivative of f as instantiated above
      double fPrime =
          (derivativeOfTOF(trueDistance) * dDist_Dt * Math.exp(-DRAG_COEFFICIENT * t)) - 1.0;

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

    double finalDrift = getDragCompensatedTOF(t);
    Translation2d virtualTarget =
        new Translation2d(
            target.getX() - turretVelocityX * finalDrift,
            target.getY() - turretVelocityY * finalDrift);

    Rotation2d targetAngleFieldRelative =
        virtualTarget.minus(turretPose.getTranslation()).getAngle();

    // FINAL NUMS
    double targetHood = getHoodAngle(turretPose, trueDistance);
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
   * This is using the low-speed formula for displacement due to linear drag. It returns the
   * effective TOF accounting in drag. Derived from -b*v = m*a, where b equals the drag coefficient,
   * a equals acceleration, v equals velocity and m equals mass. both equal the net force. After
   * setting a = dV/dT, integrating both sides and exponentiate with base e, you get v_initial *
   * e^((-b/m)*t) to get the velocity as a function of time. Integrate to get a position as a
   * function of time. remove initial velocity from v_intial*time and we result with a drag
   * compensated function of time
   *
   * @param tof instantaneous TOF, without drag
   * @return new TOF with drag compensation
   */
  private double getDragCompensatedTOF(double tof) {
    double newTOF = (1.0 - Math.exp(-DRAG_COEFFICIENT * tof)) / DRAG_COEFFICIENT;
    return (Math.abs(newTOF - tof) < DRAG_TOLERANCE)
        ? tof
        : // if really near zero, just return parameter
        newTOF;
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
  public double getHoodAngle(Pose2d lookaheadPose, double trueDist) {
    return isCloseToTrench(lookaheadPose)
        ? 0
        : LauncherConstants.getHoodAngleFromDistance(trueDist);
  }

  /**
   * Checks to see if a point's x value (in an x,y field) on the robot is close to the trench or
   * not.
   *
   * @param pose the point (pose2d) on the robot you would like to check
   * @return True if close, false if not close
   */
  public static boolean isCloseToTrench(Pose2d pose) {
    double nearestTagX = pose.nearest(trenchTags).getX();
    return Math.abs(nearestTagX - pose.getX()) < TURRET_TO_TRENCH_TOLERANCE;
  }
}
