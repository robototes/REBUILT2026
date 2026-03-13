package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import frc.robot.util.AllianceUtils;
import frc.robot.util.GetTargetFromPose;
import frc.robot.util.tuning.LauncherConstants;
import java.util.ArrayList;
import java.util.List;

public class LaunchCalculator {
  private static class Holder {
    private static final LaunchCalculator INSTANCE = new LaunchCalculator();
  }

  public static LaunchCalculator getInstance() {
    return Holder.INSTANCE;
  }

  // Transforms and pose2ds
  private static final Transform2d turretTransform = LauncherConstants.turretTransform();
  private static Pose2d staticEstimatedPose;

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

  // Network tables
  private NetworkTableEntry NT_piplineLatency; // latency from vision pipeline
  private NetworkTableEntry
      NT_captureLatency; // time between end of sensor exposure and beginning of processing pipeline

  public record LaunchingParameters(
      double targetHood,
      Rotation2d targetTurret,
      double targetFlywheels,
      double targetTurretFeedforward) {}

  static {
    for (int tag : tags) {
      Pose2d tagpose =
          field
              .getTagPose(tag)
              .orElseThrow(() -> new RuntimeException("Tag " + tag + " not found in field layout"))
              .toPose2d();
      trenchTags.add(tagpose);
    }
  }

  private LaunchCalculator() {
    NetworkTable limelightNTEntry = NetworkTableInstance.getDefault().getTable("limelight");
    NT_piplineLatency = limelightNTEntry.getEntry("tl");
    NT_captureLatency = limelightNTEntry.getEntry("cl");
  }

  // ------ MAIN LOGIC ------ //
  public LaunchingParameters getParameters(CommandSwerveDrivetrain driveTrain) {

    // Take the pose when vision was captured rather than the instantaneous pose of the robot
    double visionLatencySeconds =
        (NT_piplineLatency.getDouble(0) + NT_captureLatency.getDouble(0))
            / 1000.0; // adds pipeline latency and capture latency to get to total latency
    double now = Timer.getFPGATimestamp();
    double captureTime = now - visionLatencySeconds;

    // Grab the EXACT pose the robot had when the camera got the frame
    Pose2d instantPose = driveTrain.getPoseAtTimeStamp(captureTime);

    // Predicted robot pose after calculations have finished
    ChassisSpeeds chassisSpeeds = driveTrain.getState().Speeds;
    Pose2d estimatedPose =
        instantPose.exp(
            new Twist2d(
                chassisSpeeds.vxMetersPerSecond * PHASE_DELAY,
                chassisSpeeds.vyMetersPerSecond * PHASE_DELAY,
                chassisSpeeds.omegaRadiansPerSecond * PHASE_DELAY));
    Rotation2d robotAngle = estimatedPose.getRotation();
    // Turret VX robot relative is calculated using this formula: V_x_point = V_x_center + (-omega *
    // offset_y)
    // Turret VY robot relative is calculated using this formula: V_y_point= V_y_center + (omega *
    // offset_x)
    ChassisSpeeds turretRobotRelativeSpeeds =
        new ChassisSpeeds(
            chassisSpeeds.vxMetersPerSecond
                - chassisSpeeds.omegaRadiansPerSecond * turretTransform.getY(),
            chassisSpeeds.vyMetersPerSecond
                + chassisSpeeds.omegaRadiansPerSecond * turretTransform.getX(),
            chassisSpeeds.omegaRadiansPerSecond);
    // Let chassisspeeds built in methods handle the conversion from robot relative to field
    // relative
    ChassisSpeeds turretFieldRelativeSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(turretRobotRelativeSpeeds, robotAngle);
    double turretVelocityX = turretFieldRelativeSpeeds.vxMetersPerSecond;
    double turretVelocityY = turretFieldRelativeSpeeds.vyMetersPerSecond;

    // Target translation
    Pose2d turretPose = estimatedPose.transformBy(turretTransform);
    staticEstimatedPose = turretPose;
    Translation2d target = GetTargetFromPose.getTargetLocation(turretPose);

    // Initial distances
    double initialTurretToTarget = target.getDistance(turretPose.getTranslation());
    double distanceX = target.getX() - turretPose.getX();
    double distanceY = target.getY() - turretPose.getY();
    double distance = Math.hypot(distanceX, distanceY);

    // Final distances
    double trueDistance = distance;
    double trueDistanceX = distanceX;
    double trueDistanceY = distanceY;
    double t = LauncherConstants.getTimeFromDistance(initialTurretToTarget);
    for (int i = 0; i < NEWTON_METHOD_MAX_ITERATIONS; i++) {
      double prevT = t;

      double driftT = getDragCompensatedTOF(t);
      trueDistanceX = distanceX - turretVelocityX * driftT;
      trueDistanceY = distanceY - turretVelocityY * driftT;
      trueDistance = Math.hypot(trueDistanceX, trueDistanceY);

      double lookupT = LauncherConstants.getTimeFromDistance(trueDistance);
      double f = lookupT - t;
      // Rate of change of the distance from turret to hub (derivative of pythagorean formula)
      double dDist_Dt =
          -(trueDistanceX * turretVelocityX + trueDistanceY * turretVelocityY) / trueDistance;
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
      // x and y), then dividing by the distance to normalize the magnitude.
      double tangentialVel =
          (-trueDistanceY * turretVelocityX + trueDistanceX * turretVelocityY) / trueDistance;
      // Calculated using the standard angular velocity formula, by dividing by the distance. We
      // offset it with the robot's field angular velocity to get the true angular velocity
      feedforwardAngularVelocity =
          (tangentialVel / trueDistance) - turretFieldRelativeSpeeds.omegaRadiansPerSecond;
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
        targetHood, targetTurret, targetFlywheels, feedforwardAngularVelocity);
  }

  /**
   * returns the derivative of the Time of flight from the interpolating map
   *
   * @param distance instantaneous distance
   */
  public double derivativeOfTOF(double distance) {
    double min = LauncherConstants.getTimeFromDistance(distance - STEP_SIZE);
    double max = LauncherConstants.getTimeFromDistance(distance + STEP_SIZE);
    return (max - min) / (2 * STEP_SIZE);
  }

  /**
   * This is using the low-speed fomrula for displacement due to drag. It returns the TOF accounting
   * in drag
   */
  private double getDragCompensatedTOF(double tof) {
    double newTOF = (1.0 - Math.exp(-DRAG_COEFFICIENT * tof)) / DRAG_COEFFICIENT;
    if (Math.abs(newTOF - tof) < DRAG_TOLERANCE) {
      return tof;
    } // if really near zero, just return parameter
    else {
      return newTOF;
    }
  }

  public double getHoodAngle(Pose2d lookaheadPose, double trueDist) {
    if (isCloseToTrench(lookaheadPose)) {
      return 0;
    } else {
      return LauncherConstants.getHoodAngleFromDistance(trueDist);
    }
  }

  /** compares X net of the 2 poses */
  public static boolean isCloseToTrench(Pose2d pose) {
    double nearestTagX = pose.nearest(trenchTags).getX();
    return Math.abs(nearestTagX - pose.getX()) < TURRET_TO_TRENCH_TOLERANCE;
  }

  public static Pose2d getEstTurretPose() {
    return staticEstimatedPose;
  }
}
