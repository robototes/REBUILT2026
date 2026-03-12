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

public class LaunchCalculatorV3 {
  public static class Holder {
    private static final LaunchCalculatorV3 INSTANCE = new LaunchCalculatorV3();
  }

  public static LaunchCalculatorV3 getInstance() {
    return Holder.INSTANCE;
  }

  private static final Transform2d turretTransform = LauncherConstants.turretTransform();

  private static final double DRAG_COEFFICIENT = 0.48;
  private static final double PHASE_DELAY = 0.02;
  private static final double CONVERGENCE_TOLERANCE = 0.001;
  private static final double DRAG_TOLERANCE = 1e-6;
  private static final double STEP_SIZE = 0.01; // Instantaneous rate of change step size in meters
  private static final double MIN_SLOPE = 1e-4;

  // Trench stuff
  private static final AprilTagFieldLayout field = AllianceUtils.FIELD_LAYOUT;
  private static final double TURRET_TO_TRENCH_TOLERANCE = Units.inchesToMeters(12);
  private static final ArrayList<Pose2d> trenchTags = new ArrayList<>();
  private static int[] tags = {1, 6, 7, 12, 17, 22, 23, 28};

  // Network tables
  private static NetworkTableEntry tl;
  private static NetworkTableEntry cl;

  public record LaunchingParameters(
      double targetHood,
      Rotation2d targetTurret,
      double targetFlywheels,
      double targetTurretFeedforward) {}

  static {
    NetworkTable limelightNTEntry = NetworkTableInstance.getDefault().getTable("limelight");
    tl = limelightNTEntry.getEntry("tl");
    cl = limelightNTEntry.getEntry("cl");

    for (int tag : tags) {
      Pose2d tagpose =
          field
              .getTagPose(tag)
              .orElseThrow(() -> new RuntimeException("Tag " + tag + " not found in field layout"))
              .toPose2d();
      trenchTags.add(tagpose);
    }
  }

  public LaunchingParameters getParameters(CommandSwerveDrivetrain driveTrain) {

    // Take the pose when vision was captured rather than the instateneous pose of the robot
    double visionLatencySeconds =
        (tl.getDouble(0) + cl.getDouble(0))
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
    ChassisSpeeds fieldVel = ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeeds, robotAngle);
    // Turret velocity's x and y velocities field relative
    double turretVelocityX =
        fieldVel.vxMetersPerSecond
            - fieldVel.omegaRadiansPerSecond
                * (turretTransform.getY() * Math.cos(robotAngle.getRadians())
                    + turretTransform.getX() * Math.sin(robotAngle.getRadians()));
    double turretVelocityY =
        fieldVel.vyMetersPerSecond
            + fieldVel.omegaRadiansPerSecond
                * (turretTransform.getX() * Math.cos(robotAngle.getRadians())
                    - turretTransform.getY() * Math.sin(robotAngle.getRadians()));

    // Target translation
    Pose2d turretPose = estimatedPose.transformBy(turretTransform);
    Translation2d target = GetTargetFromPose.getTargetLocation(turretPose);

    double initialTurretToTarget = target.getDistance(turretPose.getTranslation());
    double distanceX = target.getX() - turretPose.getX();
    double distanceY = target.getY() - turretPose.getY();
    double distance = Math.hypot(distanceX, distanceY);

    double trueDistance = distance;
    double t = LauncherConstants.getTimeFromDistance(initialTurretToTarget);
    for (int i = 0; i < 5; i++) {
      double prevT = t;

      double driftT = getDragCompensatedTOF(t);
      double trueDistanceX = distanceX - turretVelocityX * driftT;
      double trueDistanceY = distanceY - turretVelocityY * driftT;
      trueDistance = Math.hypot(trueDistanceX, trueDistanceY);

      double lookupT = LauncherConstants.getTimeFromDistance(trueDistance);
      double f = lookupT - t;
      // Rate of change of the distance from turret to hub (derivative of pythagorean formula)
      double dDist_Dt =
          -(trueDistanceX * turretVelocityX + trueDistanceY * turretVelocityY) / trueDistance;
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
    if (trueDistance > 0.1) {
      // Calculated using the 2D cross product. We find the tangential velocity by taking the dot
      // product of our velocity vector and a vector perpendicular to our target direction (swapped
      // x and y), then dividing by the distance to normalize the magnitude.
      double tangentialVel =
          (-distanceY * turretVelocityX + distanceX * turretVelocityY) / trueDistance;
      // Calculated using the standard angular velocity formula, by dividing by the distance. We
      // offset it with the robot's field angular velocity to get the true angular velocity
      feedforwardAngularVelocity = (tangentialVel / trueDistance) - fieldVel.omegaRadiansPerSecond;
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

    // IF feedforward is 0, just use standard tracking. If not then
    // use this number
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
   * DRAG TOLERANCE SHOULD NOT BE 0. This is using the low-speed fomrula for displacement due to
   * drag. It returns the TOF accounting in drag
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

  public static boolean isCloseToTrench(Pose2d pose) {
    double nearestTagX = pose.nearest(trenchTags).getX();
    if (Math.abs(nearestTagX - pose.getX()) < TURRET_TO_TRENCH_TOLERANCE) {
      return true;
    } else {
      return false;
    }
  }
}
