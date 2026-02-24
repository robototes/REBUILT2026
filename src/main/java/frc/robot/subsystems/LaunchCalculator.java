package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.AllianceUtils;
import frc.robot.util.LauncherConstants;

public class LaunchCalculator {
  private static LaunchCalculator instance;

  // These filters are here to reduce noise when grabbing turret and hood angles
  // private final LinearFilter ROC__target_turret_filter =
  //     LinearFilter.movingAverage((int) (0.1 / TimedRobot.kDefaultPeriod));
  // private final LinearFilter ROC__target_hood_filter =
  //     LinearFilter.movingAverage((int) (0.1 / TimedRobot.kDefaultPeriod));

  // Turret transform
  private final Transform2d turretTransfom =
      new Transform2d(new Translation2d(0.2159, 0.1397), Rotation2d.kZero);
  // private Rotation2d lastTurretAngle;
  // private double lastHoodAngle;
  private Rotation2d targetTurretAngle;
  private double targetHoodAngle = Double.NaN;

  // private double turret_target_velocity;
  // private double hood_target_velocity;

  public static LaunchCalculator getInstance() {
    if (instance == null) instance = new LaunchCalculator();
    return instance;
  }

  public record LaunchingParameters(
      boolean isValid,
      Rotation2d turretAngle,
      // double turret_target_velocity,
      double hoodAngle,
      // double hood_target_velocity,
      double flywheelSpeed) {}

  // Cache parametersZ
  private LaunchingParameters finalParameters = null;

  private static double minDistance;
  private static double maxDistance;
  private static double phaseDelay;

  static {
    minDistance = 1.34;
    maxDistance = 5.60;
    phaseDelay = 0.03;
  }

  public LaunchingParameters getParameters(SwerveDriveState robotState) {
    if (finalParameters != null) {
      return finalParameters;
    }

    // Calculate estimated pose while accounting for phase delay
    Pose2d estimatedPose = robotState.Pose;
    ChassisSpeeds robotRelativeVelocity = robotState.Speeds;
    /* This takes dX /s and dY /s, both multiplied by the estimated delta T (in this case it's the phase delay) to get the real dX dY for the Twist2d object.
    Twist 2d objects gives us a transformation result that tells us where the robot will end up accounting for a continuous change in rotation
    in field frame by multiplying the integral of cos and sin (integral of the x and y components with the x and y components of the velocity). Then
    */
    estimatedPose =
        estimatedPose.exp(
            new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

    // - Calculate distance from turret to target - //
    Translation2d target = AllianceUtils.getHubTranslation2d();
    Pose2d turretPosition = estimatedPose.transformBy(turretTransfom);
    // grab distance between turret and center of hub
    double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());

    // Calculate field relative turret velocity
    ChassisSpeeds robotVelocity =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeVelocity, robotState.Pose.getRotation());
    // Grab robot angle
    double robotAngle = estimatedPose.getRotation().getRadians();
    // calculate the turret's velocity field relative
    double turretVelocityX =
        robotVelocity.vxMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (turretTransfom.getY() * Math.cos(robotAngle)
                    - turretTransfom.getX() * Math.sin(robotAngle));
    double turretVelocityY =
        robotVelocity.vyMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (turretTransfom.getX() * Math.cos(robotAngle)
                    - turretTransfom.getY() * Math.sin(robotAngle));

    // Account for imparted velocity by robot (turret) to offset
    double timeOfFlight;
    Pose2d lookaheadPose = turretPosition;
    // the distance from the turret to the hub. This will be updated in the for loop
    double lookaheadTurretToTargetDistance = turretToTargetDistance;
    for (int i = 0; i < 20; i++) {
      timeOfFlight = LauncherConstants.getTimeFromDistance(lookaheadTurretToTargetDistance);
      double offsetX = turretVelocityX * timeOfFlight;
      double offsetY = turretVelocityY * timeOfFlight;
      lookaheadPose =
          new Pose2d(
              turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              turretPosition.getRotation());
      lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
    }

    // Calculate parameters accounted for imparted velocity

    // // Target turret angle
    targetTurretAngle = target.minus(lookaheadPose.getTranslation()).getAngle();
    // // Target hood angle
    targetHoodAngle = LauncherConstants.getHoodAngleFromDistance(lookaheadTurretToTargetDistance);

    // // Set last turret angle to the currrent turret Angle and the last hood angle to current hood
    // // angle
    // if (lastTurretAngle == null) lastTurretAngle = targetTurretAngle;
    // if (Double.isNaN(lastHoodAngle)) lastHoodAngle = targetHoodAngle;

    // Calculate the angular velocity of the target angle. We're using filters to eliminate noise
    // from the derivative for infinitsimal values
    // turret_target_velocity =
    //     ROC__target_turret_filter.calculate(
    //         targetTurretAngle.minus(lastTurretAngle).getRadians() / TimedRobot.kDefaultPeriod);
    // hood_target_velocity =
    //     ROC__target_hood_filter.calculate(
    //         (targetHoodAngle - lastHoodAngle) / TimedRobot.kDefaultPeriod);
    // lastTurretAngle = targetTurretAngle;
    // lastHoodAngle = targetHoodAngle;
    finalParameters =
        new LaunchingParameters(
            lookaheadTurretToTargetDistance >= minDistance
                && lookaheadTurretToTargetDistance <= maxDistance,
            targetTurretAngle,
            // turret_target_velocity,
            targetHoodAngle,
            // hood_target_velocity,
            LauncherConstants.getFlywheelSpeedFromDistance(lookaheadTurretToTargetDistance));

    // Returns a final record, that contains the targetTurretAngle, targetHood angle, and flywheel
    // speed
    // with all velocities accounted for
    return finalParameters;
  }

  public void clearLaunchingParameters() {
    finalParameters = null;
  }
}
