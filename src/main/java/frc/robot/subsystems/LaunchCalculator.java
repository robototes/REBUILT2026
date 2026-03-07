package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import frc.robot.util.GetTargetFromPose;
import frc.robot.util.tuning.LauncherConstants;

public class LaunchCalculator {
  private static class Holder {
    private static final LaunchCalculator INSTANCE = new LaunchCalculator();
  }

  public static LaunchCalculator getInstance() {
    return Holder.INSTANCE;
  }

  public static Pose2d estimatedPose;
  public static double estimatedDist;

  // Turret transform
  private static final Transform2d turretTransform;
  // private Rotation2d lastTurretAngle;
  // private double lastHoodAngle;
  private Rotation2d targetTurretAngle;
  private double targetHoodAngle = Double.NaN;

  private static int D_LOOKAHEAD_ITERATIONS = 20;

  // private double turret_target_velocity;+
  // private double hood_target_velocity;

  public record LaunchingParameters(
      boolean isValid, Rotation2d turretAngle, double hoodAngle, double flywheelSpeed) {}

  private static double minDistance;
  private static double maxDistance;
  private static double D_PHASE_DELAY = 0.03;

  // Network tables
  // private final NtTunableDouble phaseDelay;
  // private final NtTunableDouble LOOKAHEAD_ITERATIONS;

  private static final DoubleSubscriber phaseDelaySub;
  private static final IntegerSubscriber iterationsSub;

  // Static initializer
  static {
    turretTransform = LauncherConstants.turretTransform();
    minDistance = 1.34;
    maxDistance = 5.60;
    // Get the table once
    NetworkTable table = NetworkTableInstance.getDefault().getTable("SmartDashboard");
    phaseDelaySub = table.getDoubleTopic("phaseDelay").subscribe(D_PHASE_DELAY);
    iterationsSub = table.getIntegerTopic("Lookahead iterations").subscribe(D_LOOKAHEAD_ITERATIONS);
  }

  public LaunchingParameters getParameters(CommandSwerveDrivetrain robotState) {

    // Calculate estimated pose while accounting for phase delay
    Pose2d estimatedPose = robotState.getState().Pose;
    ChassisSpeeds robotRelativeVelocity = robotState.getState().Speeds;
    /* This takes dX /s and dY /s, both multiplied by the estimated delta T (in this case it's the phase delay) to get the real dX dY for the Twist2d object.
    Twist 2d objects gives us a transformation result that tells us where the robot will end up (individually by each component). We then integrate the velocity of the x and
    y components (robot relative) from 0 to delta T. This gives us a delta X and delta Y, which we will then apply to the previous robot pose to get a new pose2d that
    accurately represents the robot's position accounting in for angular velocity.
    */
    double phase = phaseDelaySub.get();
    estimatedPose =
        estimatedPose.exp(
            new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * phase,
                robotRelativeVelocity.vyMetersPerSecond * phase,
                robotRelativeVelocity.omegaRadiansPerSecond * phase));

    // - Calculate distance from turret to target - //
    Translation2d target = GetTargetFromPose.getTargetLocation(estimatedPose);
    Pose2d turretPosition = estimatedPose.transformBy(turretTransform);
    LaunchCalculator.estimatedPose = turretPosition;
    // grab distance between turret and center of hub
    double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());

    // Calculate field relative turret velocity
    ChassisSpeeds robotVelocity =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeVelocity, estimatedPose.getRotation());
    // Grab robot angle
    double robotAngle = estimatedPose.getRotation().getRadians();
    // calculate the turret's tangetial velocity field relative.
    // take the turret's field relative position function, take the derivative of it and plug in
    // your velocities.
    double turretVelocityX =
        robotVelocity.vxMetersPerSecond
            - robotVelocity.omegaRadiansPerSecond
                * (turretTransform.getY() * Math.cos(robotAngle)
                    + turretTransform.getX() * Math.sin(robotAngle));
    double turretVelocityY =
        robotVelocity.vyMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (turretTransform.getX() * Math.cos(robotAngle)
                    - turretTransform.getY() * Math.sin(robotAngle));

    // Account for imparted velocity by robot (turret) to offset
    double timeOfFlight;
    Pose2d lookaheadPose = turretPosition;
    // the distance from the turret to the hub. This will be updated in the for loop
    double lookaheadTurretToTargetDistance = turretToTargetDistance;

    double iterations = (int) iterationsSub.getAsLong();
    for (int i = 0; i < iterations; i++) {
      timeOfFlight = LauncherConstants.getTimeFromDistance(lookaheadTurretToTargetDistance);
      double offsetX = turretVelocityX * timeOfFlight;
      double offsetY = turretVelocityY * timeOfFlight;
      lookaheadPose =
          new Pose2d(
              turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              turretPosition.getRotation());
      lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
    }
    LaunchCalculator.estimatedDist = lookaheadTurretToTargetDistance;
    // Calculate parameters accounted for imparted velocity

    // // Target turret angle robot relative
    Rotation2d targetAngleFieldRelative = target.minus(lookaheadPose.getTranslation()).getAngle();
    targetTurretAngle =
        targetAngleFieldRelative.minus(lookaheadPose.getRotation()).rotateBy(Rotation2d.k180deg);
    // // Target hood angle
    targetHoodAngle = LauncherConstants.getHoodAngleFromDistance(lookaheadTurretToTargetDistance);
    // System.out.println(targetTurretAngle.getDegrees());

    // Returns a final record, that contains the targetTurretAngle, targetHood angle, and flywheel
    // speed
    // with all velocities accounted for
    return new LaunchingParameters(
        lookaheadTurretToTargetDistance >= minDistance
            && lookaheadTurretToTargetDistance <= maxDistance,
        targetTurretAngle,
        targetHoodAngle,
        LauncherConstants.getFlywheelSpeedFromDistance(lookaheadTurretToTargetDistance));
  }
}
