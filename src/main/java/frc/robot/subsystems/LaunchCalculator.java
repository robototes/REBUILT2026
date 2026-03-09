package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import frc.robot.util.AllianceUtils;
import frc.robot.util.GetTargetFromPose;
import frc.robot.util.tuning.LauncherConstants;
import java.util.Collection;
import java.util.List;

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

  private static int D_LOOKAHEAD_ITERATIONS = 5;

  // private double turret_target_velocity;+
  // private double hood_target_velocity;

  public record LaunchingParameters(
      boolean isValid, Rotation2d turretAngle, double hoodAngle, double flywheelSpeed) {}

  private static double minDistance;
  private static double maxDistance;
  private static double D_PHASE_DELAY = 0.05;

  // Network tables
  // private final NtTunableDouble phaseDelay;
  // private final NtTunableDouble LOOKAHEAD_ITERATIONS;

  private final DoubleSubscriber phaseDelaySub;
  private final IntegerSubscriber iterationsSub;

  private static final AprilTagFieldLayout field = AllianceUtils.FIELD_LAYOUT;
  private static final double TURRET_TO_TRENCH_TOLERANCE = Units.inchesToMeters(12);
  private static final Collection<Pose2d> trenchTags =
      List.of(
          field.getTagPose(17).get().toPose2d(),
          field.getTagPose(28).get().toPose2d(),
          field.getTagPose(22).get().toPose2d(),
          field.getTagPose(23).get().toPose2d(),
          field.getTagPose(12).get().toPose2d(),
          field.getTagPose(1).get().toPose2d(),
          field.getTagPose(7).get().toPose2d(),
          field.getTagPose(6).get().toPose2d());

  // Static initializer
  static {
    turretTransform = LauncherConstants.turretTransform();
    minDistance = 1.34;
    maxDistance = 5.60;
  }

  private LaunchCalculator() {
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

    TLDR: Where will the robot be the moment we're finished calculating everything
    */
    double phase = phaseDelaySub.get();
    estimatedPose =
        estimatedPose.exp(
            new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * phase,
                robotRelativeVelocity.vyMetersPerSecond * phase,
                robotRelativeVelocity.omegaRadiansPerSecond * phase));

    // Apply turret transform to the estimated psoe
    Pose2d turretPosition = estimatedPose.transformBy(turretTransform);

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

    // Account for imparted target
    Translation2d realTargetLocation = GetTargetFromPose.getTargetLocation(turretPosition);
    Rotation2d targetAngleFieldRelative =
        realTargetLocation.minus(turretPosition.getTranslation()).getAngle();
    Pose2d lookaheadPose = turretPosition;
    double lastFlightTime = 0;
    // Initial guess based on the actual physical distance
    double currentFlightTime =
        LauncherConstants.getTimeFromDistance(
            realTargetLocation.getDistance(lookaheadPose.getTranslation()));

    double lookaheadDist = 0;
    double currentFlywheelSpeed = 0;
    double currentHoodAngle = 0;

    int maxIters = (int) iterationsSub.getAsLong();
    int i = 0;

    while (Math.abs(currentFlightTime - lastFlightTime) > 0.001 && i < maxIters) {
      lastFlightTime = currentFlightTime;

      // 1. Move the target "upstream" to cancel robot velocity
      double virtualX = realTargetLocation.getX() - (turretVelocityX * currentFlightTime);
      double virtualY = realTargetLocation.getY() - (turretVelocityY * currentFlightTime);
      Translation2d virtualTarget = new Translation2d(virtualX, virtualY);

      // 2. Calculate the "Physics Distance" to this ghost target
      lookaheadDist = virtualTarget.getDistance(lookaheadPose.getTranslation());

      // 3. Update ALL parameters based on this corrected distance
      currentFlightTime = LauncherConstants.getTimeFromDistance(lookaheadDist);
      currentFlywheelSpeed = LauncherConstants.getFlywheelSpeedFromDistance(lookaheadDist);
      currentHoodAngle = LauncherConstants.getHoodAngleFromDistance(lookaheadDist);

      // We update angle based on the virtual target
      targetAngleFieldRelative =
          virtualTarget
              .minus(lookaheadPose.getTranslation())
              .getAngle()
              .rotateBy(Rotation2d.k180deg);

      i++;
    }

    // Assign to static variables for trench logic/logging
    LaunchCalculator.estimatedPose = estimatedPose; // The robot's predicted pose at shot-time
    LaunchCalculator.estimatedDist = lookaheadDist; // The "faked" distance the ball needs to fly

    // Final resolution
    targetTurretAngle = targetAngleFieldRelative;
    targetHoodAngle = currentHoodAngle;
    return new LaunchingParameters(
        lookaheadDist >= minDistance && lookaheadDist <= maxDistance,
        targetTurretAngle,
        targetHoodAngle,
        currentFlywheelSpeed);
  }

  public double getHoodAngle(Pose2d lookaheadPose) {
    if (isCloseToTrench()) {
      return 0;
    } else {
      return LauncherConstants.getHoodAngleFromDistance(estimatedDist);
    }
  }

  public static boolean isCloseToTrench() {
    double nearestTagX = estimatedPose.nearest(trenchTags).getX();
    if (Math.abs(nearestTagX - estimatedPose.getX()) < TURRET_TO_TRENCH_TOLERANCE) {
      return true;
    } else {
      return false;
    }
  }
}
