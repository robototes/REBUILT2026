package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class LauncherConstants {
  private static final double LAUNCHER_OFFSET_INCHES = 12;
  private static final double LAUNCHER_OFFSET_DEGREES = 135;
  private static final Translation2d LAUNCHER_OFFSET =
      new Translation2d(
          Units.inchesToMeters(LAUNCHER_OFFSET_INCHES),
          Rotation2d.fromDegrees(LAUNCHER_OFFSET_DEGREES));

  public static class LauncherDistanceDataPoint {
    public final double hoodAngle;
    public final double flywheelPower;
    public final double distance;
    public final double time;

    public LauncherDistanceDataPoint(
        double m_hoodAngle, double m_flywheelPower, double m_distance, double m_time) {
      this.hoodAngle = m_hoodAngle;
      this.flywheelPower = m_flywheelPower;
      this.distance = m_distance;
      this.time = m_time;
    }

    @Override
    public String toString() {
      return String.format(
          "Distance: %f, flywheelPower: %f, hoodAngle: %f", distance, flywheelPower, hoodAngle);
    }
  }

  private static final java.util.List<LauncherDistanceDataPoint> distanceData = new ArrayList<>();

  private static InterpolatingDoubleTreeMap flywheelMap = new InterpolatingDoubleTreeMap();
  private static InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();

  static {
    // add in data here
    distanceData.add(new LauncherDistanceDataPoint(0.1, 2300, 2.0, 0.7));
    distanceData.add(new LauncherDistanceDataPoint(0.1, 3300, 3.0, 1));
    distanceData.add(new LauncherDistanceDataPoint(0.1, 4300, 4.0, 1.3));

    for (var point : distanceData) {
      flywheelMap.put(point.distance, point.flywheelPower);
      hoodMap.put(point.distance, point.hoodAngle);
    }
  }

  public static double getFlywheelSpeedFromDistance(double distance) {
    return flywheelMap.get(distance);
  }

  public static double getFlywheelSpeedFromPose2d(Translation2d hub, Pose2d robot) {
    Transform2d fieldRelativeLauncherOffset = new Transform2d(LAUNCHER_OFFSET, robot.getRotation());
    robot = robot.plus(fieldRelativeLauncherOffset);
    double distance = robot.getTranslation().getDistance(hub);
    return getFlywheelSpeedFromDistance(distance);
  }

  public static double getHoodAngleFromDistance(double distance) {
    return hoodMap.get(distance);
  }

  public static double getHoodAngleFromPose2d(Translation2d hub, Pose2d robot) {
    Transform2d fieldRelativeLauncherOffset = new Transform2d(LAUNCHER_OFFSET, robot.getRotation());
    robot = robot.plus(fieldRelativeLauncherOffset);
    double distance = robot.getTranslation().getDistance(hub);
    return getHoodAngleFromDistance(distance);
  }

  // calculates how long it will take for a projectile to travel a set distance given its initial velocity and angle
   public static Time calculateTimeOfFlight(LinearVelocity exitVelocity, Angle hoodAngle, Distance distance) {
        double vel = exitVelocity.in(MetersPerSecond);
        double angle = hoodAngle.in(Radians);
        double dist = distance.in(Meters);
        return Seconds.of(dist / (vel * Math.cos(angle)));
    }
  // Move a target a set time in the future along a velocity defined by fieldSpeeds
    public static Translation2d predictTargetPos(Translation2d target, ChassisSpeeds fieldSpeeds, Time timeOfFlight) {
        double predictedX = target.getX() - fieldSpeeds.vxMetersPerSecond * timeOfFlight.in(Seconds);
        double predictedY = target.getY() - fieldSpeeds.vyMetersPerSecond * timeOfFlight.in(Seconds);

        return new Translation2d(predictedX, predictedY);
    }

    public static void iterativeMovingShotFromFunnelClearance(
            Pose2d robot, ChassisSpeeds fieldSpeeds, Translation2d target, int iterations) {
        // Perform initial estimation (assuming unmoving robot) to get time of flight estimate
         double distance = robot.getTranslation().getDistance(target);
        Time timeOfFlight = calculateTimeOfFlight(shot.getExitVelocity(), shot.getHoodAngle(), distance);
        Translation2d predictedTarget = target;

        // Iterate the process, getting better time of flight estimations and updating the predicted target accordingly
        for (int i = 0; i < iterations; i++) {
            predictedTarget = predictTargetPos(target, fieldSpeeds, timeOfFlight);
            timeOfFlight = calculateTimeOfFlight(
                    shot.getExitVelocity(), shot.getHoodAngle(), getDistanceToTarget(robot, predictedTarget));
        }

        return shot;
    }
}
