package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;

public class LauncherConstants {
  // distance from any edge of the hub to center = 0.5969m, add to any measured distance

  //changed to represent front of bumper
  private static final double LAUNCHER_OFFSET_INCHES = 18.375;
  private static final double LAUNCHER_OFFSET_DEGREES = 0;

  private static final Translation2d LAUNCHER_OFFSET =
      new Translation2d(
          Units.inchesToMeters(LAUNCHER_OFFSET_INCHES),
          Rotation2d.fromDegrees(LAUNCHER_OFFSET_DEGREES));

  public static class LauncherDistanceDataPoint {
    public final double hoodAngle;
    public final double flywheelPower;
    public final double distance;

    public LauncherDistanceDataPoint(
        double m_hoodAngle, double m_flywheelPower, double m_distance) {
      this.hoodAngle = m_hoodAngle;
      this.flywheelPower = m_flywheelPower;
      this.distance = m_distance;
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
    distanceData.add(new LauncherDistanceDataPoint(0, 55, 1.1429));
    distanceData.add(new LauncherDistanceDataPoint(1.5, 60, 3.0569));
    distanceData.add(new LauncherDistanceDataPoint(1.8, 69, 4.5589));

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
}
