package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class LauncherConstants {
  private static final double TURRET_X_OFFSET = 0.2159;
  private static final double TURRET_Y_OFFSET = -0.1397;
  private static final Translation2d LAUNCHER_OFFSET =
      new Translation2d(TURRET_X_OFFSET, TURRET_Y_OFFSET);

  public static class LauncherDistanceDataPoint {
    public final double hoodAngle;
    public final double flywheelPower;
    public final double distance;
    public final double time;

    public LauncherDistanceDataPoint(
        double m_distanceM, double hoodAngleRot, double flywheelPowerRPS, double m_timeS) {
      this.hoodAngle = hoodAngleRot;
      this.flywheelPower = flywheelPowerRPS;
      this.distance = m_distanceM;
      this.time = m_timeS;
    }

    @Override
    public String toString() {
      return String.format(
          "Distance: %f, flywheelPower: %f, hoodAngle: %f, time: %f",
          distance, flywheelPower, hoodAngle, time);
    }
  }

  private static final LauncherDistanceDataPoint[] distanceData = {
    new LauncherDistanceDataPoint(2.05, 1, 74, 0.6),
    new LauncherDistanceDataPoint(3.1,2, 74, 1.3),
    new LauncherDistanceDataPoint(3.83, 2.5, 78, 1.7),
    new LauncherDistanceDataPoint(3.9, 2.6, 78, 1.9),
    new LauncherDistanceDataPoint(4.88, 4.5, 80, 2)
  };

  private static InterpolatingDoubleTreeMap flywheelMap = new InterpolatingDoubleTreeMap();
  private static InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();
  private static InterpolatingDoubleTreeMap timeMap = new InterpolatingDoubleTreeMap();

  static {
    for (var point : distanceData) {
      flywheelMap.put(point.distance, point.flywheelPower);
      hoodMap.put(point.distance, point.hoodAngle);
      timeMap.put(point.distance, point.time);
    }
  }

  public static double getFlywheelSpeedFromDistance(double distance) {
    return flywheelMap.get(distance);
  }

  public static Translation2d launcherFromRobot(Pose2d robot) {
    Transform2d fieldRelativeLauncherOffset = new Transform2d(LAUNCHER_OFFSET, Rotation2d.kZero);
    return robot.plus(fieldRelativeLauncherOffset).getTranslation();
  }

  public static double getFlywheelSpeedFromPose2d(Translation2d hub, Pose2d robot) {
    double distance = launcherFromRobot(robot).getDistance(hub);
    return getFlywheelSpeedFromDistance(distance);
  }

  public static double getHoodAngleFromDistance(double distance) {
    return hoodMap.get(distance);
  }

  public static double getHoodAngleFromPose2d(Translation2d hub, Pose2d robot) {
    double distance = launcherFromRobot(robot).getDistance(hub);
    return getHoodAngleFromDistance(distance);
  }

  public static double getTimeFromDistance(double distance) {
    return timeMap.get(distance);
  }

  // Move a target a set time in the future along a velocity defined by fieldSpeeds
  public static Translation2d predictTargetPos(
      Translation2d target, ChassisSpeeds fieldSpeeds, double timeOfFlight) {
    double predictedX = target.getX() - fieldSpeeds.vxMetersPerSecond * timeOfFlight;
    double predictedY = target.getY() - fieldSpeeds.vyMetersPerSecond * timeOfFlight;

    return new Translation2d(predictedX, predictedY);
  }

  public static Translation2d iterativeMovingShotFromFunnelClearance(
      Pose2d robot, ChassisSpeeds fieldSpeeds, Translation2d target, int iterations) {
    // Perform initial estimation (assuming unmoving robot) to get time of flight estimate
    double distance = launcherFromRobot(robot).getDistance(target);
    double timeOfFlight = getTimeFromDistance(distance);
    Translation2d predictedTarget = target;

    // Iterate the process, getting better time of flight estimations and updating the predicted
    // target accordingly
    for (int i = 0; i < iterations; i++) {
      predictedTarget = predictTargetPos(target, fieldSpeeds, timeOfFlight);
      distance = launcherFromRobot(robot).getDistance(predictedTarget);
      timeOfFlight = getTimeFromDistance(distance);
    }

    return predictedTarget;
  }
}
