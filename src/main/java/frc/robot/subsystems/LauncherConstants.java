package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;

public class LauncherConstants {
  private static final double LAUNCHER_OFFSET_INCHES = 12;
  private static final double LAUNCHER_OFFSET_DEGREES = 135;
  private static final Translation2d LAUNCHER_OFFSET =
      new Translation2d(
          Units.inchesToMeters(LAUNCHER_OFFSET_INCHES),
          Rotation2d.fromDegrees(LAUNCHER_OFFSET_DEGREES));
  private static final StructArrayPublisher<Pose2d> turretToTarget =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("lines/turretToTarget", Pose2d.struct)
          .publish();
  private static final StructArrayPublisher<Pose2d> turretRotationalVelocity =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("lines/turretRotationalVelocity", Pose2d.struct)
          .publish();

  public static class LauncherDistanceDataPoint {
    public final double hoodAngle;
    public final double flywheelPower;
    public final double distance;
    public final double time;

    public LauncherDistanceDataPoint(
        double m_distance, double m_hoodAngle, double m_flywheelPower, double m_time) {
      this.hoodAngle = m_hoodAngle;
      this.flywheelPower = m_flywheelPower;
      this.distance = m_distance;
      this.time = m_time;
    }

    @Override
    public String toString() {
      return String.format(
          "Distance: %f, flywheelPower: %f, hoodAngle: %f, time: %f",
          distance, flywheelPower, hoodAngle, time);
    }
  }

  private static final LauncherDistanceDataPoint[] distanceData = {
    new LauncherDistanceDataPoint(2.0, 0.1, 2300, 0.7),
    new LauncherDistanceDataPoint(3.0, 0.1, 3300, 1),
    new LauncherDistanceDataPoint(4.0, 0.1, 4300, 1.3),
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
    turretToTarget.set(new Pose2d[] {Pose2d.kZero, Pose2d.kZero});
    turretRotationalVelocity.set(new Pose2d[] {Pose2d.kZero, Pose2d.kZero});
  }

  public static void update(Pose2d robot, ChassisSpeeds fieldSpeeds, Translation2d target) {
    Pose2d turret = new Pose2d(launcherFromRobot(robot), Rotation2d.kZero);
    Pose2d updatedTarget =
        new Pose2d(
            iterativeMovingShotFromFunnelClearance(robot, fieldSpeeds, target, 3),
            Rotation2d.kZero);
    Pose2d turretVelocity =
        turret.plus(new Transform2d(angularVelocity(robot, fieldSpeeds), Rotation2d.kZero));

    var array = new Pose2d[] {turret, updatedTarget};
    turretToTarget.set(array, 0);

    var array2 = new Pose2d[] {turret, turretVelocity};
    turretRotationalVelocity.set(array2, 0);
  }

  public static double getFlywheelSpeedFromDistance(double distance) {
    return flywheelMap.get(distance);
  }

  public static Translation2d launcherFromRobot(Pose2d robot) {
    Transform2d fieldRelativeLauncherOffset = new Transform2d(LAUNCHER_OFFSET, robot.getRotation());
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

  // finds angular speed using velocity = angular rotation * radius
  // radius is launcher offset from center of robot
  // then converts angular speed into tangent velocity
  public static Translation2d angularVelocity(Pose2d robot, ChassisSpeeds fieldSpeeds) {
    Translation2d angle = LAUNCHER_OFFSET.rotateBy(robot.getRotation());
    double angleVelocitySpeed = (fieldSpeeds.omegaRadiansPerSecond * LAUNCHER_OFFSET.getNorm());
    double vx = -angle.getY() * angleVelocitySpeed;
    double vy = angle.getX() * angleVelocitySpeed;
    return new Translation2d(vx, vy);
  }

  // predicts fuel landing spot based on time, robot aim, robot velocity
  public static Translation2d predictTargetPos(
      Translation2d target, ChassisSpeeds fieldSpeeds, Double timeOfFlight, Pose2d robot) {
    Translation2d angularVelocity = angularVelocity(robot, fieldSpeeds);
    double vx = fieldSpeeds.vxMetersPerSecond + angularVelocity.getX();
    double vy = fieldSpeeds.vyMetersPerSecond + angularVelocity.getY();
    double predictedX = target.getX() - vx * timeOfFlight;
    double predictedY = target.getY() - vy * timeOfFlight;
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
      predictedTarget = predictTargetPos(target, fieldSpeeds, timeOfFlight, robot);
      distance = launcherFromRobot(robot).getDistance(predictedTarget);
      timeOfFlight = getTimeFromDistance(distance);
    }

    return predictedTarget;
  }
}
