package frc.robot.util.tuning;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import frc.robot.util.robotType.RobotType;

public class LauncherConstants {
  private static final Translation2d LAUNCHER_OFFSET =
      RobotType.isAlpha() ? new Translation2d(0.2159, -0.1397) : new Translation2d(0.2159, 0.1397);

  private static final StructArrayPublisher<Pose2d> turretToTarget =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("lines/turretToTarget", Pose2d.struct)
          .publish();
  private static final StructArrayPublisher<Pose2d> turretRotationalVelocity =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("lines/turretRotationalVelocity", Pose2d.struct)
          .publish();

  private static double minTime = Double.POSITIVE_INFINITY;
  private static double maxTime = Double.NEGATIVE_INFINITY;

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

  private static final LauncherDistanceDataPoint[] alphaDistanceData = {
    new LauncherDistanceDataPoint(1.0, 0.1, 55, 0.7),
    new LauncherDistanceDataPoint(2.0, 0.3, 59, 1.3),
    new LauncherDistanceDataPoint(3.0, 0.6, 65, 1.6),
    new LauncherDistanceDataPoint(4.0, 1.2, 71, 1.9),
  };

  private static final LauncherDistanceDataPoint[] compDistanceData = {
    new LauncherDistanceDataPoint(1.31, 3, 65, 0.6),
    new LauncherDistanceDataPoint(2.52, 6, 72, 1.18),
    new LauncherDistanceDataPoint(3.32, 8, 75, 1.2),
    new LauncherDistanceDataPoint(3.87, 10, 79, 1.36),
    new LauncherDistanceDataPoint(4.27, 11, 82, 32.0 / 30.0), // pending james TOF
    new LauncherDistanceDataPoint(5.6, 12.9, 100, 36.0 / 30.0) //HOOD USED TO BE 14, LOWERED DUE TO HOOD LIMIT CHANGING FROM BREAK -TD
  };

  private static final InterpolatingDoubleTreeMap flywheelMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeMap = new InterpolatingDoubleTreeMap();

  static {
    LauncherDistanceDataPoint[] distanceData =
        RobotType.isAlpha() ? alphaDistanceData : compDistanceData;
    for (var point : distanceData) {
      flywheelMap.put(point.distance, point.flywheelPower);
      hoodMap.put(point.distance, point.hoodAngle);
      timeMap.put(point.distance, point.time);
      maxTime = Math.max(maxTime, point.time);
      minTime = Math.min(minTime, point.time);
    }
  }

  // public static void update(Pose2d robot, CommandSwerveDrivetrain driveTrain) {
  //   Pose2d turret = LaunchCalculator.estimatedPose;
  //   double turret_to_hub_dist = LaunchCalculator.estimatedDist;
  //   LaunchingParameters params = LaunchCalculator.getInstance().getParameters(driveTrain);
  //   double turretAngle = params.turretAngle().getRadians();
  //   Pose2d updatedTarget =
  //       new Pose2d(
  //           new Translation2d(
  //               turret_to_hub_dist * Math.cos(turretAngle),
  //               turret_to_hub_dist * Math.sin(turretAngle)),
  //           Rotation2d.kZero);
  //   var array = new Pose2d[] {turret, updatedTarget};
  //   turretToTarget.set(array, 0);
  // }

  public static double getFlywheelSpeedFromDistance(double distance) {
    return flywheelMap.get(distance);
  }

  public static Translation2d launcherFromRobot(Pose2d robot) {
    Transform2d fieldRelativeLauncherOffset = new Transform2d(LAUNCHER_OFFSET, Rotation2d.kZero);
    return robot.plus(fieldRelativeLauncherOffset).getTranslation();
  }

  public static double getFlywheelSpeedFromPose2d(Translation2d target, Pose2d robot) {
    double distance = launcherFromRobot(robot).getDistance(target);
    return getFlywheelSpeedFromDistance(distance);
  }

  public static Transform2d turretTransform() {
    return new Transform2d(LAUNCHER_OFFSET, Rotation2d.kZero);
  }

  public static double getHoodAngleFromDistance(double distance) {
    return hoodMap.get(distance);
  }

  public static double getHoodAngleFromPose2d(Translation2d target, Pose2d robot) {
    double distance = launcherFromRobot(robot).getDistance(target);
    return getHoodAngleFromDistance(distance);
  }

  public static double getTimeFromDistance(double distance) {
    return timeMap.get(distance);
  }

  public static double minTimeOfFlight() {
    return minTime;
  }

  public static double maxTimeOfFlight() {
    return maxTime;
  }

  // Move a target a set time in the future along a velocity defined by fieldSpeeds
  // public static Translation2d predictTargetPos(
  //     Translation2d target, ChassisSpeeds fieldSpeeds, double timeOfFlight) {
  //   double predictedX = target.getX() - fieldSpeeds.vxMetersPerSecond * timeOfFlight;
  //   double predictedY = target.getY() - fieldSpeeds.vyMetersPerSecond * timeOfFlight;}

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
