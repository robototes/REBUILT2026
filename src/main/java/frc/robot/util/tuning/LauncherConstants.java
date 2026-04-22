package frc.robot.util.tuning;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.util.AllianceUtils;
import frc.robot.util.robotType.RobotType;

public class LauncherConstants {
  private static final Transform2d LAUNCHER_OFFSET =
      RobotType.isAlpha()
          ? new Transform2d(new Translation2d(0.2159, -0.1397), Rotation2d.kZero)
          : new Transform2d(new Translation2d(0.2159, 0.1397), Rotation2d.kZero);

  private static final NetworkTable table =
      NetworkTableInstance.getDefault().getTable("/SmartDashboard/LiveLauncherData");
  private static final StructPublisher<Pose2d> turretPose =
      table.getStructTopic("Turret Pose", Pose2d.struct).publish();
  private static final DoublePublisher turretToHubDistance =
      table.getDoubleTopic("Turret to hub distance").publish();

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
    new LauncherDistanceDataPoint(1, 1.5, 40, 1.018),
    new LauncherDistanceDataPoint(1.5, 2, 42, 1.138),
    new LauncherDistanceDataPoint(2, 3, 43, 1.104),
    new LauncherDistanceDataPoint(2.55, 3.5, 46, 1.204),
    new LauncherDistanceDataPoint(3.2, 4.5, 49, 1.15),
    new LauncherDistanceDataPoint(3.5, 4.8, 51, 1.229),
    new LauncherDistanceDataPoint(3.75, 5, 51.5, 1.217),
    new LauncherDistanceDataPoint(4.2, 5.125, 53, 1.275),
    new LauncherDistanceDataPoint(4.5, 5.5, 58, 1.305),
    new LauncherDistanceDataPoint(5, 6, 62, 1.349),
    new LauncherDistanceDataPoint(5.8, 6.8, 69, 1.378),
    new LauncherDistanceDataPoint(8, 9, 90, 1.53)
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
    return LAUNCHER_OFFSET.getTranslation();
  }

  public static void UpdateNT(Pose2d robot) {
    Pose2d result = robot.transformBy(LAUNCHER_OFFSET);
    turretPose.set(result);
    turretToHubDistance.set(
        AllianceUtils.getHubTranslation2d().minus(result.getTranslation()).getNorm());
  }

  public static double getFlywheelSpeedFromPose2d(Translation2d target, Pose2d robot) {
    double distance = launcherFromRobot(robot).getDistance(target);
    return getFlywheelSpeedFromDistance(distance);
  }

  public static Transform2d turretTransform() {
    return LAUNCHER_OFFSET;
  }

  public static double getHoodAngleFromDistance(double distance) {
    return hoodMap.get(distance /*+ distanceOffset*/);
  }

  public static double getHoodAngleFromPose2d(Translation2d target, Pose2d robot) {
    double distance = launcherFromRobot(robot).getDistance(target);
    return getHoodAngleFromDistance(distance);
  }

  public static double getTimeFromDistance(double distance) {
    return timeMap.get(distance /*+ distanceOffset*/);
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
    Translation2d angle = LAUNCHER_OFFSET.getTranslation().rotateBy(robot.getRotation());
    double angleVelocitySpeed =
        (fieldSpeeds.omegaRadiansPerSecond * LAUNCHER_OFFSET.getTranslation().getNorm());
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
