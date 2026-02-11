package frc.robot.util;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Robot;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;

public class ShotCalculator {
  private static ShotCalculator instance;
  private static CommandSwerveDrivetrain drive = null;

  // Offset from robot center to turret center (leave zero if turret is centered)
  private static final Transform2d robotToTurret = new Transform2d();

  public static ShotCalculator getInstance(CommandSwerveDrivetrain drive) {
    ShotCalculator.drive = drive;
    if (instance == null)
      instance = new ShotCalculator();
    return instance;
  }

  public record ShootingParameters(
      Rotation2d turretAngle,
      double flywheelSpeed) {
  }

  private ShootingParameters latestParameters = null;

  private static double phaseDelay;

  private static final InterpolatingDoubleTreeMap shotFlywheelSpeedMap = new InterpolatingDoubleTreeMap();

  private static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

  public static final double STARTING_FLYWHEEL_SPEED_OFFSET = 0;
  public static double FLYWHEEL_SPEED_OFFSET = STARTING_FLYWHEEL_SPEED_OFFSET;

  public static final double STARTING_TURRET_ANGLE_OFFSET_DEGREES = 0;
  public static double TURRET_ANGLE_OFFSET_DEGREES = STARTING_TURRET_ANGLE_OFFSET_DEGREES;

  public static void increaseFlywheelSpeedOffset() {
    FLYWHEEL_SPEED_OFFSET += 1;
  }

  public static void decreaseFlywheelSpeedOffset() {
    FLYWHEEL_SPEED_OFFSET -= 1;
  }

  public static void increaseTurretAngleOffsetDegrees() {
    TURRET_ANGLE_OFFSET_DEGREES += 1;
  }

  public static void decreaseTurretAngleOffsetDegrees() {
    TURRET_ANGLE_OFFSET_DEGREES -= 1;
  }

  static {
    phaseDelay = 0.03;

    shotFlywheelSpeedMap.put(1.34, 210.0);
    shotFlywheelSpeedMap.put(1.78, 220.0);
    shotFlywheelSpeedMap.put(2.17, 220.0);
    shotFlywheelSpeedMap.put(2.81, 230.0);
    shotFlywheelSpeedMap.put(3.82, 250.0);
    shotFlywheelSpeedMap.put(4.09, 255.0);
    shotFlywheelSpeedMap.put(4.40, 260.0);
    shotFlywheelSpeedMap.put(4.77, 265.0);
    shotFlywheelSpeedMap.put(5.57, 275.0);
    shotFlywheelSpeedMap.put(5.60, 290.0);

    timeOfFlightMap.put(5.68, 1.16);
    timeOfFlightMap.put(4.55, 1.12);
    timeOfFlightMap.put(3.15, 1.11);
    timeOfFlightMap.put(1.88, 1.09);
    timeOfFlightMap.put(1.38, 0.90);
  }

  public ShootingParameters getParameters() {
    if (latestParameters != null) {
      return latestParameters;
    }

    // Target location on the field
    //boolean feed = RobotStates.robotInFeedZone.getAsBoolean();
    Translation2d target = AllianceUtils.getHubTranslation2d();

    // Calculate estimated pose while accounting for phase delay
    Pose2d robotPose = drive.getState().Pose;
    ChassisSpeeds robotRelativeVelocity = drive.getState().Speeds;
    robotPose = robotPose.exp(
        new Twist2d(
            robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
            robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
            robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

    // Calculate distance from turret to target
    Pose2d turretPosition = robotPose.transformBy(robotToTurret);
    double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());

    // Calculate field relative turret velocity
    ChassisSpeeds robotVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeVelocity,
        drive.getState().Pose.getRotation());
    double robotAngle = robotPose.getRotation().getRadians();
    double turretVelocityX = robotVelocity.vxMetersPerSecond
        + robotVelocity.omegaRadiansPerSecond
            * (robotToTurret.getY() * Math.cos(robotAngle)
                - robotToTurret.getX() * Math.sin(robotAngle));
    double turretVelocityY = robotVelocity.vyMetersPerSecond
        + robotVelocity.omegaRadiansPerSecond
            * (robotToTurret.getX() * Math.cos(robotAngle)
                - robotToTurret.getY() * Math.sin(robotAngle));

    // Account for imparted velocity by robot (turret) to offset
    double timeOfFlight;
    Pose2d lookaheadPose = turretPosition;
    double lookaheadTurretToTargetDistance = turretToTargetDistance;
    for (int i = 0; i < 20; i++) {
      timeOfFlight = timeOfFlightMap.get(lookaheadTurretToTargetDistance);
      double offsetX = turretVelocityX * timeOfFlight;
      double offsetY = turretVelocityY * timeOfFlight;
      lookaheadPose = new Pose2d(
          turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
          turretPosition.getRotation());
      lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
    }
    // Calculate parameters accounted for imparted velocity
    Rotation2d turretAngle = target.minus(lookaheadPose.getTranslation()).getAngle();
    double flywheelSpeed = shotFlywheelSpeedMap.get(lookaheadTurretToTargetDistance);
    turretAngle = turretAngle.plus(Rotation2d.fromDegrees(TURRET_ANGLE_OFFSET_DEGREES));
    flywheelSpeed += flywheelSpeed * (FLYWHEEL_SPEED_OFFSET / 100.0);

    latestParameters = new ShootingParameters(
        turretAngle,
        flywheelSpeed);
    return latestParameters;
  }

  public void clearShootingParameters() {
    latestParameters = null;
  }
}
