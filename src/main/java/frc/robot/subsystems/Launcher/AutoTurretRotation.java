package frc.robot.subsystems.Launcher;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import frc.robot.util.AllianceUtils;

public class AutoTurretRotation {
  private final TurretSubsystem turretSubsystem;
  private final CommandSwerveDrivetrain drivetrain;

  public AutoTurretRotation(TurretSubsystem turretSubsystem, CommandSwerveDrivetrain drivetrain) {
    this.turretSubsystem = turretSubsystem;
    this.drivetrain = drivetrain;
  }

  public double calculateTurretAngle() {
    // Get current robot pose
    Pose2d robotPose = drivetrain.getState().Pose;
    Translation2d robotTranslation = robotPose.getTranslation();
    Rotation2d robotRotation = robotPose.getRotation();

    // Get hub position
    Translation2d hubTranslation = AllianceUtils.getHubTranslation2d();

    // Calculate vector from robot to hub
    Translation2d robotToHub = hubTranslation.minus(robotTranslation);

    // Calculate absolute field angle to hub
    Rotation2d absoluteAngleToHub =
        new Rotation2d(Math.atan2(robotToHub.getY(), robotToHub.getX()));

    // Calculate turret angle relative to robot's forward direction
    // Subtract robot's rotation to get robot-relative angle
    Rotation2d turretAngle = absoluteAngleToHub.minus(robotRotation);

    // Convert to rotations for the turret motor
    double rotations = turretAngle.getRotations() + Units.radiansToRotations(Math.PI);

    double min = Units.degreesToRotations(0);
    double max = Units.degreesToRotations(170);

    rotations = Math.max(min, Math.min(max, rotations));

    System.out.println("Robot x " + robotTranslation.getX());
    System.out.println("Robot y " + robotTranslation.getY());
    System.out.println("Hub x " + hubTranslation.getX());
    System.out.println("Hub y " + hubTranslation.getY());
    System.out.println("Robot to hub x " + robotToHub.getX());
    System.out.println("Robot to hub y " + robotToHub.getY());
    System.out.println("absolute angle to hub " + absoluteAngleToHub.getDegrees());
    System.out.println("robot rotation " + robotRotation.getDegrees());
    System.out.println("Turret angle " + turretAngle.getDegrees());

    System.out.println("Rots" + rotations);

    return rotations;
  }

  public Command trackHub() {
    return Commands.run(
        () -> {
          double targetRotations = calculateTurretAngle();
          turretSubsystem.setTurretPositionRaw(targetRotations);
        });
  }

  public boolean isAimedAtHub(double toleranceDegrees) {
    double targetRotations = calculateTurretAngle();
    double currentRotations = turretSubsystem.getTurretPosition();
    double errorDegrees = Math.abs(Units.rotationsToDegrees(targetRotations - currentRotations));
    return errorDegrees < toleranceDegrees;
  }
}
