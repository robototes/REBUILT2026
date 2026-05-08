package frc.robot.subsystems.auto;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Controls;
import frc.robot.Robot;
import frc.robot.Subsystems;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeMode;
import frc.robot.util.simulation.RobotSim;

public class BLineLogic {
  private static Subsystems s;

  private static FollowPath.Builder pathBuilder;
  private static Path myPath;

  public static void init(Subsystems subsystems) {

    s = subsystems;
    registerCommands();
    myPath = new Path("test1");
    s.drivebaseSubsystem.resetPose(getStartPose());
  }

  // 2. Create a reusable path builder
  public static void configure(Subsystems s) {

    pathBuilder =
        new FollowPath.Builder(
                s.drivebaseSubsystem,

                // pose supplier
                () -> s.drivebaseSubsystem.getState().Pose,

                // robot-relative speeds supplier
                () -> s.drivebaseSubsystem.getState().Speeds,

                // robot-relative speeds consumer
                (speeds) -> {
                  s.drivebaseSubsystem.setControl(
                      new SwerveRequest.ApplyRobotSpeeds()
                          .withSpeeds(ChassisSpeeds.discretize(speeds, 0.020)));
                },

                // translation PID
                new PIDController(3.0, 0.0, 0.0),

                // rotation PID
                new PIDController(5.0, 0.0, 0.0),

                // cross-track PID
                new PIDController(2.0, 0.0, 0.0))
            .withDefaultShouldFlip()
            .withPoseReset(s.drivebaseSubsystem::resetPose);
  }
  ;

  public static Command runAuto() {

    return pathBuilder.build(myPath);
    // ... rest of auto

  }

  public static Pose2d getStartPose() {

    return myPath.getStartPose(Rotation2d.fromDegrees(0));
  }

  public static FollowPath.Builder getBuilder() {

    return pathBuilder;
  }

  private static void registerCommands() {
    if (s.launcherSubsystem != null && s.indexerSubsystem != null) {
      if (Robot.isSimulation()) {
        FollowPath.registerEventTrigger("launch", RobotSim.launch(s, 1));

      } else {
        FollowPath.registerEventTrigger("launch", launcherCommand());
      }
    }
    if (s.indexerSubsystem != null) {
      FollowPath.registerEventTrigger("intake", intakeCommand());
    }
    FollowPath.registerEventTrigger("climb", climbCommand());
  }

  public static Command intakeCommand() {
    return Commands.runOnce(() -> Controls.intakeMode = IntakeMode.INTAKE)
        .withName("Auto Intake Command");
  }

  public static Command climbCommand() {
    return Commands.none().withName("Auto Climb Command");
  }

  public static Command launcherCommand() {
    return Commands.parallel(
            Commands.runOnce(
                () -> {
                  s.flywheels.resetFuelCheck();
                }),
            s.launcherSubsystem.launcherAimCommand(),
            Commands.waitUntil(() -> s.launcherSubsystem.isAtTarget())
                .andThen(s.indexerSubsystem.runIndexer(() -> s.flywheels.getTargetSpeed())))
        // .until(() -> s.flywheels.isOutOfFuel())
        .withTimeout(4.5)
        .andThen(s.launcherSubsystem.rawStowCommand())
        .withName("Auto Launcher Command");
  }
}
