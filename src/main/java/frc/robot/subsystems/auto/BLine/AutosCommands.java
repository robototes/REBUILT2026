package frc.robot.subsystems.auto.BLine;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Controls;
import frc.robot.Robot;
import frc.robot.Subsystems;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.auto.Misc.StuckOnBallRecovery;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeMode;
import frc.robot.util.simulation.RobotSim;
import java.util.ArrayList;
import java.util.List;

public class AutosCommands {
  public static Command bLineLaunching;
  public static Command bLineSimLaunching;

  public static Command intakeCommand() {

    return Commands.runOnce(() -> Controls.intakeMode = IntakeMode.INTAKE)
        .withName("Auto Intake Command");
  }

  public static Command launcherCommand(double timeout, Subsystems s) {

    if (s == null || Robot.isSimulation()) {

      return RobotSim.launch(s, timeout);
    }

    return Commands.parallel(
            Commands.runOnce(() -> s.flywheels.resetFuelCheck()),
            s.launcherSubsystem.launcherAimCommand(),
            Commands.waitUntil(() -> s.launcherSubsystem.isAtTarget())
                .andThen(s.indexerSubsystem.runIndexer(() -> s.flywheels.getTargetSpeed())))
        .withTimeout(timeout)
        .andThen(s.launcherSubsystem.rawStowCommand())
        .withName("Auto Launcher Command");
  }

  public static Command launcherCommand(Subsystems s) {

    if (s.launcherSubsystem != null && s.flywheels != null) {

      return Commands.parallel(
              Commands.runOnce(() -> s.flywheels.resetFuelCheck()),
              s.launcherSubsystem.launcherAimCommand(),
              Commands.waitUntil(() -> s.launcherSubsystem.isAtTarget())
                  .andThen(s.indexerSubsystem.runIndexer(() -> s.flywheels.getTargetSpeed())))
          .withName("Auto Launcher Command");
    }

    return Commands.none();
  }

  public static Command resume(Subsystems s) {

    if (BLineLogic.follow == null) {
      return Commands.none();
    }

    int i = BLineLogic.savedPathIndex;

    var flat =
        BLineLogic.getSelectedAutoPath().getPath().getPathElementsWithConstraintsNoWaypoints();
    List<Path.PathElement> remaining = new ArrayList<>();
    remaining.add(
        new Path.TranslationTarget(s.drivebaseSubsystem.getState().Pose.getTranslation()));

    for (int j = i; j < flat.size(); j++) {
      remaining.add(flat.get(j).getFirst().copy());
    }

    Path remainder =
        new Path(remaining, BLineLogic.getSelectedAutoPath().getPath().getPathConstraints());
    return BLineLogic.buildPath(remainder, false, true);
  }

  public static Command recoverCommand(Subsystems s) {

    return Commands.sequence(
            BLineLogic.buildPath(
                StuckOnBallRecovery.getRecoverySegment(
                    () -> s.drivebaseSubsystem.getState().Pose,
                    () ->
                        Rotation2d.fromDegrees(
                            s.drivebaseSubsystem.getPigeon2().getPitch().getValueAsDouble()),
                    () ->
                        Rotation2d.fromDegrees(
                            s.drivebaseSubsystem.getPigeon2().getRoll().getValueAsDouble())),
                false,
                false))
        .until(() -> !s.drivebaseSubsystem.isBeached(StuckOnBallRecovery.STUCK_ANGLE_THRESHOLD));
  }

  public static Command stowCommand(Subsystems s) {

    return s.launcherSubsystem != null ? s.launcherSubsystem.rawStowCommand() : Commands.none();
  }

  public static Command climbCommand() {

    return Commands.none().withName("Auto Climb Command");
  }

  public static void cancelCommand(Subsystems s) {

    if (s == null) {
      return;
    }

    if (Robot.isSimulation()) {

      CommandScheduler.getInstance().cancel(bLineSimLaunching);

    } else {

      CommandScheduler.getInstance().cancel(bLineLaunching);
    }
  }
}
