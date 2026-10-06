package frc.robot.subsystems.auto.BLine;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.Subsystems;
import frc.robot.lib.BLine.BLineCommands;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.subsystems.auto.Misc.StuckOnBallRecovery;
import frc.robot.util.simulation.RobotSim;
import java.util.Set;
import java.util.concurrent.atomic.AtomicBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class BLineTriggers {
  public static Trigger beachedTrigger;
  public static LoggedNetworkBoolean enableSotm = new LoggedNetworkBoolean("BLine/SOTM", false);
private static Boolean enableLaunchOnTheMove = true;
  public static LoggedNetworkBoolean enableUnbeach =
      new LoggedNetworkBoolean("BLine/Auto Unbeach", false);

  public static void registerTriggers(Subsystems s) {
    beachedTrigger =
        new Trigger(
            () -> {
              return enableUnbeach.getAsBoolean()
                  && RobotState.isAutonomous()
                  && s.drivebaseSubsystem.isBeached(StuckOnBallRecovery.STUCK_ANGLE_THRESHOLD);
            });

    beachedTrigger.onTrue(
        Commands.sequence(
            Commands.runOnce(
                () -> {
                  if (BLineLogic.follow != null) {
                    BLineLogic.savedPathIndex =
                        BLineLogic.follow.getCurrentTranslationElementIndex();
                  }
                }),
            AutosCommands.recoverCommand(s),
            AutosCommands.resume(s)));

    AtomicBoolean launchAllowed = new AtomicBoolean(true);

    if (s.launcherSubsystem != null && s.indexerSubsystem != null) {
      if (Robot.isSimulation()) {
        Command bLineSimLaunching = RobotSim.launch(s, 30);

        FollowPath.registerEventTrigger(
            "launch",
            BLineCommands.defer(
                () -> {
                  enableLaunchOnTheMove =
                      SmartDashboard.getBoolean("Enable SOTM?", enableLaunchOnTheMove);

                  return enableLaunchOnTheMove
                      ? Commands.runOnce(() -> launchAllowed.set(true))
                          .andThen(bLineSimLaunching.onlyWhile(launchAllowed::get))
                          .andThen(Commands.print("LAUNCH FINISHED"))
                      : Commands.none();
                },
                Set.of()));

      } else {
        Command bLineLaunching = AutosCommands.launcherCommand(s);

        FollowPath.registerEventTrigger(
            "launch",
            Commands.defer(
                () -> {
                  enableLaunchOnTheMove =
                      SmartDashboard.getBoolean("Enable SOTM?", enableLaunchOnTheMove);

                  return enableLaunchOnTheMove ? bLineLaunching : Commands.none();
                },
                Set.of()));
      }
    }

    FollowPath.registerEventTrigger("intake", AutosCommands.intakeCommand());

    FollowPath.registerEventTrigger("climb", AutosCommands.climbCommand());

    FollowPath.registerEventTrigger(
        "cancel",
        Commands.runOnce(() -> launchAllowed.set(false)).andThen(AutosCommands.stowCommand(s)));
  }
}
