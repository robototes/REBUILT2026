package frc.robot.subsystems.auto.BLine;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Subsystems;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.util.simulation.RobotSim;

public class BLineTriggers {
  public static Command bLineLaunching;

  public static void registerTriggers(Subsystems s) {

    if (s.launcherSubsystem != null && s.indexerSubsystem != null) {
      if (Robot.isSimulation()) {
        bLineLaunching = RobotSim.launch(s, 30);
      } else {
        bLineLaunching = AutosCommands.launcherCommand(s);
      }

      FollowPath.registerEventTrigger("launch", bLineLaunching);
    }

    FollowPath.registerEventTrigger("intake", AutosCommands.intakeCommand());
  }
}
