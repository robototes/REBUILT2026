package frc.robot.subsystems.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher.Flywheels;
import frc.robot.subsystems.Launcher.Hood;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import frc.robot.util.AllianceUtils;
import frc.robot.util.LauncherConstants;

public class AutoAim {
  public static Command autoAim(CommandSwerveDrivetrain drive, Hood hood, Flywheels flywheels) {
    return new AutoAimCommand(drive, hood, flywheels).withName("Auto Align");
  }

  private static class AutoAimCommand extends Command {
    protected Translation2d targetPose;
    protected CommandSwerveDrivetrain drive;
    protected Hood hood;
    protected Flywheels flywheels;
    protected double flywheelsGoal;
    protected double hoodGoal;

    private final DoubleTopic hoodGoalTopic;
    private final DoubleTopic flywheelGoalTopic;
    private final DoublePublisher flywheelGoalPub;
    private final DoublePublisher hoodGoalPub;

    AutoAimCommand(CommandSwerveDrivetrain drive, Hood hood, Flywheels flywheels) {
      this.drive = drive;
      this.hood = hood;
      this.flywheels = flywheels;

      addRequirements(hood, flywheels);
      var nt = NetworkTableInstance.getDefault();
      hoodGoalTopic = nt.getDoubleTopic("/AutoAim/hoodGoal");
      flywheelGoalTopic = nt.getDoubleTopic("/AutoAim/flywheelGoal");
      hoodGoalPub = hoodGoalTopic.publish();
      hoodGoalPub.set(0.0);
      flywheelGoalPub = flywheelGoalTopic.publish();
      flywheelGoalPub.set(0.0);
    }

    @Override
    public void initialize() {

      targetPose = (AllianceUtils.getHubTranslation2d());
    }

    @Override
    public void execute() {
      flywheelsGoal =
          LauncherConstants.getFlywheelSpeedFromPose2d(targetPose, drive.getState().Pose);
      hoodGoal = LauncherConstants.getHoodAngleFromPose2d(targetPose, drive.getState().Pose);

      flywheelGoalPub.set(flywheelsGoal);
      hoodGoalPub.set(hoodGoal);
      hood.setHoodPosition(hoodGoal);
      flywheels.setVelocityRPS(flywheelsGoal);
    }

    @Override
    public boolean isFinished() {
      return flywheels.atTargetVelocity(flywheelsGoal, flywheels.FLYWHEEL_TOLERANCE)
          && hood.atTargetPosition();
    }
  }
}
