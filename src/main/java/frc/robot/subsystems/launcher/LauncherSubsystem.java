package frc.robot.subsystems.launcher;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import frc.robot.util.AllianceUtils;
import frc.robot.util.LauncherConstants;

public class LauncherSubsystem {
  protected Hood hood;
  protected Flywheels flywheels;
  protected TurretSubsystem turret;
  protected double flywheelsGoal;
  protected double hoodGoal;

  private final DoubleTopic hoodGoalTopic;
  private final DoubleTopic flywheelGoalTopic;
  private final DoublePublisher flywheelGoalPub;
  private final DoublePublisher hoodGoalPub;

  public LauncherSubsystem(Hood hood, Flywheels flywheels, TurretSubsystem turret) {
    this.hood = hood;
    this.flywheels = flywheels;
    this.turret = turret;

    var nt = NetworkTableInstance.getDefault();
    hoodGoalTopic = nt.getDoubleTopic("/AutoAim/hoodGoal");
    flywheelGoalTopic = nt.getDoubleTopic("/AutoAim/flywheelGoal");
    hoodGoalPub = hoodGoalTopic.publish();
    hoodGoalPub.set(0.0);
    flywheelGoalPub = flywheelGoalTopic.publish();
    flywheelGoalPub.set(0.0);
  }

  public Command launcherAimCommand(CommandSwerveDrivetrain drive) {
    return Commands.runEnd(
        () -> {
          Translation2d targetPose = (AllianceUtils.getHubTranslation2d());

          hoodGoal = LauncherConstants.getHoodAngleFromPose2d(targetPose, drive.getState().Pose);
          flywheelsGoal =
              LauncherConstants.getFlywheelSpeedFromPose2d(targetPose, drive.getState().Pose);

          hoodGoalPub.set(hoodGoal);
          flywheelGoalPub.set(flywheelsGoal);

          hood.setHoodPosition(hoodGoal);
          flywheels.setVelocityRPS(flywheelsGoal);
          CommandScheduler.getInstance().schedule(turret.rotateToHub());
        },
        () -> CommandScheduler.getInstance().schedule(stowCommand()));
  }

  // TODO: add tolerance range calculation
  public boolean isAtTarget() {
    return flywheels.atTargetVelocity(flywheelsGoal, flywheels.FLYWHEEL_TOLERANCE)
        && hood.atTargetPosition()
        && turret.atTarget(2);
  }

  public Command zeroSubsystemCommand() {
    return Commands.parallel(hood.zeroHoodCommand(), turret.zeroTurret());
  }

  public Command stowCommand() {
    return Commands.parallel(hood.hoodPositionCommand(0.0), flywheels.setVelocityCommand(0.0));
  }
}
