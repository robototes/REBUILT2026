package frc.robot.subsystems.launcher;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LaunchCalculator;
import frc.robot.subsystems.LaunchCalculator.LaunchingParameters;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import frc.robot.util.GetTargetFromPose;
import frc.robot.util.tuning.LauncherConstants;

public class LauncherSubsystem extends SubsystemBase {
  protected Hood hood;
  protected Flywheels flywheels;
  protected double flywheelsGoal;
  protected double hoodGoal;

  private final DoublePublisher hoodGoalPub;
  private final DoublePublisher flywheelGoalPub;

  public LauncherSubsystem(Hood hood, Flywheels flywheels) {
    this.hood = hood;
    this.flywheels = flywheels;

    var nt = NetworkTableInstance.getDefault();
    hoodGoalPub = nt.getDoubleTopic("/AutoAim/hoodGoal").publish();
    hoodGoalPub.set(0.0);
    flywheelGoalPub = nt.getDoubleTopic("/AutoAim/flywheelGoal").publish();
    flywheelGoalPub.set(0.0);
  }

  public Command launcherAimCommand(CommandSwerveDrivetrain drive) {
    return Commands.run(
        () -> {
          Translation2d targetPose = GetTargetFromPose.getTargetLocation(drive);

          hoodGoal = LauncherConstants.getHoodAngleFromPose2d(targetPose, drive.getState().Pose);
          flywheelsGoal =
              LauncherConstants.getFlywheelSpeedFromPose2d(targetPose, drive.getState().Pose);

          hoodGoalPub.set(hoodGoal);
          flywheelGoalPub.set(flywheelsGoal);

          hood.setHoodPosition(hoodGoal);
          flywheels.setVelocityRPS(flywheelsGoal);
        });
  }

  // Will use after week 1
  public Command launcherAimCommandV2(CommandSwerveDrivetrain drive) {
    return Commands.run(
        () -> {
          LaunchingParameters para = LaunchCalculator.getInstance().getParameters(drive);
          hoodGoal = para.targetHood();
          flywheelsGoal = para.targetFlywheels();

          hood.setHoodPosition(hoodGoal);
          flywheels.setVelocityRPS(flywheelsGoal);
        });
  }

  // TODO: add tolerance range calculation
  public boolean isAtTarget() {
    return flywheels.atTargetVelocity(flywheelsGoal, flywheels.FLYWHEEL_TOLERANCE)
        && hood.atTargetPosition()
        && !LaunchCalculator.isCloseToTrench(LaunchCalculator.getPose());
  }

  public boolean isHoodAtTarget() {
    return hood.atTargetPosition();
  }

  public Command zeroSubsystemCommand() {
    return hood.zeroHoodCommand();
  }

  public Command stowCommand() {
    return Commands.parallel(hood.hoodPositionCommand(0.0), flywheels.stopCommand());
  }

  public Command rawStowCommand() {
    hoodGoal = 0;
    flywheelsGoal = 0;
    return Commands.parallel(
        Commands.runOnce(() -> hood.setHoodPosition(0)),
        Commands.runOnce(() -> flywheels.stopVoid()));
  }
}
