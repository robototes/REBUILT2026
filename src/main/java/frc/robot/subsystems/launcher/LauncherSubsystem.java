package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems;
import frc.robot.subsystems.launcher.LaunchCalculator.LaunchingParameters;
import frc.robot.util.tuning.LauncherConstants;

public class LauncherSubsystem extends SubsystemBase {
  protected double flywheelsGoal;
  protected double hoodGoal;
  protected Subsystems s;

  private final DoublePublisher hoodGoalPub;
  private final DoublePublisher flywheelGoalPub;
  private LaunchingParameters launchParameters;

  public LauncherSubsystem(Subsystems s) {
    this.s = s;

    var nt = NetworkTableInstance.getDefault();
    hoodGoalPub = nt.getDoubleTopic("/AutoAim/hoodGoal").publish();
    hoodGoalPub.set(0.0);
    flywheelGoalPub = nt.getDoubleTopic("/AutoAim/flywheelGoal").publish();
    flywheelGoalPub.set(0.0);
  }

  public Command launcherAimCommand() {
    return Commands.run(
            () -> {
              LaunchingParameters para =
                  LaunchCalculator.getInstance()
                      .getParameters(s.drivebaseSubsystem, s.turretSubsystem);
              this.launchParameters = para;
              hoodGoal = para.targetHood();
              flywheelsGoal = para.targetFlywheels();

              s.hood.setHoodPosition(hoodGoal);
              s.flywheels.setVelocityRPS(flywheelsGoal);
            })
        .withName("Launcher Aim Command");
  }

  // TODO: add tolerance range calculation
  public boolean isAtTarget() {
    if (launchParameters == null) {
      return false;
    }
    SwerveDriveState driveState = s.drivebaseSubsystem.getState();
    return s.flywheels.atTargetVelocity(flywheelsGoal, s.flywheels.FLYWHEEL_TOLERANCE)
        && s.hood.atTargetPosition()
        && s.turretSubsystem.atTarget()
        && !LaunchCalculator.isApproachingTrench(driveState.Pose, driveState.Speeds)
        && !LaunchCalculator.isUnderClimb(
            driveState.Pose.transformBy(LauncherConstants.turretTransform()));
  }

  public boolean isHoodAtTarget() {
    return s.hood.atTargetPosition();
  }

  public Command zeroSubsystemCommand() {
    return s.hood.zeroHoodCommand();
  }

  public Command rawStowCommand() {
    hoodGoal = 0;
    flywheelsGoal = 0;
    return Commands.parallel(
            Commands.runOnce(() -> s.hood.setHoodPosition(0)),
            Commands.runOnce(() -> s.flywheels.setVelocityRPS(Flywheels.IDLE_FLYWHEEL_RPS)))
        .withName("Raw Stow Command");
  }
}
