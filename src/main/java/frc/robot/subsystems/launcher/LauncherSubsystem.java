package frc.robot.subsystems.launcher;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems;
import frc.robot.subsystems.LaunchCalculator;
import frc.robot.subsystems.LaunchCalculator.LaunchingParameters;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import frc.robot.util.GetTargetFromPose;
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

  public Command launcherAimCommand(CommandSwerveDrivetrain drive) {
    return Commands.run(
            () -> {
              Translation2d targetPose = GetTargetFromPose.getTargetLocation(drive);

              hoodGoal =
                  LauncherConstants.getHoodAngleFromPose2d(targetPose, drive.getState().Pose);
              flywheelsGoal =
                  LauncherConstants.getFlywheelSpeedFromPose2d(targetPose, drive.getState().Pose);

              hoodGoalPub.set(hoodGoal);
              flywheelGoalPub.set(flywheelsGoal);

              s.hood.setHoodPosition(hoodGoal);
              s.flywheels.setVelocityRPS(flywheelsGoal);
            })
        .withName("Launcher Aim Command");
  }

  // Will use after week 1
  public Command launcherAimCommandV2() {
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
        .withName("Launcher Aim Command V2");
  }

  // TODO: add tolerance range calculation
  public boolean isAtTarget() {
    if (launchParameters == null) {
      return false;
    }
    return s.flywheels.atTargetVelocity(flywheelsGoal, s.flywheels.FLYWHEEL_TOLERANCE)
        && s.hood.atTargetPosition()
        && !LaunchCalculator.isCloseToTrench(launchParameters.turretPose())
        && s.turretSubsystem.atTarget(TurretSubsystem.TURRET_DEGREE_TOLERANCE);
  }

  public boolean isAtTargetFallback() {
    if (launchParameters == null) {
      return false;
    }
    return s.flywheels.atTargetVelocity(flywheelsGoal, s.flywheels.FLYWHEEL_TOLERANCE)
        && s.hood.atTargetPosition()
        && !LaunchCalculator.isCloseToTrench(launchParameters.turretPose())
        && isDriveBaseAtTarget();
  }

  public boolean isHoodAtTarget() {
    return s.hood.atTargetPosition();
  }

  public Command zeroSubsystemCommand() {
    return s.hood.zeroHoodCommand();
  }

  public boolean isDriveBaseAtTarget() {
    return Math.abs(launchParameters.targetTurret().getDegrees())
        <= TurretSubsystem.TURRET_DEGREE_TOLERANCE;
  }

  public Command stowCommand() {
    return Commands.parallel(s.hood.hoodPositionCommand(0.0), s.flywheels.stopCommand())
        .withName("Stow Launcher Command");
  }

  public Command rawStowCommand() {
    hoodGoal = 0;
    flywheelsGoal = 0;
    return Commands.parallel(
            Commands.runOnce(() -> s.hood.setHoodPosition(0)),
            Commands.runOnce(() -> s.flywheels.stopVoid()))
        .withName("Raw Stow Command");
  }
}
