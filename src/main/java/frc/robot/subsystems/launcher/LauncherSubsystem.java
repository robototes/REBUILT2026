package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems;
import frc.robot.subsystems.launcher.LaunchCalculator.LaunchingParameters;
import frc.robot.util.GetTargetFromPose;
import frc.robot.util.tuning.LauncherConstants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class LauncherSubsystem extends SubsystemBase {
  protected double flywheelsGoal;
  protected double hoodGoal;
  protected Subsystems s;

  private boolean turretAtTarget;
  private boolean hoodAtTarget;
  private boolean flywheelAtTarget;
  private boolean notUnderClimb;
  private boolean notGoingToBeUnderTrench;

  private LaunchingParameters launchParameters;
  private final double MIN_FAR_DIST = 6.0; // Meters

  public double flywheelOffset = 0.0; // Starting offset added in RPS
  private static double fwOffsetJogAmount = 1.0; // Jog amount in RPS

  public LauncherSubsystem(Subsystems s) {
    this.s = s;
    Logger.recordOutput("AutoAim/hoodGoal", 0.0);
    Logger.recordOutput("AutoAim/flywheelGoal", 0.0);
    Logger.recordOutput("AutoAim/flywheelAtTarget", false);
    Logger.recordOutput("AutoAim/hoodAtTarget", false);
    Logger.recordOutput("AutoAim/turretAtTarget", false);
    Logger.recordOutput("AutoAim/NotUnderClimb", false);
    Logger.recordOutput("AutoAim/NotUnderTrench", false);
    SmartDashboard.putNumber("Flywheel Offset [RPS]", flywheelOffset);
  }

  public Command launcherAimCommand() {
    return Commands.run(
            () -> {
              LaunchingParameters para =
                  LaunchCalculator.getInstance()
                      .getParameters(s.drivebaseSubsystem, s.turretSubsystem);
              this.launchParameters = para;
              hoodGoal = para.targetHood();
              flywheelsGoal = para.targetFlywheels() + flywheelOffset;

              Logger.recordOutput("AutoAim/hoodGoal", hoodGoal);
              Logger.recordOutput("AutoAim/flywheelGoal", flywheelsGoal);

              s.hood.setHoodPosition(hoodGoal);
              s.flywheels.setVelocityRPS(flywheelsGoal);
            })
        .withName("Launcher Aim Command");
  }

  public Command offsetFlywheelCommand(int direction) {
    return runOnce(
            () -> {
              flywheelOffset += fwOffsetJogAmount * direction;
              SmartDashboard.putNumber("Flywheel Offset [RPS]", flywheelOffset);
            })
        .withName("Changed Flywheel Offset");
  }

  // TODO: add tolerance range calculation
  public boolean isAtTarget() {
    if (launchParameters == null) {
      return false;
    }
    SwerveDriveState driveState = s.drivebaseSubsystem.getState();
    Pose2d turretPose = driveState.Pose.transformBy(LauncherConstants.turretTransform());
    double flywheelTolerance = s.flywheels.FLYWHEEL_TOLERANCE;
    boolean passing = GetTargetFromPose.isPassing(turretPose);

    if (passing) {
      flywheelTolerance = 30;
    }

    flywheelAtTarget = s.flywheels.atTargetVelocity(flywheelsGoal, flywheelTolerance);
    Logger.recordOutput("AutoAim/flywheelAtTarget", flywheelAtTarget);

    hoodAtTarget = s.hood.atTargetPosition();
    Logger.recordOutput("AutoAim/hoodAtTarget", hoodAtTarget);

    DoubleSupplier turretTolerance =
        () ->
            MathUtil.clamp(
                Math.atan(
                    0.3
                        / LauncherConstants
                            .distToHub()), // 0.3 meters of error radially from center of hub
                Units.degreesToRadians(3), // lower bound for error, 5.7m equivalent
                Units.degreesToRadians(7)); // upper bound for error, 2.5m equivalent

    // If we're passing, just get the ball out of the robot ASAP
    if (passing) {
      turretTolerance = () -> Units.degreesToRadians(20);
    }

    turretAtTarget = s.turretSubsystem.atTarget(turretTolerance);
    Logger.recordOutput("AutoAim/turretAtTarget", turretAtTarget);

    notUnderClimb =
        !LaunchCalculator.isUnderClimb(
            driveState.Pose.transformBy(LauncherConstants.turretTransform()));
    Logger.recordOutput("AutoAim/NotUnderClimb", notUnderClimb);

    notGoingToBeUnderTrench =
        !LaunchCalculator.isApproachingTrench(driveState.Pose, driveState.Speeds);
    Logger.recordOutput("AutoAim/NotUnderTrench", notGoingToBeUnderTrench);

    return flywheelAtTarget
        && hoodAtTarget
        && turretAtTarget
        && notUnderClimb
        && notGoingToBeUnderTrench;
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
            Commands.runOnce(() -> s.flywheels.stop()))
        .withName("Raw Stow Command");
  }
}
