package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
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
  private final BooleanPublisher hoodBooleanPub;
  private final BooleanPublisher turretBooleanPub;
  private final BooleanPublisher flywheelBooleanPub;
  private final BooleanPublisher notGoingToBeUnderTrenchPub;
  private final BooleanPublisher notUnderClimbPub;

  private boolean turretAtTarget;
  private boolean hoodAtTarget;
  private boolean flywheelAtTarget;
  private boolean notUunderClimb;
  private boolean notGoingToBeUnderTrench;

  private LaunchingParameters launchParameters;
  private final double MIN_FAR_DIST = 6; // Meters
  private static final double MIN_TURRET_TOLERANCE_DISTANCE = 0.1;

  public LauncherSubsystem(Subsystems s) {
    this.s = s;

    var nt = NetworkTableInstance.getDefault();
    hoodGoalPub = nt.getDoubleTopic("/AutoAim/hoodGoal").publish();
    hoodGoalPub.set(0.0);
    flywheelGoalPub = nt.getDoubleTopic("/AutoAim/flywheelGoal").publish();
    flywheelGoalPub.set(0.0);
    hoodBooleanPub = nt.getBooleanTopic("/AutoAim/hoodAtTarget").publish();
    turretBooleanPub = nt.getBooleanTopic("/AutoAim/turretAtTarget").publish();
    flywheelBooleanPub = nt.getBooleanTopic("/AutoAim/flywheelAtTarget").publish();
    notUnderClimbPub = nt.getBooleanTopic("/AutoAim/NotUnderClimb").publish();
    notGoingToBeUnderTrenchPub = nt.getBooleanTopic("/AutoAim/NotUnderTrench").publish();
    hoodBooleanPub.set(false);
    turretBooleanPub.set(false);
    flywheelBooleanPub.set(false);
    notUnderClimbPub.set(false);
    notGoingToBeUnderTrenchPub.set(false);
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

              hoodGoalPub.set(hoodGoal);
              flywheelGoalPub.set(flywheelsGoal);

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
    double flywheelTolerance = s.flywheels.FLYWHEEL_TOLERANCE;
    double targetDistance = Math.max(launchParameters.currentDist(), MIN_TURRET_TOLERANCE_DISTANCE);

    if (targetDistance >= MIN_FAR_DIST) {
      flywheelTolerance = 30;
    }

    flywheelAtTarget = s.flywheels.atTargetVelocity(flywheelsGoal, flywheelTolerance);
    flywheelBooleanPub.set(flywheelAtTarget);

    hoodAtTarget = s.hood.atTargetPosition();
    hoodBooleanPub.set(hoodAtTarget);

    turretAtTarget =
        s.turretSubsystem.atTarget(
            () ->
                Math.min(
                    Units.degreesToRadians(20),
                    Math.max(Units.degreesToRadians(4), Math.atan(0.3 / targetDistance))));
    turretBooleanPub.set(turretAtTarget);

    notUunderClimb =
        !LaunchCalculator.isUnderClimb(
            driveState.Pose.transformBy(LauncherConstants.turretTransform()));
    notUnderClimbPub.set(notUunderClimb);

    notGoingToBeUnderTrench =
        !LaunchCalculator.isApproachingTrench(driveState.Pose, driveState.Speeds);
    notGoingToBeUnderTrenchPub.set(notGoingToBeUnderTrench);

    return flywheelAtTarget
        && hoodAtTarget
        && turretAtTarget
        && notUunderClimb
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
