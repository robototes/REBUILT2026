package frc.robot.subsystems.launcher;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LaunchCalculator;
import frc.robot.subsystems.LaunchCalculator.LaunchingParameters;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import frc.robot.util.AllianceUtils;
import frc.robot.util.LauncherConstants;

public class LauncherSubsystem extends SubsystemBase {
  protected Hood hood;
  protected Flywheels flywheels;
  protected TurretSubsystem turret;
  protected double flywheelsGoal;
  protected double hoodGoal;
  protected double turretGoal;

  private final DoublePublisher hoodGoalPub;
  private final DoublePublisher flywheelGoalPub;
  private final DoublePublisher turretGoalPub;

  public LauncherSubsystem(Hood hood, Flywheels flywheels, TurretSubsystem turret) {
    this.hood = hood;
    this.flywheels = flywheels;
    this.turret = turret;

    var nt = NetworkTableInstance.getDefault();
    hoodGoalPub = nt.getDoubleTopic("/AutoAim/hoodGoal").publish();
    hoodGoalPub.set(0.0);
    flywheelGoalPub = nt.getDoubleTopic("/AutoAim/flywheelGoal").publish();
    flywheelGoalPub.set(0.0);
    turretGoalPub = nt.getDoubleTopic("/AutoAim/turretGoal").publish();
    turretGoalPub.set(0.0);
  }

  public Command launcherAimCommand(CommandSwerveDrivetrain drive) {
    return Commands.runEnd(
        () -> {
          Translation2d targetPose = (AllianceUtils.getHubTranslation2d());

          hoodGoal = LauncherConstants.getHoodAngleFromPose2d(targetPose, drive.getState().Pose);
          flywheelsGoal =
              LauncherConstants.getFlywheelSpeedFromPose2d(targetPose, drive.getState().Pose);
          turretGoal = turret.calculateTurretAngle();

          hoodGoalPub.set(hoodGoal);
          flywheelGoalPub.set(flywheelsGoal);
          turretGoalPub.set(turretGoal);

          hood.setHoodPosition(hoodGoal);
          flywheels.setVelocityRPS(flywheelsGoal);
          turret.setTurretRawPosition(turretGoal);
        },
        () -> CommandScheduler.getInstance().schedule(stowCommand()));
  }

  public Command launcherAimV2(CommandSwerveDrivetrain drive) {
    return Commands.runEnd(() -> {
    LaunchingParameters para = LaunchCalculator.getInstance().getParameters(drive);
    hood.setHoodPosition(para.hoodAngle());
    flywheels.setVelocityRPS(para.flywheelSpeed());

    double turretDegrees = MathUtil.clamp(turret.getTurretPosition() + ((para.turretAngle().getDegrees() - turret.getTurretPosition() + 540) % 360) - 180, TurretSubsystem.TURRET_MIN, TurretSubsystem.TURRET_MAX);
    turret.setTurretRawPosition(Units.degreesToRotations(turretDegrees));
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
    return Commands.parallel(hood.hoodPositionCommand(0.0), flywheels.stopCommand());
  }
}
