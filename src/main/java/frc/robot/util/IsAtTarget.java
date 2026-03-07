package frc.robot.util;

import frc.robot.Subsystems;
import frc.robot.subsystems.LaunchCalculator;
import frc.robot.subsystems.launcher.Flywheels;
import frc.robot.subsystems.launcher.Hood;
import frc.robot.subsystems.launcher.TurretSubsystem;
import frc.robot.util.tuning.LauncherConstants;

public class IsAtTarget {
  private TurretSubsystem turret;
  private Hood hood;
  private Flywheels flyWheels;

  private double TURRET_TOLERANCE = 0.5;
  private double RPS_TOLERANCE = 5;

  public void init(Subsystems s) {
    turret = s.turretSubsystem;
    hood = s.hood;
    flyWheels = s.flywheels;
  }

  public boolean isReadyToShoot() {
    return turretAtTarget() && hoodAtTarget() && flywheelsAtTarget();
  }

  public boolean isHoodReadyForTrench() {
    return hood.atTargetPosition(0);
  }

  public boolean turretAtTarget() {
    return turret.atTarget(TURRET_TOLERANCE);
  }

  public boolean hoodAtTarget() {
    return hood.atTargetPosition();
  }

  public boolean flywheelsAtTarget() {
    return flyWheels.atTargetVelocity(
        LauncherConstants.getFlywheelSpeedFromDistance(LaunchCalculator.estimatedDist),
        RPS_TOLERANCE);
  }
}
