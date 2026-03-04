package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import java.util.Optional;

public class ClockAndHubStatus {
  public double matchLength = 2.5;

  private Translation2d pointLeftFieldTop = new Translation2d(2, 2);
  private Translation2d pointLeftFieldBottom = new Translation2d(2, 6);
  private Translation2d pointRightFieldTop = new Translation2d(14, 2);
  private Translation2d pointRightFieldBottom = new Translation2d(14, 6);

  public Translation2d getTargetLocation(CommandSwerveDrivetrain drivetrain) {
    if (isHubActive(0)) {
      return AllianceUtils.getHubTranslation2d();
    } else {
      if (AllianceUtils.isRed()) {
        if (drivetrain.getState().Pose.getX() <= Units.inchesToMeters(158.6)) {
          return AllianceUtils.getHubTranslation2d();
        } else if (drivetrain.getState().Pose.getY() >= Units.inchesToMeters(317.7 / 2)) {
          return pointLeftFieldTop;
        } else {
          return pointLeftFieldBottom;
        }
      }
    }
    if (AllianceUtils.isBlue()) {

      if (drivetrain.getState().Pose.getX() >= Units.inchesToMeters(651.2 - 158.6)) {
        return AllianceUtils.getHubTranslation2d();
      } else if (drivetrain.getState().Pose.getY() >= Units.inchesToMeters(317.7 / 2)) {
        return pointRightFieldBottom;
      } else {
        return pointRightFieldTop;
      }
    } else {
      return AllianceUtils.getHubTranslation2d();
    }
  }

  public static boolean isHubActive(double lookAheadTime) {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // If we have no alliance, we cannot be enabled, therefore no hub.
    if (alliance.isEmpty()) {
      return false;
    }
    // Hub is always enabled in autonomous.
    if (DriverStation.isAutonomousEnabled()) {
      return true;
    }
    // At this point, if we're not teleop enabled, there is no hub.
    if (!DriverStation.isTeleopEnabled()) {
      return false;
    }

    // We're teleop enabled, compute.
    double matchTime = DriverStation.getMatchTime() + lookAheadTime;
    String gameData = DriverStation.getGameSpecificMessage();
    if (gameData.isEmpty()) {
      return false;
    }
    boolean redInactiveFirst = false;
    switch (gameData.charAt(0)) {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default -> {
        // If we have invalid game data, assume hub is active.
        return true;
      }
    }

    // Shift was is active for blue if red won auto, or red if blue won auto.
    boolean shift1Active =
        switch (alliance.get()) {
          case Red -> !redInactiveFirst;
          case Blue -> redInactiveFirst;
        };

    if (matchTime > 130) {
      // Transition shift, hub is active.
      return true;
    } else if (matchTime > 105) {
      // Shift 1
      return shift1Active;
    } else if (matchTime > 80) {
      // Shift 2
      return !shift1Active;
    } else if (matchTime > 55) {
      // Shift 3
      return shift1Active;
    } else if (matchTime > 30) {
      // Shift 4
      return !shift1Active;
    } else {
      // End game, hub always active.
      return true;
    }
  }

  public boolean isGameDataValid() {
    String gameData = DriverStation.getGameSpecificMessage();

    switch (gameData.charAt(0)) {
      case 'R':
        return true;
      case 'B':
        return true;
      default:
        {
          return false;
        }
    }
  }
}
