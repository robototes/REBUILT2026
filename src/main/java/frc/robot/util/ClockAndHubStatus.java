package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import java.util.Optional;

public class ClockAndHubStatus {
  public double matchLength = 2.5;

  private Translation2d pointLeftFieldTop = new Translation2d(2, 6);
  private Translation2d pointLeftFieldBottom = new Translation2d(2, 2);
  private Translation2d pointRightFieldTop = new Translation2d(14, 6);
  private Translation2d pointRightFieldBottom = new Translation2d(14, 2);

  private double fieldLength = Units.inchesToMeters(651.2);
  private double fieldWidth = Units.inchesToMeters(317.7);
  private double allianceLineX = Units.inchesToMeters(158.6);
  private double robotOffset = Units.inchesToMeters(15);

  private static final double TRANSITION_PERIOD_END_TIME = 130;
  private static final double SHIFT_1_END_TIME = 105;
  private static final double SHIFT_2_END_TIME = 80;
  private static final double SHIFT_3_END_TIME = 55;
  private static final double END_GAME_START_TIME = 30;

  public Translation2d getTargetLocation(CommandSwerveDrivetrain drivetrain) {
    // if (isHubActive(0)) {
    //   return AllianceUtils.getHubTranslation2d();
    // }

    if (AllianceUtils.isBlue()) {
      if (drivetrain.getState().Pose.getX() <= allianceLineX + robotOffset) {
        return AllianceUtils.getHubTranslation2d();
      } else if (drivetrain.getState().Pose.getY() >= (fieldWidth / 2)) {
        return pointLeftFieldTop;
      } else {
        return pointLeftFieldBottom;
      }
    } else if (AllianceUtils.isRed()) {
      if (drivetrain.getState().Pose.getX() >= (fieldLength - allianceLineX - robotOffset)) {
        return AllianceUtils.getHubTranslation2d();
      } else if (drivetrain.getState().Pose.getY() >= (fieldWidth / 2)) {
        return pointRightFieldTop;
      } else {
        return pointRightFieldBottom;
      }
    } else {
      return AllianceUtils.getHubTranslation2d();
    }
  }

  public boolean isHubActive(double lookAheadTime) {
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

    // Shift is active
    boolean shift1Active =
        switch (alliance.get()) {
          case Red -> !redInactiveFirst;
          case Blue -> redInactiveFirst;
        };

    if (matchTime > TRANSITION_PERIOD_END_TIME) {
      // Transition shift, hub is active.
      return true;
    } else if (matchTime > SHIFT_1_END_TIME) {
      // Shift 1
      return shift1Active;
    } else if (matchTime > SHIFT_2_END_TIME) {
      // Shift 2
      return !shift1Active;
    } else if (matchTime > SHIFT_3_END_TIME) {
      // Shift 3
      return shift1Active;
    } else if (matchTime > END_GAME_START_TIME) {
      // Shift 4
      return !shift1Active;
    } else {
      // End game, hub always active.
      return true;
    }
  }

  public boolean isGameDataValid() {
    String gameData = DriverStation.getGameSpecificMessage();
    if (gameData.isEmpty()) {
      return false;
    }

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
