package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;
import java.util.function.Supplier;
import lombok.Setter;

public class ClockAndHubStatus {
  // seconds
  private static final double TRANSITION_PERIOD_END_TIME = 130;
  private static final double SHIFT_1_END_TIME = 105;
  private static final double SHIFT_2_END_TIME = 80;
  private static final double SHIFT_3_END_TIME = 55;
  private static final double END_GAME_START_TIME = 30;
  private static final Timer shiftTimer = new Timer();

  boolean redInactiveFirst = false;

  /** Starts the timer at the begining of teleop. */
  public static void initialize() {
    shiftTimer.restart();
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

    // Return FMS value
    String message = DriverStation.getGameSpecificMessage();
    if (message.length() > 0) {
      char character = message.charAt(0);
      if (character == 'R') {
        return redInactiveFirst = true;
      } else if (character == 'B') {
        return redInactiveFirst = false;
      }
    }

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
    if (DriverStation.isTeleopEnabled()) {
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
    return false;
  }

  @Setter private static Supplier<Optional<Boolean>> allianceWinOverride = () -> Optional.empty();

  public static Optional<Boolean> getAllianceWinOverride() {
    return allianceWinOverride.get();
  }

  public static Alliance getFirstActiveAlliance() {
    var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    // Return override value
    var winOverride = getAllianceWinOverride();
    if (!winOverride.isEmpty()) {
      return winOverride.get()
          ? (alliance == Alliance.Blue ? Alliance.Red : Alliance.Blue)
          : (alliance);
    }

    // Return FMS value
    String message = DriverStation.getGameSpecificMessage();
    if (message.length() > 0) {
      char character = message.charAt(0);
      if (character == 'R') {
        return Alliance.Blue;
      } else if (character == 'B') {
        return Alliance.Red;
      }
    }

    // Return default value
    return alliance == Alliance.Blue ? Alliance.Red : Alliance.Blue;
  }
}
