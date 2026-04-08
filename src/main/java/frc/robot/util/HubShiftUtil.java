package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.tuning.LauncherConstants;
import java.util.Optional;
import java.util.function.Supplier;
import lombok.Setter;

public class HubShiftUtil {
  public enum ShiftEnum {
    TRANSITION,
    SHIFT1,
    SHIFT2,
    SHIFT3,
    SHIFT4,
    ENDGAME,
    AUTO,
    DISABLED;
  }

  public record ShiftInfo(
      ShiftEnum currentShift, double elapsedTime, double remainingTime, boolean active) {}

  private static Timer shiftTimer = new Timer();
  private static final ShiftEnum[] shiftsEnums = ShiftEnum.values();

  private static final double[] shiftStartTimes = {0.0, 10.0, 35.0, 60.0, 85.0, 110.0};
  private static final double[] shiftEndTimes = {10.0, 35.0, 60.0, 85.0, 110.0, 140.0};

  private static final double minFuelCountDelay = 1.0;
  private static final double maxFuelCountDelay = 2.0;
  private static final double shiftEndFuelCountExtension = 3.0;
  private static final double minTimeOfFlight = LauncherConstants.minTimeOfFlight();
  private static final double maxTimeOfFlight = LauncherConstants.maxTimeOfFlight();
  private static final double approachingActiveFudge = -1 * (minTimeOfFlight + minFuelCountDelay);
  private static final double endingActiveFudge =
      shiftEndFuelCountExtension + -1 * (maxTimeOfFlight + maxFuelCountDelay);

  public static final double autoEndTime = 20.0;
  public static final double teleopDuration = 140.0;
  private static final boolean[] activeSchedule = {true, true, false, true, false, true};
  private static final boolean[] inactiveSchedule = {true, false, true, false, true, true};

  // Pre-cached shifted time arrays for both alliance schedule cases
  private static final double[] shiftedStartTimesActive;
  private static final double[] shiftedEndTimesActive;
  private static final double[] shiftedStartTimesInactive;
  private static final double[] shiftedEndTimesInactive;

  static {
    shiftedStartTimesActive = computeShiftedStartTimes(activeSchedule);
    shiftedEndTimesActive = computeShiftedEndTimes(activeSchedule);
    shiftedStartTimesInactive = computeShiftedStartTimes(inactiveSchedule);
    shiftedEndTimesInactive = computeShiftedEndTimes(inactiveSchedule);
  }

  private static double[] computeShiftedStartTimes(boolean[] schedule) {
    double[] starts = shiftStartTimes.clone();
    for (int i = 1; i < schedule.length; i++) {
      if (schedule[i] && !schedule[i - 1]) {
        starts[i] += approachingActiveFudge;
      } else if (!schedule[i] && schedule[i - 1]) {
        starts[i] += endingActiveFudge;
      }
    }
    return starts;
  }

  private static double[] computeShiftedEndTimes(boolean[] schedule) {
    double[] ends = shiftEndTimes.clone();
    for (int i = 1; i < schedule.length; i++) {
      if (schedule[i] && !schedule[i - 1]) {
        ends[i - 1] += approachingActiveFudge;
      } else if (!schedule[i] && schedule[i - 1]) {
        ends[i - 1] += endingActiveFudge;
      }
    }
    return ends;
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
          ? (alliance == Alliance.Red ? Alliance.Blue : Alliance.Red)
          : alliance;
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

  /** Starts the timer at the beginning of teleop. */
  public static void initialize() {
    shiftTimer.restart();
  }

  private static boolean[] getSchedule() {
    Alliance startAlliance = getFirstActiveAlliance();
    return startAlliance == DriverStation.getAlliance().orElse(Alliance.Blue)
        ? activeSchedule
        : inactiveSchedule;
  }

  private static ShiftInfo getShiftInfo(
      boolean[] currentSchedule, double[] shiftStartTimes, double[] shiftEndTimes) {
    double currentTime = shiftTimer.get();
    double stateTimeElapsed = shiftTimer.get();
    double stateTimeRemaining = 0.0;
    boolean active = false;
    ShiftEnum currentShift = ShiftEnum.DISABLED;

    if (DriverStation.isAutonomousEnabled()) {
      stateTimeElapsed = currentTime;
      stateTimeRemaining = autoEndTime - currentTime;
      active = true;
      currentShift = ShiftEnum.AUTO;
    } else if (DriverStation.isEnabled()) {
      int currentShiftIndex = -1;
      for (int i = 0; i < shiftStartTimes.length; i++) {
        if (currentTime >= shiftStartTimes[i] && currentTime < shiftEndTimes[i]) {
          currentShiftIndex = i;
          break;
        }
      }
      if (currentShiftIndex < 0) {
        // After last shift, so assume endgame
        currentShiftIndex = shiftStartTimes.length - 1;
      }

      // Calculate elapsed and remaining time in the current shift, ignoring combined shifts
      stateTimeElapsed = currentTime - shiftStartTimes[currentShiftIndex];
      stateTimeRemaining = shiftEndTimes[currentShiftIndex] - currentTime;

      // If the state is the same as the last shift, combine the elapsed time
      if (currentShiftIndex > 0) {
        if (currentSchedule[currentShiftIndex] == currentSchedule[currentShiftIndex - 1]) {
          stateTimeElapsed = currentTime - shiftStartTimes[currentShiftIndex - 1];
        }
      }

      // If the state is the same as the next shift, combine the remaining time
      if (currentShiftIndex < shiftEndTimes.length - 1) {
        if (currentSchedule[currentShiftIndex] == currentSchedule[currentShiftIndex + 1]) {
          stateTimeRemaining = shiftEndTimes[currentShiftIndex + 1] - currentTime;
        }
      }

      active = currentSchedule[currentShiftIndex];
      currentShift = shiftsEnums[currentShiftIndex];
    }
    return new ShiftInfo(currentShift, stateTimeElapsed, stateTimeRemaining, active);
  }

  public static ShiftInfo getOfficialShiftInfo() {
    return getShiftInfo(getSchedule(), shiftStartTimes, shiftEndTimes);
  }

  public static ShiftInfo getShiftedShiftInfo() {
    boolean[] schedule = getSchedule();
    if (schedule == activeSchedule) {
      return getShiftInfo(activeSchedule, shiftedStartTimesActive, shiftedEndTimesActive);
    } else {
      return getShiftInfo(inactiveSchedule, shiftedStartTimesInactive, shiftedEndTimesInactive);
    }
  }
}
