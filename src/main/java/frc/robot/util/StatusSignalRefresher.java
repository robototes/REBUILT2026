package frc.robot.util;

import com.ctre.phoenix6.BaseStatusSignal;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

public final class StatusSignalRefresher {
  private static final StatusSignalRefresher INSTANCE = new StatusSignalRefresher();

  public static StatusSignalRefresher getInstance() {
    return INSTANCE;
  }

  private final Map<Integer, List<BaseStatusSignal>> pending = new TreeMap<>();
  private int[] divisors = new int[0];
  private BaseStatusSignal[][] groups = new BaseStatusSignal[0][];
  private boolean finalized = false;
  private int cycle = 0;

  /** Refreshed every loop. */
  public void addStatusSignals(BaseStatusSignal... signals) {
    addStatusSignals(1, signals);
  }

  /** Refreshed every N loops (temps, faults, etc). */
  public void addStatusSignals(int delayLoopCount, BaseStatusSignal... signals) {
    if (finalized) {
      throw new IllegalStateException("Signals registered after finalize()");
    }
    pending.computeIfAbsent(delayLoopCount, k -> new ArrayList<>()).addAll(Arrays.asList(signals));
  }

  /** Call once, after every subsystem/IO is constructed. */
  public void finalizeStatusSignals() {
    divisors = pending.keySet().stream().mapToInt(Integer::intValue).toArray();
    groups = new BaseStatusSignal[divisors.length][];
    for (int i = 0; i < divisors.length; i++) {
      groups[i] = pending.get(divisors[i]).toArray(new BaseStatusSignal[0]);
    }
    finalized = true;
  }

  /** Call at the top of robotPeriodic(). */
  public void refreshStatusSignals() {
    for (int i = 0; i < groups.length; i++) {
      if (cycle % divisors[i] == 0) {
        BaseStatusSignal.refreshAll(groups[i]);
      }
    }
    cycle++;
  }
}
