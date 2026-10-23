package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Tracer;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.lang.reflect.Field;
import java.lang.reflect.InaccessibleObjectException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

/** Exports WPILib's existing watchdog epochs without instrumenting or wrapping commands. */
public final class LoopTiming {
  private static final class Sample {
    final String key;
    long microseconds;

    Sample(String name) {
      key = "LoopTiming/" + name + "MS";
    }
  }

  private final List<Sample> samples = new ArrayList<>();
  private final Map<String, Sample> robotSamples = new HashMap<>();
  private final Map<String, Sample> schedulerSamples = new HashMap<>();
  private final Map<String, Sample> periodicSamples = new HashMap<>();
  private final CommandScheduler scheduler;
  private Map<?, ?> robotEpochs;
  private Map<?, ?> schedulerEpochs;
  private Field schedulerDisabled;
  private long checkpoint;
  private long collectionTime;
  private long overrunCount;
  private boolean available;

  public LoopTiming(IterativeRobotBase robot, CommandScheduler scheduler) {
    this.scheduler = scheduler;
    try {
      robotEpochs = epochs(IterativeRobotBase.class, robot);
      schedulerEpochs = epochs(CommandScheduler.class, scheduler);
      schedulerDisabled = field(CommandScheduler.class, "m_disabled");
      if (schedulerDisabled.getType() != boolean.class) {
        throw new IllegalStateException("Unexpected WPILib scheduler disabled flag type");
      }
      available = true;
    } catch (ReflectiveOperationException
        | InaccessibleObjectException
        | SecurityException
        | IllegalStateException exception) {
      reportUnavailable(exception);
    }
    Logger.recordOutput("LoopTiming/WatchdogEpochsAvailable", available);
  }

  private static Field field(Class<?> owner, String name) throws NoSuchFieldException {
    Field field = owner.getDeclaredField(name);
    field.setAccessible(true);
    return field;
  }

  private static Map<?, ?> epochs(Class<?> owner, Object instance)
      throws ReflectiveOperationException {
    Object watchdog = field(owner, "m_watchdog").get(instance);
    if (!(watchdog instanceof Watchdog)) {
      throw new IllegalStateException("Unexpected WPILib watchdog type");
    }
    Object tracer = field(Watchdog.class, "m_tracer").get(watchdog);
    if (!(tracer instanceof Tracer)) {
      throw new IllegalStateException("Unexpected WPILib tracer type");
    }
    Object epochs = field(Tracer.class, "m_epochs").get(tracer);
    if (!(epochs instanceof Map<?, ?> map)) {
      throw new IllegalStateException("Unexpected WPILib epoch map type");
    }
    return map;
  }

  public void beginLoop() {
    long start = RobotController.getFPGATime();
    for (Sample sample : samples) {
      sample.microseconds = 0;
    }
    collectionTime = RobotController.getFPGATime() - start;
  }

  /** Starts a series of adjacent timing slices within robotPeriodic. */
  public void resetCheckpoint() {
    checkpoint = RobotController.getFPGATime();
  }

  /** Records elapsed real time since the previous checkpoint, not AdvantageKit's frozen clock. */
  public void checkpoint(String name) {
    long now = RobotController.getFPGATime();
    record(periodicSamples, "RobotPeriodic/", name, now - checkpoint);
    checkpoint = RobotController.getFPGATime();
    collectionTime += checkpoint - now;
  }

  public void captureScheduler() {
    long start = RobotController.getFPGATime();
    if (available) {
      try {
        if (!schedulerDisabled.getBoolean(scheduler)) {
          capture("SchedulerEpochs/", schedulerEpochs, schedulerSamples);
        }
      } catch (IllegalAccessException | IllegalStateException exception) {
        reportUnavailable(exception);
      }
    }
    collectionTime += RobotController.getFPGATime() - start;
  }

  public void endLoop(long loopStart, double periodSeconds) {
    long start = RobotController.getFPGATime();
    if (available) {
      try {
        capture("RobotEpochs/", robotEpochs, robotSamples);
      } catch (IllegalStateException exception) {
        reportUnavailable(exception);
      }
    }
    long publishStart = RobotController.getFPGATime();
    collectionTime += publishStart - start;
    for (Sample sample : samples) {
      Logger.recordOutput(sample.key, sample.microseconds / 1000.0);
    }
    long end = RobotController.getFPGATime();
    Logger.recordOutput("LoopTiming/CollectionMS", collectionTime / 1000.0);
    Logger.recordOutput("LoopTiming/PublishMS", (end - publishStart) / 1000.0);
    double totalMs = (RobotController.getFPGATime() - loopStart) / 1000.0;
    boolean overrun = totalMs > periodSeconds * 1000.0;
    if (overrun) {
      overrunCount++;
    }
    Logger.recordOutput("LoopTiming/TotalMS", totalMs);
    Logger.recordOutput("LoopTiming/Overrun", overrun);
    Logger.recordOutput("LoopTiming/OverrunCount", overrunCount);
  }

  private void capture(String prefix, Map<?, ?> epochs, Map<String, Sample> cache) {
    for (Map.Entry<?, ?> entry : epochs.entrySet()) {
      if (!(entry.getKey() instanceof String name)
          || !(entry.getValue() instanceof Long microseconds)) {
        throw new IllegalStateException("Unexpected WPILib watchdog epoch format");
      }
      record(cache, prefix, name, microseconds);
    }
  }

  private void record(Map<String, Sample> cache, String prefix, String name, long microseconds) {
    Sample sample = cache.get(name);
    if (sample == null) {
      sample = new Sample(prefix + name);
      cache.put(name, sample);
      samples.add(sample);
    }
    sample.microseconds += microseconds;
  }

  private void reportUnavailable(Exception exception) {
    available = false;
    Logger.recordOutput("LoopTiming/WatchdogEpochsAvailable", false);
    DriverStation.reportWarning(
        "Loop timing: WPILib watchdog epochs unavailable; explicit timings remain active. "
            + exception,
        false);
  }
}
