package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.lang.management.ThreadMXBean;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.LongSupplier;
import org.littletonrobotics.junction.Logger;

/** Main-thread CPU and JVM-wide GC counters sampled only around active loop work. */
final class LoopRuntimeMetrics {
  private final ThreadMXBean threads;
  private final GarbageCollectorMXBean[] collectors;
  private final long[] gcCounts;
  private final long[] gcTimes;
  private final LongSupplier nanoTime;
  private final Consumer<String> warning;
  private boolean cpuEnabled;
  private boolean cpuWarningReported;
  private long wallStart;
  private long cpuStart;
  private long userStart;
  double wallMs;
  double cpuMs = -1;
  double userMs = -1;
  double systemMs = -1;
  double offCpuMs = -1;
  long collections = -1;
  long collectionTimeMs = -1;

  LoopRuntimeMetrics() {
    this(
        ManagementFactory.getThreadMXBean(),
        ManagementFactory.getGarbageCollectorMXBeans(),
        System::nanoTime,
        message -> DriverStation.reportWarning(message, false));
  }

  LoopRuntimeMetrics(
      ThreadMXBean threads,
      List<GarbageCollectorMXBean> collectors,
      LongSupplier nanoTime,
      Consumer<String> warning) {
    this.threads = threads;
    this.collectors = collectors.toArray(GarbageCollectorMXBean[]::new);
    gcCounts = new long[this.collectors.length];
    gcTimes = new long[this.collectors.length];
    this.nanoTime = nanoTime;
    this.warning = warning;
    try {
      if (threads.isCurrentThreadCpuTimeSupported()) {
        if (!threads.isThreadCpuTimeEnabled()) {
          threads.setThreadCpuTimeEnabled(true);
        }
        cpuEnabled = threads.isThreadCpuTimeEnabled();
      }
    } catch (UnsupportedOperationException | SecurityException exception) {
      warnCpuUnavailable(exception.toString());
    }
    if (!cpuEnabled) {
      warnCpuUnavailable("Current-thread CPU timing is unsupported or could not be enabled.");
    }
  }

  void begin() {
    for (int i = 0; i < collectors.length; i++) {
      gcCounts[i] = collectors[i].getCollectionCount();
      gcTimes[i] = collectors[i].getCollectionTime();
    }
    userStart = cpuEnabled ? threads.getCurrentThreadUserTime() : -1;
    wallStart = nanoTime.getAsLong();
    cpuStart = readCpu();
  }

  void end() {
    long cpuEnd = readCpu();
    wallMs = (nanoTime.getAsLong() - wallStart) / 1.0e6;
    long userEnd = cpuEnd >= 0 ? threads.getCurrentThreadUserTime() : -1;
    long cpuNanos = delta(cpuStart, cpuEnd);
    cpuMs = cpuNanos < 0 ? -1 : cpuNanos / 1.0e6;
    userMs = -1;
    systemMs = -1;
    offCpuMs = -1;
    if (cpuMs >= 0) {
      offCpuMs = Math.max(0, wallMs - cpuMs);
      if (userStart >= 0 && userEnd >= userStart) {
        userMs = (userEnd - userStart) / 1.0e6;
        systemMs = Math.max(0, cpuMs - userMs);
      }
    }
    long count = 0;
    long time = 0;
    for (int i = 0; i < collectors.length; i++) {
      long countDelta = delta(gcCounts[i], collectors[i].getCollectionCount());
      long timeDelta = delta(gcTimes[i], collectors[i].getCollectionTime());
      count = count < 0 || countDelta < 0 ? -1 : count + countDelta;
      time = time < 0 || timeDelta < 0 ? -1 : time + timeDelta;
    }
    collections = collectors.length == 0 ? -1 : count;
    collectionTimeMs = collectors.length == 0 ? -1 : time;
  }

  void publish() {
    Logger.recordOutput("LoopTiming/Thread/CpuTimeAvailable", cpuMs >= 0);
    Logger.recordOutput("LoopTiming/Thread/UserTimeAvailable", userMs >= 0);
    Logger.recordOutput("LoopTiming/Thread/WallMS", wallMs);
    Logger.recordOutput("LoopTiming/Thread/CpuMS", cpuMs);
    Logger.recordOutput("LoopTiming/Thread/UserMS", userMs);
    Logger.recordOutput("LoopTiming/Thread/SystemMS", systemMs);
    Logger.recordOutput("LoopTiming/Thread/OffCpuMS", offCpuMs);
    Logger.recordOutput("LoopTiming/GC/CollectionCountAvailable", collections >= 0);
    Logger.recordOutput("LoopTiming/GC/CollectionTimeAvailable", collectionTimeMs >= 0);
    Logger.recordOutput("LoopTiming/GC/Collections", collections);
    Logger.recordOutput("LoopTiming/GC/CollectionTimeMS", collectionTimeMs);
    Logger.recordOutput("LoopTiming/GC/ActivityDetected", collections > 0 || collectionTimeMs > 0);
  }

  private long readCpu() {
    if (!cpuEnabled) {
      return -1;
    }
    long cpu = threads.getCurrentThreadCpuTime();
    if (cpu < 0) {
      cpuEnabled = false;
      warnCpuUnavailable("The JVM returned an unavailable CPU counter.");
    }
    return cpu;
  }

  private void warnCpuUnavailable(String reason) {
    if (!cpuWarningReported) {
      cpuWarningReported = true;
      warning.accept(
          "Loop timing: CPU timing unavailable; wall and GC metrics remain active. " + reason);
    }
  }

  private static long delta(long start, long end) {
    return start < 0 || end < start ? -1 : end - start;
  }
}
