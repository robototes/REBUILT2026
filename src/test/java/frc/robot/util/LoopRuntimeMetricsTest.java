package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.lang.management.ThreadMXBean;
import java.lang.reflect.InvocationHandler;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

class LoopRuntimeMetricsTest {
  private static final class Counters {
    long wall;
    long cpu;
    long user;
    long count;
    long gcTime;
    boolean supported = true;
    boolean enabled;
    boolean denyEnable;
    final List<String> warnings = new ArrayList<>();
    final ThreadMXBean threads =
        bean(
            ThreadMXBean.class,
            (proxy, method, args) -> {
              return switch (method.getName()) {
                case "isCurrentThreadCpuTimeSupported" -> supported;
                case "isThreadCpuTimeEnabled" -> enabled;
                case "setThreadCpuTimeEnabled" -> {
                  if (denyEnable) {
                    throw new SecurityException("Test permission denied");
                  }
                  enabled = (boolean) args[0];
                  yield null;
                }
                case "getCurrentThreadCpuTime" -> cpu;
                case "getCurrentThreadUserTime" -> user;
                default -> throw new AssertionError(method.getName());
              };
            });
    final GarbageCollectorMXBean collector =
        bean(
            GarbageCollectorMXBean.class,
            (proxy, method, args) ->
                switch (method.getName()) {
                  case "getCollectionCount" -> count;
                  case "getCollectionTime" -> gcTime;
                  default -> throw new AssertionError(method.getName());
                });

    LoopRuntimeMetrics tracker() {
      return new LoopRuntimeMetrics(threads, List.of(collector), () -> wall, warnings::add);
    }
  }

  @Test
  void measuresActiveWorkAndDoesNotIncludeTimeBetweenLoops() {
    Counters counters = new Counters();
    counters.wall = 1_000_000_000L;
    counters.cpu = 100_000_000L;
    counters.user = 80_000_000L;
    LoopRuntimeMetrics metrics = counters.tracker();
    assertTrue(counters.enabled);
    metrics.begin();
    counters.wall += 20_000_000;
    counters.cpu += 8_000_000;
    counters.user += 6_000_000;
    counters.count += 2;
    counters.gcTime += 5;
    metrics.end();
    assertEquals(20, metrics.wallMs);
    assertEquals(8, metrics.cpuMs);
    assertEquals(6, metrics.userMs);
    assertEquals(2, metrics.systemMs);
    assertEquals(12, metrics.offCpuMs);
    assertEquals(2, metrics.collections);
    assertEquals(5, metrics.collectionTimeMs);

    counters.wall += 1_000_000_000L;
    counters.cpu += 100_000_000;
    counters.user += 100_000_000;
    counters.count++;
    counters.gcTime += 20;
    metrics.begin();
    counters.wall += 4_000_000;
    counters.cpu += 5_000_000;
    counters.user += 5_000_000;
    metrics.end();
    assertEquals(4, metrics.wallMs);
    assertEquals(5, metrics.cpuMs);
    assertEquals(0, metrics.offCpuMs, "Small clock/granularity differences must not go negative");
    assertEquals(0, metrics.collections);
    assertEquals(0, metrics.collectionTimeMs);
    assertTrue(counters.warnings.isEmpty());
  }

  @Test
  void unsupportedOrDeniedCpuTimingPreservesWallAndGcMeasurements() {
    for (boolean denied : List.of(false, true)) {
      Counters counters = new Counters();
      counters.supported = denied;
      counters.denyEnable = denied;
      LoopRuntimeMetrics metrics = counters.tracker();
      metrics.begin();
      counters.wall += 12_000_000;
      counters.count++;
      counters.gcTime++;
      metrics.end();
      assertEquals(12, metrics.wallMs);
      assertEquals(-1, metrics.cpuMs);
      assertEquals(-1, metrics.userMs);
      assertEquals(-1, metrics.systemMs);
      assertEquals(-1, metrics.offCpuMs);
      assertEquals(1, metrics.collections);
      assertEquals(1, metrics.collectionTimeMs);
      assertEquals(1, counters.warnings.size());
    }
  }

  @Test
  void unavailableAndResetCountersNeverBecomePlausibleZeroMeasurements() {
    Counters counters = new Counters();
    counters.cpu = 10_000_000;
    counters.user = -1;
    counters.count = -1;
    counters.gcTime = -1;
    LoopRuntimeMetrics metrics = counters.tracker();
    metrics.begin();
    counters.wall += 5_000_000;
    counters.cpu += 1_000_000;
    metrics.end();
    assertEquals(1, metrics.cpuMs);
    assertEquals(4, metrics.offCpuMs);
    assertEquals(-1, metrics.userMs);
    assertEquals(-1, metrics.systemMs);
    assertEquals(-1, metrics.collections);
    assertEquals(-1, metrics.collectionTimeMs);

    metrics.begin();
    counters.cpu = -1;
    metrics.end();
    assertEquals(-1, metrics.cpuMs);
    assertEquals(-1, metrics.offCpuMs);
    metrics.begin();
    metrics.end();
    assertEquals(1, counters.warnings.size());

    counters = new Counters();
    counters.count = 5;
    counters.gcTime = 20;
    metrics = counters.tracker();
    metrics.begin();
    counters.cpu = -2;
    counters.count = 4;
    counters.gcTime = 19;
    metrics.end();
    assertEquals(-1, metrics.collections);
    assertEquals(-1, metrics.collectionTimeMs);
  }

  @Test
  void emptyOrPartiallyUnsupportedCollectorListsAreExplicitlyUnavailable() {
    Counters counters = new Counters();
    LoopRuntimeMetrics metrics =
        new LoopRuntimeMetrics(
            counters.threads, List.of(), () -> counters.wall, counters.warnings::add);
    metrics.begin();
    metrics.end();
    assertEquals(-1, metrics.collections);
    assertEquals(-1, metrics.collectionTimeMs);

    Counters other = new Counters();
    other.gcTime = -1;
    metrics =
        new LoopRuntimeMetrics(
            counters.threads,
            List.of(counters.collector, other.collector),
            () -> counters.wall,
            counters.warnings::add);
    metrics.begin();
    counters.count += 2;
    other.count += 3;
    counters.gcTime += 6;
    metrics.end();
    assertEquals(5, metrics.collections);
    assertEquals(-1, metrics.collectionTimeMs);
  }

  @Test
  void realMxBeanDistinguishesBusyWorkFromSleeping() throws InterruptedException {
    ThreadMXBean threads = ManagementFactory.getThreadMXBean();
    assumeTrue(threads.isCurrentThreadCpuTimeSupported());
    LoopRuntimeMetrics metrics =
        new LoopRuntimeMetrics(
            threads,
            List.of(),
            System::nanoTime,
            message -> {
              throw new AssertionError(message);
            });
    metrics.begin();
    long startCpu = threads.getCurrentThreadCpuTime();
    long deadline = System.nanoTime() + 2_000_000_000L;
    while (threads.getCurrentThreadCpuTime() - startCpu < 20_000_000L
        && System.nanoTime() < deadline) {
      Thread.onSpinWait();
    }
    metrics.end();
    assertTrue(metrics.cpuMs >= 20, "Busy work must consume measurable thread CPU");
    assertTrue(metrics.offCpuMs >= 0);

    metrics.begin();
    Thread.sleep(40);
    metrics.end();
    assertTrue(metrics.wallMs >= 30);
    assertTrue(metrics.offCpuMs >= 20, "Sleeping should be mostly off-CPU");
    assertTrue(metrics.cpuMs < metrics.wallMs);
  }

  private static <T> T bean(Class<T> type, InvocationHandler handler) {
    return type.cast(Proxy.newProxyInstance(type.getClassLoader(), new Class<?>[] {type}, handler));
  }
}
