package frc.robot.util;

import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.util.concurrent.atomic.AtomicLong;
import java.util.logging.Logger;

import javax.management.Notification;
import javax.management.NotificationListener;
import javax.management.openmbean.CompositeData;

import com.sun.management.GarbageCollectionNotificationInfo;
import javax.management.NotificationEmitter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * GCMonitor registers JMX listeners for garbage collection notifications and
 * increments an internal counter each time a GC occurs. It also writes the
 * count to SmartDashboard under the key "GCCount" and logs an info line.
 *
 * Enable by calling {@link #start()} early in application startup (Robot constructor).
 */
public final class GCMonitor {
  private static final AtomicLong gcCount = new AtomicLong(0);
  private static final Logger logger = Logger.getLogger(GCMonitor.class.getName());
  private static volatile boolean started = false;

  private GCMonitor() {}

  public static void start() {
    if (started) {
      return;
    }
    try {
      for (GarbageCollectorMXBean gcBean : ManagementFactory.getGarbageCollectorMXBeans()) {
        if (gcBean instanceof NotificationEmitter) {
          NotificationEmitter emitter = (NotificationEmitter) gcBean;
          NotificationListener listener = new NotificationListener() {
            @Override
            public void handleNotification(Notification notification, Object handback) {
              if (GarbageCollectionNotificationInfo.GARBAGE_COLLECTION_NOTIFICATION.equals(notification.getType())) {
                try {
                  CompositeData cd = (CompositeData) notification.getUserData();
                  GarbageCollectionNotificationInfo info = GarbageCollectionNotificationInfo.from(cd);
                  long current = gcCount.incrementAndGet();
                  // Log a concise message and publish to SmartDashboard for easy visibility in WPILib tools
                  logger.info(String.format("GC #%d action=%s name=%s duration=%dms", current, info.getGcAction(), info.getGcName(), info.getGcInfo().getDuration()));
                  try {
                    SmartDashboard.putNumber("GCCount", current);
                  } catch (Throwable t) {
                    // ignore SmartDashboard errors (e.g., not initialized in some contexts)
                  }
                } catch (Throwable t) {
                  logger.warning("Error processing GC notification: " + t);
                }
              }
            }
          };
          emitter.addNotificationListener(listener, null, null);
        }
      }
      started = true;
      logger.info("GCMonitor started");
    } catch (Throwable t) {
      logger.warning("GCMonitor failed to start: " + t);
    }
  }

  public static long getGcCount() {
    return gcCount.get();
  }
}
