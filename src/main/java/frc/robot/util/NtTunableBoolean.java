package frc.robot.util;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedBoolean;

/**
 * A pure NT4 tunable boolean that requires no Shuffleboard/SmartDashboard. Publish a default once;
 * read the current value (including edits from Elastic).
 */
public final class NtTunableBoolean {
  private final BooleanTopic topic;
  private final BooleanPublisher pub;
  private final BooleanSubscriber sub;

  /**
   * @param path Full NT path (e.g., "/tuning/intakeSpeed")
   * @param defaultValue Initial/default value to publish and subscribe to
   */
  public NtTunableBoolean(String path, boolean defaultValue) {
    var nt = NetworkTableInstance.getDefault();
    this.topic = nt.getBooleanTopic(path);
    // publisher stays alive while this object exists
    this.pub = topic.publish();
    this.sub = topic.subscribe(defaultValue);
    // make the topic visible immediately
    this.pub.set(defaultValue);
  }

  // Latest value, including edits from Elastic’s slider/text widget.
  public boolean get() {
    return sub.get();
  }

  public TimestampedBoolean getAtomic() {
    return sub.getAtomic();
  }

  // Optionally update from robot code (e.g., to reflect a new default).
  public void set(boolean value) {
    pub.set(value);
  }

  // Check if the value has changed since a given timestamp from getAtomic
  public boolean hasChangedSince(long lastChangeTime) {
    return sub.getLastChange() > lastChangeTime;
  }

  // Call when done to stop publishing (optional but good hygiene).
  public void close() {
    pub.close();
    sub.close();
  }
}
