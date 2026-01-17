package frc.robot.util;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDouble;

/**
 * A pure NT4 tunable double that requires no Shuffleboard/SmartDashboard. Publish a default once;
 * read the current value (including edits from Elastic).
 */
public final class NtTunableDouble {
  private final DoubleTopic topic;
  private final DoublePublisher pub;
  private final DoubleSubscriber sub;

  /**
   * @param path Full NT path (e.g., "/tuning/intakeSpeed")
   * @param defaultValue Initial/default value to publish and subscribe to
   */
  public NtTunableDouble(String path, double defaultValue) {
    var nt = NetworkTableInstance.getDefault();
    this.topic = nt.getDoubleTopic(path);
    // publisher stays alive while this object exists
    this.pub = topic.publish();
    this.sub = topic.subscribe(defaultValue);
    // make the topic visible immediately
    this.pub.set(defaultValue);
  }

  // Latest value, including edits from Elastic’s slider/text widget.
  public double get() {
    return sub.get();
  }

  public TimestampedDouble getAtomic() {
    return sub.getAtomic();
  }

  // Optionally update from robot code (e.g., to reflect a new default).
  public void set(double value) {
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
