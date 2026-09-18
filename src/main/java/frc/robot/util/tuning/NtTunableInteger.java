package frc.robot.util.tuning;

import org.wpilib.networktables.IntegerPublisher;
import org.wpilib.networktables.IntegerSubscriber;
import org.wpilib.networktables.IntegerTopic;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.TimestampedInteger;

/**
 * A pure NT4 tunable integer. This ensures the topic is published (visible) and subscribed
 * (editable).
 */
public final class NtTunableInteger {
  private final IntegerTopic topic;
  private final IntegerPublisher pub;
  private final IntegerSubscriber sub;

  /**
   * @param path Full NT path (e.g., "/SmartDashboard/LaunchCalculator/iterations")
   * @param defaultValue Initial value to publish
   */
  public NtTunableInteger(String path, int defaultValue) {
    var nt = NetworkTableInstance.getDefault();
    this.topic = nt.getIntegerTopic(path);

    // Create both to ensure it shows up in dashboards and reads edits back
    this.pub = topic.publish();
    this.sub = topic.subscribe(defaultValue);

    // Force the initial value so it appears in the tree immediately
    this.pub.set(defaultValue);
  }

  /** Gets the latest value from the dashboard (or the default if never changed). */
  public int get() {
    return Math.toIntExact(sub.get());
  }

  public TimestampedInteger getAtomic() {
    return sub.getAtomic();
  }

  /** Updates the value from the robot side. */
  public void set(int value) {
    pub.set(value);
  }

  public boolean hasChangedSince(long lastChangeTime) {
    return sub.getLastChange() > lastChangeTime;
  }

  public void close() {
    pub.close();
    sub.close();
  }
}
