package frc.robot.util;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDouble;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedNetworkInput;

/** Manages a number value published to the root table of NT. */
public class CustomLoggedNetworkNumber extends LoggedNetworkInput implements DoubleSupplier {
  private final String key;
  private final DoubleEntry entry;
  private double defaultValue = 0.0;
  private double value;

  /**
   * Creates a new LoggedNetworkNumber, for handling a number input sent via NetworkTables.
   *
   * @param key The key for the number, published to the root table of NT or
   *     "/DashboardInputs/{key}" when logged.
   */
  public CustomLoggedNetworkNumber(String key) {
    this.key = key;
    this.entry = NetworkTableInstance.getDefault().getDoubleTopic(key).getEntry(0.0);
    this.value = defaultValue;
    Logger.registerDashboardInput(this);
  }

  /**
   * Creates a new LoggedNetworkNumber, for handling a number input sent via NetworkTables.
   *
   * @param key The key for the number, published to the root table of NT or
   *     "/DashboardInputs/{key}" when logged.
   * @param defaultValue The default value if no value in NT is found.
   */
  public CustomLoggedNetworkNumber(String key, double defaultValue) {
    this(key);
    setDefault(defaultValue);
    this.value = defaultValue;
  }

  /**
   * Updates the default value, which is used if no value in NT is found.
   *
   * @param defaultValue The new default value.
   */
  public void setDefault(double defaultValue) {
    this.defaultValue = defaultValue;
    entry.set(entry.get(defaultValue));
  }

  /**
   * Publishes a new value. Note that the value will not be returned by {@link #get()} until the
   * next cycle.
   *
   * @param value The new value.
   */
  public void set(double value) {
    entry.set(value);
  }

  /**
   * Returns the current value.
   *
   * @return The current value.
   */
  public double get() {
    return value;
  }

  private final LoggableInputs inputs =
      new LoggableInputs() {
        public void toLog(LogTable table) {
          table.put(removeSlash(key), value);
        }

        public void fromLog(LogTable table) {
          value = table.get(removeSlash(key), defaultValue);
        }
      };

  public void periodic() {
    if (!Logger.hasReplaySource()) {
      value = entry.get(defaultValue);
    }
    Logger.processInputs(prefix, inputs);
  }

  @Override
  public double getAsDouble() {
    return value;
  }

  public TimestampedDouble getAtomic() {
    return entry.getAtomic();
  }

  public boolean hasChangedSince(long lastChangeTime) {
    return entry.getLastChange() > lastChangeTime;
  }
}
