package frc.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.function.BooleanSupplier;

public class SlowMode {
  private static final double SLOW_FACTOR = 0.1;
  private static final double DEFAULT_FACTOR = 1.0;
  private final NetworkTable table = NetworkTableInstance.getDefault().getTable("SlowMode");
  private final NetworkTableEntry slowModeEntry;

  public SlowMode() {
    slowModeEntry = table.getEntry("slowMode");
    slowModeEntry.setBoolean(false);
  }

  public boolean isSlowMode() {
    return slowModeEntry.getBoolean(false);
  }

  public double slowFactor() {
    return isSlowMode() ? SLOW_FACTOR : DEFAULT_FACTOR;
  }
}
}
