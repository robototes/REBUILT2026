package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;

// This is a hack to re-expose the package-private Choreo field sizes
public class FieldSize {
  public static final double FIELD_LENGTH_M = 16.541;
  public static final double FIELD_WIDTH_M = 8.0692;
  public static final Distance FIELD_WIDTH = Meters.of(FIELD_WIDTH_M);
  public static final Distance FIELD_LENGTH = Meters.of(FIELD_LENGTH_M);
}
