package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class HardwareConstants {
    public static final int feederMotorID = 1;
    public static final int feederDigitalInputChannel = 0;
  }
  public static class TuningConstants {
    public static final double feederSpeed = 1.0;
    public static final int feederRumbleThreshold = 67;
  }
}
