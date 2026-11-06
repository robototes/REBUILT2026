package frc.robot.subsystems.launcher;

public interface HoodIO  {

    public static class HoodIOInputs {
        public double hoodStatorCurrent = 0;
        public double hoodSupplyCurrent = 0;
        public double position = 0;
    }

    public default void updateInputs(HoodIOInputs inputs) {}
}
