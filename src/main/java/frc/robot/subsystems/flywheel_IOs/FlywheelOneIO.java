package frc.robot.subsystems.flywheel_IOs;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelOneIO {
    @AutoLog
    public static class FlywheelOneIOInputs {
        public double velocity = 0;
        public double statorCurrent = 0;
        public double supplyCurrent = 0;
        public double motorTemp = 0;

        public boolean isConnected = false;
    }
    public default void updateInputs(FlywheelOneIOInputs inputs) {}
}
