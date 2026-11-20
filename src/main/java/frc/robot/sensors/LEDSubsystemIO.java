package frc.robot.sensors;

import org.littletonrobotics.junction.AutoLog;
import frc.robot.sensors.LEDSubsystem.LEDMode;
import frc.robot.sensors.LEDSubsystem.LEDPattern;

import com.ctre.phoenix6.signals.RGBWColor;

public interface LEDSubsystemIO {
    @AutoLog
    public class LEDSubsystemIOInputs {
        public RGBWColor primaryColor;
        public RGBWColor secondaryColor;

        private LEDMode currentMode = LEDMode.DEFAULT;
        private LEDPattern currentPattern = LEDPattern.SOLID;

        public double interval = 0.25;
        public double lastToggleTime = 0;
        public boolean showingPrimary = true;
    }

    public default void updateInputs(){}
}
