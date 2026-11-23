package frc.robot.sensors;

import com.ctre.phoenix6.signals.RGBWColor;

public interface LEDSubsystemIO {

    default void setHardwareColor(RGBWColor color, double brightness) {}

    default void setHardwareColor(RGBWColor color) {
        setHardwareColor(color, 1.0);
    }

    default void setRainbow() {}

}
