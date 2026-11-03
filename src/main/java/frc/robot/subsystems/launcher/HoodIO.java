package frc.robot.subsystems.launcher;

import java.io.ObjectInputFilter.Status;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Angle;

public interface HoodIO  {

    public static class HoodIOInputs {
        public double hoodStatorCurrent = 0;
        public double hoodSupplyCurrent = 0;
        public double position = 0;
    }

    public default void updateInputs(HoodIOInputs inputs) {}

}
