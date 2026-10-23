package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        // rollers velocity
        public double currentVelocityMotor1 = 0;
        public double currentVelocityMotor2 = 0;
        public double currentPos = 0; // pivot position
        // rollers stator and supply
        public double statorCurrentRoller1 = 0;
        public double supplyCurrentRoller1 = 0;
        public double statorCurrentRoller2 = 0;
        public double supplyCurrentRoller2 = 0;
        // pivot stator and supply
        public double statorCurrentPivot = 0;
        public double supplyCurrentPivot = 0;
    }
    public default void updateInputs(IntakeIOInputs inputs) {}
}
