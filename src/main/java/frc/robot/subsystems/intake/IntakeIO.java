package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    // rollers velocity
    public double currentVelocityMotor1 = 0;
    public double currentVelocityMotor2 = 0;
    // rollers stator and supply
    public double statorCurrentRoller1 = 0;
    public double supplyCurrentRoller1 = 0;
    public double statorCurrentRoller2 = 0;
    public double supplyCurrentRoller2 = 0;

    public boolean isConnected1 = false;
    public boolean isConnected2 = false;
    public double motorTemp1 = 0;
    public double motorTemp2 = 0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}
}
