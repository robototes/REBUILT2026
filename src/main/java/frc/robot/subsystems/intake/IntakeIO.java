package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    // rollers velocity
    public double currentVelocityMotor1;
    public double currentVelocityMotor2;
    // rollers stator and supply
    public double statorCurrentRoller1;
    public double supplyCurrentRoller1;
    public double statorCurrentRoller2;
    public double supplyCurrentRoller2;

    public boolean isConnected1;
    public boolean isConnected2;
    public double motorTemp1;
    public double motorTemp2;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}
}
