package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    // rollers velocity
    public StatusSignal currentVelocityMotor1;
    public StatusSignal currentVelocityMotor2;
    // rollers stator and supply
    public StatusSignal statorCurrentRoller1;
    public StatusSignal supplyCurrentRoller1;
    public StatusSignal statorCurrentRoller2;
    public StatusSignal supplyCurrentRoller2;

    public StatusSignal isConnected1;
    public StatusSignal isConnected2;
    public StatusSignal motorTemp1;
    public StatusSignal motorTemp2;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}
}
