package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
  @AutoLog
  public static class PivotIOInputs {
    public StatusSignal currentPos; // pivot position
    public StatusSignal targetPos;
    public StatusSignal isConnected;
    public StatusSignal motorTemp;
    // pivot stator and supply
    public StatusSignal statorCurrentPivot;
    public StatusSignal supplyCurrentPivot;
  }

  public default void updateInputs(PivotIOInputs inputs) {}
}
