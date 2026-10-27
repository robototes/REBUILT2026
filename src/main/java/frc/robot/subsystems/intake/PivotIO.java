package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
  @AutoLog
  public static class PivotIOInputs {
    public double currentPos = 0; // pivot position
    public boolean isConnected = false;
    public double motorTemp = 0;
    // pivot stator and supply
    public double statorCurrentPivot = 0;
    public double supplyCurrentPivot = 0;
  }

  public default void updateInputs(PivotIOInputs inputs) {}
}
