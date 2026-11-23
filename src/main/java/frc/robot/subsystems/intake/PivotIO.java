package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
  @AutoLog
  public static class PivotIOInputs {
    public double currentPos; // pivot position
    public boolean isConnected;
    public double motorTemp;
    // pivot stator and supply
    public double statorCurrentPivot;
    public double supplyCurrentPivot;
  }

  public default void updateInputs(PivotIOInputs inputs) {}
}
