package frc.robot.subsystems.index;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
  @AutoLog
  public static class FeederIOInputs {
    public double feederRPS;
    public double statorCurrent;
    public double supplyCurrent;
    public double motorTemp;
    public boolean isConnected;
  }

  public default void updateInputs(FeederIOInputs inputs) {}
}
