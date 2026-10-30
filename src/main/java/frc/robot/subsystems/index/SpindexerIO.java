package frc.robot.subsystems.spindexer;

import org.littletonrobotics.junction.AutoLog;

public interface SpindexerIO {

  @AutoLog
  public static class SpindexerIOInputs {
    public double rps;
    public double stator;
    public double supply;
  }

  public default void updateInputs(SpindexerIOInputs inputs) {}
}
