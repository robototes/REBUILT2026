package frc.robot.subsystems.index;

public interface SpindexerIO {

  public static class SpindexerIOInputs {
    public double velocityRPS = 0.0;
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
  }

  public void updateInputs(SpindexerIOInputs inputs);
}
