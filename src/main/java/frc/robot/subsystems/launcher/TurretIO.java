package frc.robot.subsystems.launcher;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {

  @AutoLog
  public static class TurretIOInputs {
    public double positionRotations;
    public double velocityRPS;
    public double statorCurrentAmps;
    public double limitSwitchVolts;
    public double motorTemp;
    public boolean isConnected;
  }

  public default void updateInputs(TurretIOInputs inputs) {}
}
