package frc.robot.subsystems.launcher;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {

  @AutoLog
  public static class TurretIOInputs {
    public double positionSignal;
    public double velocitySignal;
    public double statorCurrentSignal;
    public double limitSwitchCurrent;
  }

  public default void updateInputs(TurretIOInputs inputs) {}
}
