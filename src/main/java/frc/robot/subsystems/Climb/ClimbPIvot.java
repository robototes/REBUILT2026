package frc.robot.subsystems.Climb;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;

public class ClimbPivot extends SubsystemBase {
  public static final double SERVO_STOWED = 0.0;
  public static final double SERVO_DEPLOYED = 0.5;

  private final Servo pivotServo = new Servo(Hardware.CLIMB_PIVOT_SERVO_CHANNEL);

  public ClimbPivot() {}

  public void deploy() {
    pivotServo.set(SERVO_DEPLOYED);
  }

  public void stow() {
    pivotServo.set(SERVO_STOWED);
  }

  public boolean isDeployed() {
    return pivotServo.get() == SERVO_DEPLOYED;
  }

  public Command deployCommand() {
    return runOnce(this::deploy).withName("Deploy Pivot");
  }

  public Command stowCommand() {
    return runOnce(this::stow).withName("Stow Pivot");
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pivot/Servo Position", pivotServo.get());
    SmartDashboard.putBoolean("Pivot/Deployed", isDeployed());
  }
}
