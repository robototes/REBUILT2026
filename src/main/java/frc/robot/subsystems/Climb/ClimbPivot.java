package frc.robot.subsystems.Climb;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;

public class ClimbPivot extends SubsystemBase {
  public static final double SERVO_STOWED = 0.0;
  public static final double SERVO_DEPLOYED = 0.5;
  public static final double PIVOT_DELAY_SECONDS = 0.5;
  private final Servo pivotServo = new Servo(Hardware.CLIMB_PIVOT_SERVO_CHANNEL);

  private final DoublePublisher nt_targetPos;
  private final BooleanPublisher nt_IsDeployed;

  public ClimbPivot() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    nt_targetPos = inst.getDoubleTopic("SmartDashboard/ClimbPivot/TargetPos").publish();
    nt_IsDeployed = inst.getBooleanTopic("SmartDashboard/ClimbPivot/IsDeployed").publish();
  }

  public void deploy() {
    pivotServo.set(SERVO_DEPLOYED);
    nt_targetPos.set(SERVO_DEPLOYED);
    nt_IsDeployed.set(true);
  }

  public void stow() {
    pivotServo.set(SERVO_STOWED);
    nt_targetPos.set(SERVO_STOWED);
    nt_IsDeployed.set(false);
  }

  public Command deployCommand() {
    return Commands.runOnce(this::deploy, this).withName("Deploy pivot");
  }

  public Command stowCommand() {
    return Commands.runOnce(this::stow, this).withName("Stow Pivot");
  }
}
