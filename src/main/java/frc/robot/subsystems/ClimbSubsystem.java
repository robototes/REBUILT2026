package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;

public class ClimbSubsystem extends SubsystemBase {

  public static final double SERVO_STOWED = 0.0;
  public static final double SERVO_DEPLOYED = 0.5;

  private static final double CLIMB_VOLTAGE = 8.0;

  private final Servo pivotServo = new Servo(Hardware.CLIMB_PIVOT_SERVO_CHANNEL);
  private final TalonFX driveMotor = new TalonFX(Hardware.CLIMB_DRIVE_MOTOR_ID);

  private final VoltageOut driveVoltageReq = new VoltageOut(0);

  public ClimbSubsystem() {
    configureDrive();
  }

  private void configureDrive() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    driveMotor.getConfigurator().apply(cfg);
  }

  public void deploy() {
    pivotServo.set(SERVO_DEPLOYED);
  }

  public void stow() {
    pivotServo.set(SERVO_STOWED);
  }

  public boolean isDeployed() {
    return pivotServo.get() == SERVO_DEPLOYED;
  }

  public void setDriveVoltage(double volts) {
    driveMotor.setControl(driveVoltageReq.withOutput(volts));
  }

  public void stopDrive() {
    driveMotor.stopMotor();
  }

  public Command deployCommand() {
    return runOnce(() -> deploy()).withName("Deploy Climb");
  }

  public Command stowCommand() {
    return runOnce(() -> stow()).withName("Stow Climb");
  }

  public Command climbCommand() {
    return deployCommand()
        .andThen(upCommand())
        .withName("Climb");
  }

  public Command upCommand() {
    return runEnd(() -> setDriveVoltage(CLIMB_VOLTAGE), () -> stopDrive()).withName("Climb Up");
  }

  public Command lowerCommand() {
    return runEnd(() -> setDriveVoltage(-CLIMB_VOLTAGE), () -> stopDrive()).withName("Lower Climb");
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber/Servo Position", pivotServo.get());
    SmartDashboard.putBoolean("Climber/Deployed", isDeployed());
    SmartDashboard.putNumber(
        "Climber/Drive Supply", driveMotor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber(
        "Climber/Drive Voltage", driveMotor.getMotorVoltage().getValueAsDouble());
  }
}
