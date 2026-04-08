package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;

public class ClimbSubsystem extends SubsystemBase {

  public static final double PIVOT_STOWED = 0.0;
  public static final double PIVOT_DEPLOYED = 12.5;

  private static final double CLIMB_VOLTAGE = 8.0;
  private static final double LOWER_VOLTAGE = -6.0;

  private static final double PIVOT_KP = 60.0;
  private static final double PIVOT_KI = 0.0;
  private static final double PIVOT_KD = 1.5;
  private static final double PIVOT_KV = 0.12;
  private static final double PIVOT_KA = 0.01;

  private static final double PIVOT_MM_CRUISE = 40.0;
  private static final double PIVOT_MM_ACCEL = 80.0;
  private static final double PIVOT_MM_JERK = 400.0;

  private final TalonFX pivotMotor = new TalonFX(Hardware.CLIMB_PIVOT_MOTOR_ID);
  private final TalonFX driveMotor = new TalonFX(Hardware.CLIMB_DRIVE_MOTOR_ID);

  private final MotionMagicVoltage pivotPositionReq = new MotionMagicVoltage(0);
  private final VoltageOut driveVoltageReq = new VoltageOut(0);

  public ClimbSubsystem() {
    configurePivot();
    configureDrive();
  }

  private void configurePivot() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    Slot0Configs slot0 = cfg.Slot0;
    slot0.kP = PIVOT_KP;
    slot0.kI = PIVOT_KI;
    slot0.kD = PIVOT_KD;
    slot0.kV = PIVOT_KV;
    slot0.kA = PIVOT_KA;

    MotionMagicConfigs mm = cfg.MotionMagic;
    mm.MotionMagicCruiseVelocity = PIVOT_MM_CRUISE;
    mm.MotionMagicAcceleration = PIVOT_MM_ACCEL;
    mm.MotionMagicJerk = PIVOT_MM_JERK;

    cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    pivotMotor.getConfigurator().apply(cfg);
    pivotMotor.setPosition(0.0);
  }

  private void configureDrive() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    driveMotor.getConfigurator().apply(cfg);
  }

  public void setPivotPosition(double rotations) {
    pivotMotor.setControl(pivotPositionReq.withPosition(rotations));
  }

  public boolean isPivotAtTarget(double targetRot, double toleranceRot) {
    double current = pivotMotor.getPosition().getValueAsDouble();
    return Math.abs(current - targetRot) < toleranceRot;
  }

  public void setDriveVoltage(double volts) {
    driveMotor.setControl(driveVoltageReq.withOutput(volts));
  }

  public void stopDrive() {
    driveMotor.stopMotor();
  }

  public void zeroPivot() {
    pivotMotor.setPosition(0.0);
  }

  public boolean isDeployed() {
    return isPivotAtTarget(PIVOT_DEPLOYED, 0.1);
  }

  public Command deployCommand() {
    return runOnce(() -> setPivotPosition(PIVOT_DEPLOYED)).withName("Deploy Climb");
  }

  public Command stowCommand() {
    return runOnce(() -> setPivotPosition(PIVOT_STOWED)).withName("Stow Climb");
  }

  public Command climbCommand() {
    return deployCommand()
        .andThen(runEnd(() -> setDriveVoltage(CLIMB_VOLTAGE), () -> stopDrive()))
        .withName("Climb");
  }

  public Command lowerCommand() {
    return runEnd(() -> setDriveVoltage(LOWER_VOLTAGE), () -> stopDrive()).withName("Lower Climb");
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber/Pivot Position", pivotMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Climber/Pivot Velocity", pivotMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber(
        "Climber/Pivot Supply", pivotMotor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber(
        "Climber/Drive Supply", driveMotor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber(
        "Climber/Drive Voltage", driveMotor.getMotorVoltage().getValueAsDouble());
  }
}
