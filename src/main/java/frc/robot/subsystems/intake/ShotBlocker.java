package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;

public class ShotBlocker extends SubsystemBase {
  private final TalonFX blockerPivot;

  // Request
  private final MotionMagicVoltage request = new MotionMagicVoltage(0);

  private final double PIVOT_POS = 0.5; // Degrees
  private final double GEAR_RATIO = 5;
  private final double MM_CRUISE_VELOCITY = 5; // Rot/s^2
  private final double MM_ACCELERATION = 300; // Rot/s^2
  private final double MM_JERK = 1000;

  private final DoublePublisher NT_blockerPos;
  private final DoublePublisher NT_blockerCurrent;
  private final DoublePublisher NT_blockerVoltage;

  // Status signals
  private StatusSignal<Current> SS_blockerCurrent;
  private StatusSignal<Voltage> SS_blockerVoltage;
  private StatusSignal<Angle> SS_blockerPos;

  // motor pos
  private double STOWED_POS = 0;
  private double DEPLOYED_POS = 0;

  public ShotBlocker() {
    blockerPivot = new TalonFX(Hardware.SHOT_BLOCKER_ID);
    NetworkTableInstance ntInst = NetworkTableInstance.getDefault();
    NT_blockerPos = ntInst.getDoubleTopic("SmartDashboard/Shot blocker/Position").publish();
    NT_blockerCurrent = ntInst.getDoubleTopic("SmartDashboard/Shot blocker/Current").publish();
    NT_blockerVoltage = ntInst.getDoubleTopic("SmartDashboard/Shot blocker/Voltage").publish();

    NT_blockerPos.set(0);
    NT_blockerCurrent.set(0);
    NT_blockerVoltage.set(0);

    SS_blockerPos = blockerPivot.getPosition();
    SS_blockerVoltage = blockerPivot.getSupplyVoltage();
    SS_blockerCurrent = blockerPivot.getStatorCurrent();
  }

  public void initMotors() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    MotionMagicConfigs MM_config = new MotionMagicConfigs();
    Slot0Configs slot0 = config.Slot0;

    slot0.kP = 1;
    slot0.kI = 0;
    slot0.kD = 0.01;
    slot0.kS = 0.1;
    slot0.kV = 0;
    slot0.kA = 0;
    slot0.kG = 0;

    MM_config.MotionMagicCruiseVelocity = MM_CRUISE_VELOCITY;
    MM_config.MotionMagicAcceleration = MM_ACCELERATION;
    MM_config.MotionMagicJerk = MM_JERK;

    config.MotionMagic = MM_config;
    blockerPivot.getConfigurator().apply(config);
  }

  private void setPos(double pos) {
    blockerPivot.setControl(request.withPosition(pos));
  }

  public Command deploy() {
    return Commands.runOnce(
        () -> {
          setPos(DEPLOYED_POS);
        },
        this);
  }

  public Command stow() {
    return Commands.runOnce(
        () -> {
          setPos(STOWED_POS);
        },
        this);
  }

  @Override
  public void periodic() {
    // Refresh
    StatusSignal.refreshAll(SS_blockerCurrent, SS_blockerPos, SS_blockerVoltage);
    // Set NT
    NT_blockerCurrent.set(SS_blockerCurrent.getValue().in(Amp));
    NT_blockerPos.set(SS_blockerPos.getValue().in(Degrees));
    NT_blockerVoltage.set(SS_blockerVoltage.getValue().in(Volts));
  }
}
