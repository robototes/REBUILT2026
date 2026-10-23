package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;
import org.littletonrobotics.junction.Logger;

public class Blocker extends SubsystemBase {
  private final TalonFX blockerMotor;
  private final MotionMagicVoltage request = new MotionMagicVoltage(0);

  // Positions
  private double targetPos;
  public static final double DEPLOYED_POS = -0.24;
  public static final double RETRACTED_POS = 0;
  private static final double degreeTolerance = 3;

  // PID variables
  private static final double kP = 10;
  private static final double kI = 0;
  private static final double kD = 0;
  private static final double kG = 0;
  private static final double kS = 0.15;
  private static final double kV = 4.5;
  private static final double kA = 0;

  // Current limits
  private static final int STATOR_CURRENT_LIMIT = 30; // amps
  private static final int SUPPLY_CURRENT_LIMIT = 20; // amps

  // Motion Magic Config
  private static final double CRUISE_VELOCITY = 2;
  private static final double ACCELERATION = 6;
  private static final double JERK = 0;

  // Gear Ratio
  private static final double GEAR_RATIO = 45;

  // Soft Limits
  public static final double BLOCK_MIN = -0.25; // rotations
  public static final double BLOCK_MAX = 0.0;

  // Status Signals
  private final StatusSignal<Current> SS_Current;
  private final StatusSignal<Angle> SS_position;

  public Blocker() {
    blockerMotor = new TalonFX(Hardware.BLOCKER_MOTOR_ID, CANBus.roboRIO());
    blockerConfig();
    blockerMotor.clearStickyFaults();
    SS_Current = blockerMotor.getStatorCurrent();
    SS_position = blockerMotor.getPosition();
    Logger.recordOutput("Zero/blockerZero", false);
  }

  public void blockerConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

    config.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = BLOCK_MAX;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = BLOCK_MIN;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    config.MotionMagic.MotionMagicCruiseVelocity = CRUISE_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = ACCELERATION;
    config.MotionMagic.MotionMagicJerk = JERK;

    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    config.Slot0.kG = kG;
    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kA = kA;

    blockerMotor.getConfigurator().apply(config);
  }

  public void setPivotPosition(double pos) {
    targetPos = pos;
    blockerMotor.setControl(request.withPosition(pos));
  }

  public Command deployedCommand() {
    return Commands.startEnd(
        () -> {
          setPivotPosition(DEPLOYED_POS);
          // anything else that should happen on start
        },
        () -> {
          setPivotPosition(RETRACTED_POS);
          // anything else that should happen on end
        });
  }

  public Command zeroBlocker() {
    return runOnce(
            () -> {
              blockerMotor.setPosition(RETRACTED_POS);
              targetPos = RETRACTED_POS;
              Logger.recordOutput("Zero/blockerZero", true);
            })
        .withName("Zero Blocker");
  }

  public double getBlockerPosition() {
    return SS_position.getValueAsDouble();
  }

  public double getBlockerCurrent() {
    return SS_Current.getValueAsDouble();
  }

  public double getBlockerTargetPosition() {
    return targetPos;
  }

  @Override
  // update simulation
  public void periodic() {
    StatusSignal.refreshAll(SS_position);
    StatusSignal.refreshAll(SS_Current);
    Logger.recordOutput("Blocker/blockerCurrent", getBlockerCurrent());
    Logger.recordOutput("Blocker/blockerCurrentPosition", getBlockerPosition());
    Logger.recordOutput("Blocker/blockerTargetPosition", getBlockerTargetPosition());
  }

  public boolean isAtTarget(double pose) {
    return Math.abs(blockerMotor.getPosition().getValueAsDouble() - pose)
        < Units.degreesToRotations(degreeTolerance);
  }
}
