package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;

public class Blocker extends SubsystemBase {
  private final TalonFX blockerMotor;
  private final MotionMagicVoltage request = new MotionMagicVoltage(0);

  // Positions
  private double targetPos;
  public static final double DEPLOYED_POS = 0;
  public static final double RETRACTED_POS = 0;
  private static final double degreeTolerance = 5;

  // PID variables
  private static final double kP = 40;
  private static final double kI = 0;
  private static final double kD = 1;
  private static final double kG = 0.4;
  private static final double kS = 0.2;
  private static final double kV = 0;
  private static final double kA = 0;

  // Current limits
  private static final int STATOR_CURRENT_LIMIT = 60; // amps
  private static final int SUPPLY_CURRENT_LIMIT = 30; // amps

  // Motion Magic Config
  private static final double CRUISE_VELOCITY = 100;
  private static final double ACCELERATION = 400;
  private static final double JERK = 0;

  // Gear Ratio
  private static final double GEAR_RATIO = 35;

  // Soft Limits
  public static final double PIVOT_MIN = -0.45; // rotations
  public static final double PIVOT_MAX = 0.0;

  public Blocker() {
    blockerMotor = new TalonFX(Hardware.BLOCKER_MOTOR_ID, CANBus.roboRIO());
    blockerConfig();
    blockerMotor.clearStickyFaults();
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

    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = PIVOT_MAX;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = PIVOT_MIN;
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

  public Command blockerInCommand() {
    return Commands.runOnce(() -> setPivotPosition(RETRACTED_POS));
  }

  public Command blockerOutCommand() {
    return Commands.runOnce(() -> setPivotPosition(DEPLOYED_POS));
  }

  public Command zeroBlocker() {
    return runOnce(
            () -> {
              blockerMotor.setPosition(RETRACTED_POS);
              targetPos = RETRACTED_POS;
            })
        .withName("Zero Blocker");
  }

  public boolean isAtTarget(double pose) {
    return Math.abs(blockerMotor.getPosition().getValueAsDouble() - pose)
        < Units.degreesToRotations(degreeTolerance);
  }
}
