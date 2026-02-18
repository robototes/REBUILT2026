package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import frc.robot.util.AllianceUtils;
import frc.robot.util.LauncherConstants;
import java.util.function.Supplier;

public class IntakePivot extends SubsystemBase {
  private final TalonFX pivotMotor;
  private final MotionMagicVoltage request = new MotionMagicVoltage(0);

  // Positions
  private double targetPos;
  public static final double DEPLOYED_POS = 0.5;
  public static final double RETRACTED_POS = 0.0;

  // PID variables
  private static final double kP = 2.97;
  private static final double kI = 0;
  private static final double kD = 1;
  private static final double kG = 0;
  private static final double kS = 0.41;
  private static final double kV = 0.9;
  private static final double kA = 0.12;

  // Current limits
  private static final int STATOR_CURRENT_LIMIT = 60; // amps
  private static final int SUPPLY_CURRENT_LIMIT = 30; // amps

  // Motion Magic Config
  private static final double CRUISE_VELOCITY = 25;
  private static final double ACCELERATION = 10;
  private static final double JERK = 50;

  // Gear Ratio
  private static final double GEAR_RATIO = 36;

  // Soft Limits
  private static final double PIVOT_MAX = 180; // degrees
  private static final double PIVOT_MIN = 0; // degrees


  public IntakePivot() {
    pivotMotor = new TalonFX(Hardware.INTAKE_PIVOT_MOTOR_ID);
    turretConfig();
    pivotMotor.setPosition(0);
  }

  public void turretConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

    config.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.degreesToRotations(PIVOT_MAX);
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.degreesToRotations(PIVOT_MIN);
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

    pivotMotor.getConfigurator().apply(config);
  }

  public Command setPivotPosition(double pos) {
    return runOnce(
        () -> {
          pivotMotor.setControl(request.withPosition(pos));
          targetPos = pos;
        });
  }

  public Command zeroPivot() {
    return runOnce(
        () -> {
          pivotMotor.setPosition(0);
          targetPos = 0;
        });
  }

  public Command manualMovingVoltage(Supplier<Voltage> speed) {
    return runEnd(
        () -> pivotMotor.setVoltage(speed.get().in(Volts)), () -> pivotMotor.stopMotor());
  }


  public double getTurretPosition() {
    return pivotMotor.getPosition().getValueAsDouble();
  }

  public boolean isDeployed(double degreeTolerance) {
    return Math.abs(pivotMotor.getPosition().getValueAsDouble() - targetPos)
        < Units.degreesToRotations(degreeTolerance);
  }


}
