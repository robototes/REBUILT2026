package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.networktables.TimestampedDouble;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Hardware;
import frc.robot.util.robotType.RobotType;
import frc.robot.util.tuning.NtTunableDouble;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class Flywheels extends SubsystemBase {
  private final TalonFX flywheelOne; // left spins clockwise
  private final TalonFX flywheelTwo; // right spins counterclockwise

  // Debounce stuff
  private static final double DURATION = 1; // second
  private final Debouncer m_dippedDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kFalling);
  private final Debouncer m_recoveredDebouncer =
      new Debouncer(DURATION, Debouncer.DebounceType.kRising);
  private boolean hasDipped = false;

  // Config apply
  private static final int MAX_APPLY_CONFIG_ATTEMPTS = 5;
  private static final double MAX_APPLY_CONFIG_TIMEOUT = 0.1; // Default is 100 ms

  private VelocityTorqueCurrentFOC request = new VelocityTorqueCurrentFOC(0);

  public NtTunableDouble targetVelocity;
  private long lastPositionUpdateTime = 0;

  public final double FLYWHEEL_TOLERANCE = 10;
  public final LoggedNetworkBoolean TUNER_CONTROLLED =
      new LoggedNetworkBoolean("Tuning/Flywheels", false);

  // Status signals
  private StatusSignal<AngularVelocity> flywheelOneRPS;
  private StatusSignal<Current> flywheelOneStatorCurrent;
  private StatusSignal<Current> flywheelOneSupplyCurrent;
  private StatusSignal<AngularVelocity> flywheelTwoRPS;
  private StatusSignal<Current> flywheelTwoStatorCurrent;
  private StatusSignal<Current> flywheelTwoSupplyCurrent;

  // Constructor
  public Flywheels() {
    flywheelOne = new TalonFX(Hardware.FLYWHEEL_ONE_ID);
    flywheelTwo = new TalonFX(Hardware.FLYWHEEL_TWO_ID);

    targetVelocity = new NtTunableDouble("Tuning/launcher/flywheelTuner", 0.0);
    configureMotors();

    flywheelOneRPS = flywheelOne.getVelocity();
    flywheelOneStatorCurrent = flywheelOne.getStatorCurrent();
    flywheelOneSupplyCurrent = flywheelOne.getSupplyCurrent();
    flywheelTwoRPS = flywheelTwo.getVelocity();
    flywheelTwoStatorCurrent = flywheelTwo.getStatorCurrent();
    flywheelTwoSupplyCurrent = flywheelTwo.getSupplyCurrent();

    flywheelOne.clearStickyFaults();
    flywheelTwo.clearStickyFaults();
  }

  private void configureMotors() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    // set current limits
    config.CurrentLimits.SupplyCurrentLimit = 80;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = 0;
    config.CurrentLimits.StatorCurrentLimit = 80;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    // create coast mode for motors
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // create PID gains
    config.Slot0.kP = RobotType.isAlpha() ? 5 : 10;
    config.Slot0.kS = 5.0;
    config.Slot0.kA = 0.5;

    config.MotorOutput.Inverted =
        RobotType.isAlpha()
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    applyConfig(flywheelOne, config);

    config.MotorOutput.Inverted =
        RobotType.isAlpha()
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive;
    applyConfig(flywheelTwo, config);
  }

  private void applyConfig(TalonFX motor, TalonFXConfiguration config) {
    int id = motor.getDeviceID();
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < MAX_APPLY_CONFIG_ATTEMPTS; i++) {
      status = motor.getConfigurator().apply(config, MAX_APPLY_CONFIG_TIMEOUT);
      if (status.isOK()) {
        DataLogManager.log("Successfully applied configuration to motor ID " + id);
        break; // Success, exit the loop
      }
    }
    if (!status.isOK()) {
      DriverStation.reportError(
          "CRITICAL: Failed to configure Talon ID "
              + id
              + " after "
              + MAX_APPLY_CONFIG_ATTEMPTS
              + " attempts: "
              + status.getDescription(),
          true);
    }
  }

  public Command setVelocityCommand(double rps) {
    return runEnd(
            () -> {
              setVelocityRPS(rps);
            },
            () -> {
              flywheelOne.stopMotor();
              flywheelTwo.stopMotor();
            })
        .withName("Set Flywheel Velocity");
  }

  public void setVelocityRPS(double rps) {
    request.Velocity = rps;
    flywheelOne.setControl(request);
    flywheelTwo.setControl(request);
  }

  public Command stopCommand() {
    return runOnce(
            () -> {
              flywheelOne.stopMotor();
              flywheelTwo.stopMotor();
            })
        .withName("Stop Flywheels");
  }

  public void stop() {
    flywheelOne.stopMotor();
    flywheelTwo.stopMotor();
  }

  public boolean atTargetVelocity(double targetRPS, double toleranceRPS) {
    double velocity = (flywheelOneRPS.getValueAsDouble());
    boolean atTarget = Math.abs(velocity - targetRPS) <= toleranceRPS;
    return atTarget;
  }

  public Trigger atTargetVelocityTrigger(double targetRPS, double toleranceRPS) {
    return new Trigger(() -> atTargetVelocity(targetRPS, toleranceRPS));
  }

  public double getTargetSpeed() {
    return request.Velocity;
  }

  public void resetFuelCheck() {
    hasDipped = false;
  }

  public boolean isOutOfFuel() {
    boolean atTarget = atTargetVelocity(request.Velocity, FLYWHEEL_TOLERANCE);

    boolean stillAtTarget = m_dippedDebouncer.calculate(atTarget);
    if (!stillAtTarget) {
      hasDipped = true;
    }

    if (!hasDipped) return false;

    return m_recoveredDebouncer.calculate(atTarget);
  }

  @Override
  public void periodic() {
    StatusSignal.refreshAll(
        flywheelOneRPS,
        flywheelOneSupplyCurrent,
        flywheelOneStatorCurrent,
        flywheelTwoRPS,
        flywheelTwoStatorCurrent,
        flywheelTwoSupplyCurrent);
    Logger.recordOutput("Launcher/flywheel1/velocity", flywheelOneRPS.getValueAsDouble());
    Logger.recordOutput(
        "Launcher/flywheel1/StatorCurrent", flywheelOneStatorCurrent.getValueAsDouble());
    Logger.recordOutput(
        "Launcher/flywheel1/supplyCurrent", flywheelOneSupplyCurrent.getValueAsDouble());
    Logger.recordOutput("Launcher/flywheel2/velocity", flywheelTwoRPS.getValueAsDouble());
    Logger.recordOutput(
        "Launcher/flywheel2/StatorCurrent", flywheelTwoStatorCurrent.getValueAsDouble());
    Logger.recordOutput(
        "Launcher/flywheel2/supplyCurrent", flywheelTwoSupplyCurrent.getValueAsDouble());
    if (TUNER_CONTROLLED.get()) {
      if (targetVelocity.hasChangedSince(lastPositionUpdateTime)) {
        TimestampedDouble currentTarget = targetVelocity.getAtomic();
        setVelocityRPS(currentTarget.value);
        lastPositionUpdateTime = currentTarget.timestamp;
      }
    }
  }
}
