package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDouble;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Hardware;
import frc.robot.util.tuning.NtTunableBoolean;
import frc.robot.util.tuning.NtTunableDouble;
import java.util.function.DoubleSupplier;

public class Flywheels extends SubsystemBase {
  private final TalonFX FlywheelOne; // left
  private final TalonFX FlywheelTwo; // right
  private final DoubleTopic currentTopic; // supply current in amps
  private final DoubleTopic velocityTopic; // velocity in rps
  private final DoublePublisher currentPub;
  private final DoublePublisher velocityPub;

  // Debounce stuff
  private final double DURATION = 1; // second
  private final Debouncer m_dippedDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kFalling);
  private final Debouncer m_recoveredDebouncer =
      new Debouncer(DURATION, Debouncer.DebounceType.kRising);
  private boolean hasDipped = false;

  // Config apply
  private final int MAX_APPLY_CONFIG_ATTEMPTS = 5;
  private final double MAX_APPLY_CONFIG_TIMEOUT = 0.01; // Default is 10 ms

  private FlywheelsSim flywheelSim;
  private VelocityVoltage request = new VelocityVoltage(0);
  private final Follower follow =
      new Follower(Hardware.FLYWHEEL_TWO_ID, MotorAlignmentValue.Opposed);

  public NtTunableDouble targetVelocity;
  private long lastPositionUpdateTime = 0;

  public final double FLYWHEEL_TOLERANCE =
      15; // RPS // increased on drive practice 3/18 from 5 -> 10 //Increased to 15 by TD 3/18
  public final NtTunableBoolean TUNER_CONTROLLED =
      new NtTunableBoolean("/SmartDashboard/Tunables/Flywheels", false);

  // Status signals
  private StatusSignal<AngularVelocity> flywheelOneRPS;
  private StatusSignal<Current> flywheelOneSupplyCurrent;

  // Constructor
  public Flywheels() {
    FlywheelOne = new TalonFX(Hardware.FLYWHEEL_ONE_ID);
    FlywheelTwo = new TalonFX(Hardware.FLYWHEEL_TWO_ID);

    targetVelocity = new NtTunableDouble("/launcher/flywheelTuner", 0.0);
    configureMotors();

    var nt = NetworkTableInstance.getDefault();
    velocityTopic = nt.getDoubleTopic("/launcher/velocity");
    currentTopic = nt.getDoubleTopic("/launcher/current");
    velocityPub = velocityTopic.publish();
    currentPub = currentTopic.publish();
    velocityPub.set(0.0);
    currentPub.set(0.0);

    if (RobotBase.isSimulation()) {
      flywheelSim = new FlywheelsSim(FlywheelOne, FlywheelTwo);
    }

    flywheelOneRPS = FlywheelOne.getVelocity();
    flywheelOneSupplyCurrent = FlywheelOne.getSupplyCurrent();
  }

  private void configureMotors() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    TalonFXConfigurator flConfigurator = FlywheelOne.getConfigurator();
    TalonFXConfigurator frConfigurator = FlywheelTwo.getConfigurator();
    // set current limits
    config.CurrentLimits.SupplyCurrentLimit = 80;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = 0;
    config.CurrentLimits.StatorCurrentLimit = 100;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    // create coast mode for motors
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // create PID gains
    config.Slot0.kP = 0.1;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    config.Slot0.kA = 0.0;
    config.Slot0.kV = 6.0 / 45.0;
    config.Slot0.kS = 0.3;
    config.Slot0.kG = 0.0;

    // create PID gains slot 1
    config.Slot1.kP = 1.0;
    config.Slot1.kI = 0.0;
    config.Slot1.kD = 0.0;
    config.Slot1.kA = 0.0;
    config.Slot1.kV = 6.0 / 45.0;
    config.Slot1.kS = 0.3;
    config.Slot1.kG = 0.0;

    applyConfig(FlywheelOne, config);
    applyConfig(FlywheelTwo, config);
  }

  private void applyConfig(TalonFX motor, TalonFXConfiguration config) {
    int ID = motor.getDeviceID();
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < MAX_APPLY_CONFIG_ATTEMPTS; i++) {
      status = motor.getConfigurator().apply(config, MAX_APPLY_CONFIG_TIMEOUT);
      if (status.isOK()) {
        DataLogManager.log("Successfully applied configuation to motor ID " + ID);
        break; // Success, exit the loop
      }
    }
    if (!status.isOK()) {
      DriverStation.reportError(
          "CRITICAL: Failed to configure Talon ID "
              + ID
              + " after "
              + MAX_APPLY_CONFIG_ATTEMPTS
              + "attempts: "
              + status.getDescription(),
          true);
    }
  }

  public void switchSlot(boolean isLaunching) {
    if (isLaunching) {
      request = request.withSlot(1);
    } else {
      request = request.withSlot(0);
    }
  }

  public Command setVelocityCommand(double rps) {
    return runEnd(
            () -> {
              request.Velocity = rps;
              FlywheelTwo.setControl(request);
              FlywheelOne.setControl(follow);
            },
            () -> {
              FlywheelOne.stopMotor();
              FlywheelTwo.stopMotor();
            })
        .withName("Set Flywheel Velocity");
  }

  public Command suppliedSetVelocityCommand(DoubleSupplier rps) {
    return runEnd(
            () -> {
              request.Velocity = rps.getAsDouble();
              FlywheelTwo.setControl(request);
              FlywheelOne.setControl(follow);
            },
            () -> {
              FlywheelOne.stopMotor();
              FlywheelTwo.stopMotor();
            })
        .withName("Set Flywheel Supplied Velocity");
  }

  public void setVelocityRPS(double rps) {
    request.Velocity = rps;
    FlywheelTwo.setControl(request);
    FlywheelOne.setControl(follow);
  }

  public Command stopCommand() {
    return runOnce(
            () -> {
              FlywheelOne.stopMotor();
              FlywheelTwo.stopMotor();
            })
        .withName("Stop Flywheels");
  }

  public void stopVoid() {
    FlywheelOne.stopMotor();
    FlywheelTwo.stopMotor();
  }

  public boolean atTargetVelocity(double targetRPS, double toleranceRPS) {
    double velocity = (flywheelOneRPS.getValueAsDouble());
    boolean atTarget = Math.abs(velocity - targetRPS) <= toleranceRPS;
    return atTarget;
  }

  public Trigger atTargetVelocityTrigger(double targetRPS, double toleranceRPS) {
    return new Trigger(() -> atTargetVelocity(targetRPS, toleranceRPS));
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
  public void simulationPeriodic() {
    if (flywheelSim != null) {
      flywheelSim.update();
    }
  }

  @Override
  public void periodic() {
    StatusSignal.refreshAll(flywheelOneRPS, flywheelOneSupplyCurrent);
    velocityPub.set(flywheelOneRPS.getValueAsDouble());
    currentPub.set(flywheelOneSupplyCurrent.getValueAsDouble());
    if (TUNER_CONTROLLED.get()) {
      if (targetVelocity.hasChangedSince(lastPositionUpdateTime)) {
        TimestampedDouble currentTarget = targetVelocity.getAtomic();
        setVelocityRPS(currentTarget.value);
        lastPositionUpdateTime = currentTarget.timestamp;
      }
    }
  }
}
