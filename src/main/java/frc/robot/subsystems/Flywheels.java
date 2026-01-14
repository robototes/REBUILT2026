package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Hardware;

public class Flywheels extends SubsystemBase {
  private final TalonFX Flywheel;

  private final MotionMagicVelocityDutyCycle request = new MotionMagicVelocityDutyCycle(0);

  private final FlywheelSim Sim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), 0.6194175216, 1),
          DCMotor.getKrakenX60(1),
          0);
  // Constructor
  public Flywheels() {
    Flywheel = new TalonFX(Hardware.FLYWHEEL_ID);

    configureMotors();
    simulationInit();
  }

  private void configureMotors() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    TalonFXConfigurator flConfigurator = Flywheel.getConfigurator();
    // set current limits
    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 80;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    // create brake mode for motors
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // create PID gains
    config.Slot0.kP = 100;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    config.Slot0.kA = 0.0;
    config.Slot0.kV = 0.00;
    config.Slot0.kS = 0.0;
    config.Slot0.kG = 0.0;

    config.MotionMagic.MotionMagicAcceleration = 3000; // RPM/s
    config.MotionMagic.MotionMagicJerk = 1500; // RPM/s^2

    flConfigurator.apply(config);
  }

  public Command setVelocityCommand(double rpm) {
    request.Velocity = rpm;
    return runOnce(
            () -> {
              Flywheel.setControl(request);
            })
        .withName("Set Flywheel Velocity");
  }

  public Command stopCommand() {
    return runOnce(
            () -> {
              Flywheel.stopMotor();
            })
        .withName("Stop Flywheels");
  }

  public boolean atTargetVelocity(
      double targetRPM, double toleranceRPM) {
    double velocity =
        (Flywheel.getVelocity().getValueAsDouble());

    boolean atTarget = Math.abs(velocity - targetRPM) <= toleranceRPM;
    return atTarget;
  }

  public Trigger atTargetVelocityTrigger(
      double targetRPM, double toleranceRPM) {
    return new Trigger(() -> atTargetVelocity(targetRPM, toleranceRPM));
  }

  public void simulationInit() {
    var SimState = Flywheel.getSimState();
    SimState.Orientation = ChassisReference.Clockwise_Positive;
    SimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
  }

  public void simulationPeriodic() {

    // Get the sim states
    var simState = Flywheel.getSimState();

    // Set the supply voltage for the sims
    simState.setSupplyVoltage(RobotController.getBatteryVoltage());

    // Get the motor voltage outputs from the Talon FXs
    var voltageSim = simState.getMotorVoltageMeasure();

    // Set the inputs to the flywheel sims
    Sim.setInputVoltage(voltageSim.in(Volts));

    // Update the sims
    Sim.update(0.02);

    // Update the simulated sensor readings
    simState.setRotorVelocity(Sim.getAngularVelocity());
  }
}
