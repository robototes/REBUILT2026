package frc.robot.subsystems.Launcher;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Hardware;
import frc.robot.Robot;
import frc.robot.util.NtTunableDouble;
import java.util.function.DoubleSupplier;

public class Flywheels extends SubsystemBase {
  private final TalonFX FlywheelOne; // left
  private final TalonFX FlywheelTwo; // right
  private final DoubleTopic currentTopic; // supply current in amps
  private final DoubleTopic velocityTopic; // velocity in rps
  private final DoublePublisher currentPub;
  private final DoublePublisher velocityPub;

  private final MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(0);
  private final Follower follow =
      new Follower(Hardware.FLYWHEEL_ONE_ID, MotorAlignmentValue.Opposed);

  private final FlywheelSim sim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), 0.6194175216, 1),
          DCMotor.getKrakenX60(1),
          0);

  public NtTunableDouble targetVelocity;

  public final double FLYWHEEL_TOLERANCE = 5; // RPS
  public final boolean TUNER_CONTROLLED =
      false; // boolean to decide if it should be controlled using the NtTunableDouble

  // Constructor
  public Flywheels() {
    FlywheelOne = new TalonFX(Hardware.FLYWHEEL_ONE_ID);
    FlywheelTwo = new TalonFX(Hardware.FLYWHEEL_TWO_ID);

    targetVelocity = new NtTunableDouble("/launcher/targetVelocity", 0.0);
    configureMotors();
    simulationInit();

    var nt = NetworkTableInstance.getDefault();
    velocityTopic = nt.getDoubleTopic("/launcher/velocity");
    currentTopic = nt.getDoubleTopic("/launcher/current");
    velocityPub = velocityTopic.publish();
    currentPub = currentTopic.publish();
    velocityPub.set(0.0);
    currentPub.set(0.0);
  }

  private void configureMotors() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    TalonFXConfigurator flConfigurator = FlywheelOne.getConfigurator();
    // set current limits
    config.CurrentLimits.SupplyCurrentLimit = 20;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 40;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    // create coast mode for motors
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // create PID gains
    config.Slot0.kP = 0.1;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    config.Slot0.kA = 0.0;
    config.Slot0.kV = 11.825 / 99;
    config.Slot0.kS = 0.0;
    config.Slot0.kG = 0.0;

    config.MotionMagic.MotionMagicAcceleration = 40; // RPS^2

    flConfigurator.apply(config);
  }

  public Command setVelocityCommand(double rps) {
    return runOnce(
            () -> {
              request.Velocity = rps;
              FlywheelOne.setControl(request);
              FlywheelTwo.setControl(follow);
            })
        .withName("Set Flywheel Velocity");
  }

  public Command suppliedSetVelocityCommand(DoubleSupplier rps) {
    return runOnce(
            () -> {
              request.Velocity = rps.getAsDouble();
              FlywheelOne.setControl(request);
              FlywheelTwo.setControl(follow);
            })
        .withName("Set Flywheel Velocity");
  }

  public void setVelocityRPS(double rps) {
    request.Velocity = rps;
    FlywheelOne.setControl(request);
    FlywheelTwo.setControl(follow);
  }

  public Command stopCommand() {
    return runOnce(
            () -> {
              FlywheelOne.stopMotor();
              FlywheelTwo.stopMotor();
            })
        .withName("Stop Flywheels");
  }

  public boolean atTargetVelocity(double targetRPS, double toleranceRPS) {
    if (Robot.isSimulation()) {
      return true;
    }
    double velocity = (FlywheelOne.getVelocity().getValueAsDouble());

    boolean atTarget = Math.abs(velocity - targetRPS) <= toleranceRPS;
    return atTarget;
  }

  public Trigger atTargetVelocityTrigger(double targetRPM, double toleranceRPM) {
    return new Trigger(() -> atTargetVelocity(targetRPM, toleranceRPM));
  }

  public void simulationInit() {
    var simState = FlywheelOne.getSimState();
    simState.Orientation = ChassisReference.Clockwise_Positive;
    simState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
  }

  public void simulationPeriodic() {

    // Get the sim states
    var simState = FlywheelOne.getSimState();

    // Set the supply voltage for the sims
    simState.setSupplyVoltage(RobotController.getBatteryVoltage());

    // Get the motor voltage outputs from the Talon FXs
    var voltageSim = simState.getMotorVoltageMeasure();

    // Set the inputs to the flywheel sims
    sim.setInputVoltage(voltageSim.in(Volts));

    // Update the sims
    sim.update(0.02);

    // Update the simulated sensor readings
    simState.setRotorVelocity(sim.getAngularVelocity());
  }

  @Override
  public void periodic() {
    velocityPub.set(FlywheelOne.getVelocity().getValueAsDouble());
    currentPub.set(FlywheelOne.getSupplyCurrent().getValueAsDouble());
  }
}
