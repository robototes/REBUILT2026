package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;

public class SpindexerSubsystem extends SubsystemBase {

  public static final double serializerSpeed = 1.0;

  private final TalonFX serialMotor;

  private final FlywheelSim motorSim;

  public SpindexerSubsystem() {
    serialMotor = new TalonFX(Hardware.SPINDEXER_MOTOR_ID);
    spindexerConfig();

    if (RobotBase.isSimulation()) {
      LinearSystem serialMotorSystem =
          LinearSystemId.createFlywheelSystem(
              DCMotor.getKrakenX60(1),
              0.001,
              1.0); // TODO: Update to final moment of inertia and gear ratio
      motorSim =
          new FlywheelSim(
              serialMotorSystem, DCMotor.getKrakenX60(1), 1.0); // TODO: Update to final gear ratio
    } else {
      motorSim = null;
    }
  }

  public void spindexerConfig() {
    // DigitalInput m_sensor = new DigitalInput(HardwareConstants.digitalInputChannel);

    TalonFXConfigurator cfg = serialMotor.getConfigurator();
    TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();

    // Inverting motor output direction
    talonFXConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    // Setting the motor to brake when not moving
    talonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // enabling current limits
    talonFXConfiguration.CurrentLimits.StatorCurrentLimit = 40;
    talonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    talonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 20;
    talonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;

    cfg.apply(talonFXConfiguration);
  }

  public void setSpeed(double speed) {
    serialMotor.set(speed);
  }

  public Command startMotor() {
    return runOnce(
            () -> {
              setSpeed(serializerSpeed);
            })
        .withName("Start Spindexer Motor");
  }

  public Command stopMotor() {
    return runOnce(
            () -> {
              setSpeed(0);
            })
        .withName("Stop Spindexer Motor");
  }

  @Override
  public void simulationPeriodic() {
    motorSim.setInput(serialMotor.getSimState().getMotorVoltage());
    motorSim.update(TimedRobot.kDefaultPeriod); // every 20 ms
  }
}
