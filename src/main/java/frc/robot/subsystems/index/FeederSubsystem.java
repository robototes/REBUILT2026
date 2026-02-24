package frc.robot.subsystems.index;

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
import frc.robot.generated.CompTunerConstants;

public class FeederSubsystem extends SubsystemBase {
  private int ballsDetectedNum = 0;
  public static final double feederSpeed = 0.7;
  public static final int feederRumbleThreshold = 67;

  private static final double FEEDMOTOR_KP = 38.5;
  private static final double FEEDMOTOR_KI = 0;
  private static final double FEEDMOTOR_KD = 0;
  private static final double FEEDMOTOR_KS = 0;
  private static final double FEEDMOTOR_KV = 0;
  private static final double FEEDMOTOR_KG = 0;
  
  private static final double FEEDMOTOR_KA = 0;

  private final TalonFX feedMotor;

  private final FlywheelSim motorSim;

  public FeederSubsystem() {
    feedMotor = new TalonFX(Hardware.FEEDER_MOTOR_ID, CompTunerConstants.kCANBus);
    feederConfig();

    if (RobotBase.isSimulation()) {
      LinearSystem feedMotorSystem =
          LinearSystemId.createFlywheelSystem(
              DCMotor.getKrakenX60(1),
              0.001,
              1.0); // TODO: Update to final moment of intertia and gear ratio
      motorSim =
          new FlywheelSim(
              feedMotorSystem, DCMotor.getKrakenX60(1), 1.0); // TODO Update to final gear ratio
    } else {
      motorSim = null;
    }
  }

  public void feederConfig() {
    // DigitalInput m_sensor = new DigitalInput(HardwareConstants.digitalInputChannel);

    TalonFXConfigurator cfg = feedMotor.getConfigurator();
    TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();

    // Inverting motor output direction
    talonFXConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    // Setting the motor to brake when not moving
    talonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // enabling current limits
    talonFXConfiguration.CurrentLimits.StatorCurrentLimit = 60;
    talonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    talonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 30;
    talonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;

    cfg.apply(talonFXConfiguration);
  }

  public void setSpeed(double speed) {
    feedMotor.set(speed);
  }

  public Command startMotor() {
    return runEnd(
            () -> {
              setSpeed(feederSpeed);
            },
            () -> {
              setSpeed(0);
            })
        .withName("Start Feeder Motor");
  }

  public Command stopMotor() {
    return runOnce(
            () -> {
              setSpeed(0);
            })
        .withName("Stop Feeder Motor");
  }

  // PLACEHOLDER FOR SENSOR CHECK
  public Command checkSensor() {
    return runOnce(
            () -> {
              // TODO: add logic for ballNum going up after sensor triggers
            })
        .withName("Check Feeder Sensor");
  }

  public int getBallsDetectedNum() {
    return ballsDetectedNum;
  }

  @Override
  public void simulationPeriodic() {
    motorSim.setInput(feedMotor.getSimState().getMotorVoltage());
    motorSim.update(TimedRobot.kDefaultPeriod);
  }
}
