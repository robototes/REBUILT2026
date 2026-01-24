package frc.robot.subsystems;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class FeederSubsystem extends SubsystemBase {
  private int ballNum = 0;
  public static final double feederSpeed = 1.0;
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

  LinearSystem feedMotorSystem = LinearSystemId.createFlywheelSystem(
    DCMotor.getKrakenX60(1),
    0.0,0.0
  );

  public FeederSubsystem() {
    feedMotor = new TalonFX(Hardware.feederMotorID);
    feederConfig();

    if (RobotBase.isSimulation()) {
      motorSim = new FlywheelSim(
        feedMotorSystem,
        DCMotor.getKrakenX60(1),
        0.0
      );
    } else {
      motorSim = null;
    }
  }

  public void feederConfig() {
    //DigitalInput m_sensor = new DigitalInput(HardwareConstants.digitalInputChannel);

    TalonFXConfigurator cfg = feedMotor.getConfigurator();
    TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();

    // Inverting motor output direction
    talonFXConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    // Setting the motor to brake when not moving
    talonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // enabling current limits
    talonFXConfiguration.CurrentLimits.StatorCurrentLimit = 20;
    talonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    talonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 10;
    talonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;

    // PID
    // set slot 0 gains
    talonFXConfiguration.Slot0.kS = FEEDMOTOR_KS;
    talonFXConfiguration.Slot0.kV = FEEDMOTOR_KV;
    talonFXConfiguration.Slot0.kA = FEEDMOTOR_KA;
    talonFXConfiguration.Slot0.kP = FEEDMOTOR_KP;
    talonFXConfiguration.Slot0.kI = FEEDMOTOR_KI;
    talonFXConfiguration.Slot0.kD = FEEDMOTOR_KD;
    talonFXConfiguration.Slot0.kG = FEEDMOTOR_KG;
    talonFXConfiguration.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    cfg.apply(talonFXConfiguration);
  }

  public void setSpeed(double speed) {
    feedMotor.set(speed);
  }

  public Command startMotor() {
    return runOnce(
        () -> {
          setSpeed(feederSpeed);
        });
  }

  public Command stopMotor() {
    return runOnce(
        () -> {
          setSpeed(0);
        });
  }

  //remove in final, only here to stop Autos.java from throwing errors while also doing nothing
  public Command doNothing() {
    return runOnce(
        () -> {
        });
  }

  //PLACEHOLDER FOR SENSOR CHECK
  public Command checkSensor() {
    return runOnce(
        () -> {
          //TODO: add logic for ballNum going up after sensor triggers
        });
  }

  public int getBallNum() {
    return ballNum;
  }

  @Override
  public void periodic() {

  }

  @Override
  public void simulationPeriodic() {
    motorSim.setInput(feedMotor.getSimState().getMotorVoltage());
    motorSim.update(0.020); //every 20 ms
    RoboRioSim.setVInVoltage(
      BatterySim.calculateDefaultBatteryLoadedVoltage(
        motorSim.getCurrentDrawAmps()
      )
    );
  }
}
