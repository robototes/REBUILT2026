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

public class SerializerSubsystem extends SubsystemBase {
  private static final double SERIALMOTOR_KP = 38.5;
  private static final double SERIALMOTOR_KI = 0;
  private static final double SERIALMOTOR_KD = 0;
  private static final double SERIALMOTOR_KS = 0;
  private static final double SERIALMOTOR_KV = 0;
  private static final double SERIALMOTOR_KG = 0;
  private static final double SERIALMOTOR_KA = 0;

  public static final double serializerSpeed = 1.0;

  private final TalonFX serialMotor;

  private final FlywheelSim motorSim;

  LinearSystem serialMotorSystem = LinearSystemId.createFlywheelSystem(
    DCMotor.getKrakenX60(1),
    0.0,0.0
  );

  public SerializerSubsystem() {
    serialMotor = new TalonFX(Hardware.serializerMotorID);
    feederConfig();

    if (RobotBase.isSimulation()) {
      motorSim = new FlywheelSim(
        serialMotorSystem,
        DCMotor.getKrakenX60(1),
        0.0
      );
    } else {
      motorSim = null;
    }
  }

  public void feederConfig() {
    //DigitalInput m_sensor = new DigitalInput(HardwareConstants.digitalInputChannel);

    TalonFXConfigurator cfg = serialMotor.getConfigurator();
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
    talonFXConfiguration.Slot0.kS = SERIALMOTOR_KS;
    talonFXConfiguration.Slot0.kV = SERIALMOTOR_KV;
    talonFXConfiguration.Slot0.kA = SERIALMOTOR_KA;
    talonFXConfiguration.Slot0.kP = SERIALMOTOR_KP;
    talonFXConfiguration.Slot0.kI = SERIALMOTOR_KI;
    talonFXConfiguration.Slot0.kD = SERIALMOTOR_KD;
    talonFXConfiguration.Slot0.kG = SERIALMOTOR_KG;
    talonFXConfiguration.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    cfg.apply(talonFXConfiguration);
  }

  public void setSpeed(double speed) {
    serialMotor.set(speed);
  }

  public Command startMotor() {
    return runOnce(
        () -> {
          setSpeed(serializerSpeed);
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

  @Override
  public void periodic() {

  }

  @Override
  public void simulationPeriodic() {
    motorSim.setInput(serialMotor.getSimState().getMotorVoltage());
    motorSim.update(0.020); //every 20 ms
    RoboRioSim.setVInVoltage(
      BatterySim.calculateDefaultBatteryLoadedVoltage(
        motorSim.getCurrentDrawAmps()
      )
    );
  }
}
