package frc.robot.subsystems.index;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VoltageOut;
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

  private static final double SPINDEXER_VOLTAGE = 12;
  private VoltageOut voltReq = new VoltageOut(0);

  private final TalonFX spindexerMotor;

  private final FlywheelSim motorSim;

  public SpindexerSubsystem() {
    spindexerMotor = new TalonFX(Hardware.SPINDEXER_MOTOR_ID);
    spindexerConfig();

    if (RobotBase.isSimulation()) {
      LinearSystem spindexerMotorSystem =
          LinearSystemId.createFlywheelSystem(
              DCMotor.getKrakenX60(1),
              0.001,
              ((52 / 12) * (52 / 18)));
      motorSim =
          new FlywheelSim(
              spindexerMotorSystem,
              DCMotor.getKrakenX60(1),
              ((52 / 12) * (52 / 18)));
    } else {
      motorSim = null;
    }
  }

  public void spindexerConfig() {
    // DigitalInput m_sensor = new DigitalInput(HardwareConstants.digitalInputChannel);

    TalonFXConfigurator cfg = spindexerMotor.getConfigurator();
    TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();

    // Inverting motor output direction
    talonFXConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    // Setting the motor to brake when not moving
    talonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // enabling current limits
    talonFXConfiguration.CurrentLimits.StatorCurrentLimit = 40;
    talonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    talonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 20;
    talonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;

    cfg.apply(talonFXConfiguration);
  }

  public void setVoltage(double voltage) {
    spindexerMotor.setControl(voltReq.withOutput(voltage));
  }

  public Command startMotor() {
    return runEnd(
            () -> {
              setVoltage(SPINDEXER_VOLTAGE);
            },
            () -> spindexerMotor.stopMotor())
        .withName("Start Spindexer Motor");
  }

  public Command stopMotorCommand() {
    return runOnce(() -> spindexerMotor.stopMotor());
  }

  @Override
  public void simulationPeriodic() {
    motorSim.setInput(spindexerMotor.getSimState().getMotorVoltage());
    motorSim.update(TimedRobot.kDefaultPeriod); // every 20 ms
  }
}
