package frc.robot.subsystems.index;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;
import frc.robot.generated.CompTunerConstants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Feeder extends SubsystemBase {
  private final double D_TARGET_RPS = 95;
  private final LoggedNetworkBoolean TUNABLE_ENABLE =
      new LoggedNetworkBoolean("Tuning/FeederRPS", false);
  private final LoggedNetworkNumber TARGET_RPS =
      new LoggedNetworkNumber("Tuning/FeederSubsystem/TargetVelocityRPS", D_TARGET_RPS);
  private final VelocityTorqueCurrentFOC velocityRequest =
      new VelocityTorqueCurrentFOC(D_TARGET_RPS); // Rotations/s

  private final TalonFX feedMotor;
  private final FlywheelSim motorSim;

  private final FeederIO io;
  private final FeederIOInputsAutoLogged inputs;

  public Feeder() {
    feedMotor = new TalonFX(Hardware.FEEDER_MOTOR_ID, CompTunerConstants.kCANBus);
    feederConfig();
    feedMotor.clearStickyFaults();

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
    io = new FeederIOTalonFX(feedMotor);
    inputs = new FeederIOInputsAutoLogged();
  }

  public void feederConfig() {
    TalonFXConfigurator cfg = feedMotor.getConfigurator();
    TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();

    talonFXConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Setting the motor to brake when not moving
    talonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // enabling current limits
    talonFXConfiguration.CurrentLimits.StatorCurrentLimit = 120;
    talonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;

    talonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 80;
    talonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    talonFXConfiguration.CurrentLimits.SupplyCurrentLowerLimit = 0;

    talonFXConfiguration.Slot0.kS = 4.2;
    talonFXConfiguration.Slot0.kA = 0.5;
    talonFXConfiguration.Slot0.kP = 10;

    cfg.apply(talonFXConfiguration);
  }

  public void runVelocity() {
    if (TUNABLE_ENABLE.get()) {
      feedMotor.setControl(velocityRequest.withVelocity(TARGET_RPS.get()));
    } else {
      feedMotor.setControl(velocityRequest.withVelocity(D_TARGET_RPS));
    }
  }

  public void setVelocity(double rps) {
    if (TUNABLE_ENABLE.get()) {
      feedMotor.setControl(velocityRequest.withVelocity(TARGET_RPS.get()));
    } else {
      feedMotor.setControl(velocityRequest.withVelocity(rps));
    }
  }

  public void stopMotor() {
    feedMotor.stopMotor();
  }

  @Override
  public void simulationPeriodic() {
    motorSim.setInput(feedMotor.getSimState().getMotorVoltage());
    motorSim.update(TimedRobot.kDefaultPeriod);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Feeder/Feeder", inputs);
  }
}
