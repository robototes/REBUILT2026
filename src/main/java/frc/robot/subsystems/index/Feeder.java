package frc.robot.subsystems.index;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
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
import frc.robot.util.robotType.RobotType;
import frc.robot.util.tuning.NtTunableBoolean;
import frc.robot.util.tuning.NtTunableDouble;

public class Feeder extends SubsystemBase {
  private final double D_TARGET_RPS = 92;
  private final NtTunableBoolean TUNABLE_ENABLE =
      new NtTunableBoolean("SmartDashboard/Tunables/FeederRPS", false);
  private final NtTunableDouble TARGET_RPS =
      new NtTunableDouble("SmartDashboard/FeederSubsystem/TargetVelocityRPS", D_TARGET_RPS);
  private final VelocityVoltage TARGET_VELOCITY = new VelocityVoltage(D_TARGET_RPS); // Rotations/s

  private final TalonFX feedMotor;
  private final FlywheelSim motorSim;

  public Feeder() {
    feedMotor =
        new TalonFX(
            Hardware.FEEDER_MOTOR_ID,
            (RobotType.isAlpha()) ? CANBus.roboRIO() : CompTunerConstants.kCANBus);
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
    TalonFXConfigurator cfg = feedMotor.getConfigurator();
    TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();

    talonFXConfiguration.MotorOutput.Inverted =
        (RobotType.isAlpha())
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    // Setting the motor to brake when not moving
    talonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // enabling current limits
    talonFXConfiguration.CurrentLimits.StatorCurrentLimit = 60;
    talonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    talonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 30;
    talonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;

    talonFXConfiguration.Slot0.kV = 11.28 / 92;

    cfg.apply(talonFXConfiguration);
  }

  public void runVelocity() {
    if (TUNABLE_ENABLE.get()) {
      feedMotor.setControl(TARGET_VELOCITY.withVelocity(TARGET_RPS.get()));
    } else {
      feedMotor.setControl(TARGET_VELOCITY.withVelocity(D_TARGET_RPS));
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
}
