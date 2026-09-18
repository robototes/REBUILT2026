package frc.robot.subsystems.index;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.wpilib.math.system.LinearSystem;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.LinearSystemId;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.util.datalog.DataLog;
import org.wpilib.util.datalog.DoubleLogEntry;
import org.wpilib.system.DataLogManager;
import org.wpilib.framework.RobotBase;
import org.wpilib.framework.TimedRobot;
import org.wpilib.simulation.FlywheelSim;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.SubsystemBase;
import frc.robot.Hardware;
import frc.robot.generated.CompTunerConstants;
import frc.robot.util.robotType.RobotType;
import frc.robot.util.tuning.NtTunableBoolean;
import frc.robot.util.tuning.NtTunableDouble;

public class Feeder extends SubsystemBase {
  private final double D_TARGET_RPS = 95;
  private final NtTunableBoolean TUNABLE_ENABLE =
      new NtTunableBoolean("SmartDashboard/Tunables/FeederRPS", false);
  private final NtTunableDouble TARGET_RPS =
      new NtTunableDouble("SmartDashboard/FeederSubsystem/TargetVelocityRPS", D_TARGET_RPS);
  private final VelocityVoltage TARGET_VELOCITY = new VelocityVoltage(D_TARGET_RPS); // Rotations/s

  private final TalonFX feedMotor;
  private final FlywheelSim motorSim;

  // Status signals and logging
  private final StatusSignal<AngularVelocity> feederRPS;
  private final DoubleLogEntry statorCurrentLog;
  private final DoubleLogEntry supplyCurrentLog;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;

  public Feeder() {
    feedMotor =
        new TalonFX(
            Hardware.FEEDER_MOTOR_ID,
            (RobotType.isAlpha()) ? CANBus.roboRIO() : CompTunerConstants.kCANBus);
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
    feederRPS = feedMotor.getVelocity();
    DataLog log = DataLogManager.getLog();
    statorCurrentLog = new DoubleLogEntry(log, "/FeederLogs/statorCurrent");
    supplyCurrentLog = new DoubleLogEntry(log, "/FeederLogs/supplyCurrent");
    statorCurrent = feedMotor.getStatorCurrent();
    supplyCurrent = feedMotor.getSupplyCurrent();
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

    talonFXConfiguration.Slot0.kV = 11.28 / 92;
    talonFXConfiguration.Slot0.kP = 0.5;

    cfg.apply(talonFXConfiguration);
  }

  public void runVelocity() {
    if (TUNABLE_ENABLE.get()) {
      feedMotor.setControl(TARGET_VELOCITY.withVelocity(TARGET_RPS.get()));
    } else {
      feedMotor.setControl(TARGET_VELOCITY.withVelocity(D_TARGET_RPS));
    }
  }

  public void setVelocity(double rps) {
    if (TUNABLE_ENABLE.get()) {
      feedMotor.setControl(TARGET_VELOCITY.withVelocity(TARGET_RPS.get()));
    } else {
      feedMotor.setControl(TARGET_VELOCITY.withVelocity(rps));
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
    StatusSignal.refreshAll(statorCurrent, supplyCurrent, feederRPS);
    // Log on NT at all times
    SmartDashboard.putNumber("FeederSubsystem/VelocityRPS", feederRPS.getValueAsDouble());
    // Log stuff
    statorCurrentLog.append(statorCurrent.getValueAsDouble());
    supplyCurrentLog.append(supplyCurrent.getValueAsDouble());
  }
}
