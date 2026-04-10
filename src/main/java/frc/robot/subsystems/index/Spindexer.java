package frc.robot.subsystems.index;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;
import frc.robot.util.robotType.RobotType;
import frc.robot.util.tuning.NtTunableBoolean;
import frc.robot.util.tuning.NtTunableDouble;

public class Spindexer extends SubsystemBase {
  private final TalonFX spindexerMotor;

  private final double D_TARGET_RPS = 55;
  private final double D_TARGET_ACCEL = 1000; // Rotations /s /s
  private final NtTunableBoolean TUNABLE_ENABLE =
      new NtTunableBoolean("SmartDashboard/Tunables/TuneSpindexer", false);
  private final NtTunableDouble TARGET_ACCEL =
      new NtTunableDouble("SmartDashboard/SpindexerSubsystem/TargetAccelRPS", D_TARGET_ACCEL);
  private final NtTunableDouble TARGET_RPS =
      new NtTunableDouble("SmartDashboard/SpindexerSubsystem/TargetVelocityRPS", D_TARGET_RPS);
  private final VelocityVoltage TARGET_VELOCITY = new VelocityVoltage(D_TARGET_RPS); // Rotations/s

  private final FlywheelSim motorSim;

  // Logs
  private final StatusSignal<AngularVelocity> spindexerRPS;
  private final DoubleLogEntry statorCurrentLog;
  private final DoubleLogEntry supplyCurrentLog;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;

  public Spindexer() {
    spindexerMotor = new TalonFX(Hardware.SPINDEXER_MOTOR_ID);
    spindexerConfig();
    spindexerMotor.clearStickyFaults();

    if (RobotBase.isSimulation()) {
      LinearSystem spindexerMotorSystem =
          LinearSystemId.createFlywheelSystem(
              DCMotor.getKrakenX60(1), 0.001, ((52 / 12) * (52 / 18)));
      motorSim =
          new FlywheelSim(spindexerMotorSystem, DCMotor.getKrakenX60(1), ((52 / 12) * (52 / 18)));
    } else {
      motorSim = null;
    }

    // Instiate log variables
    DataLog log = DataLogManager.getLog();
    statorCurrentLog = new DoubleLogEntry(log, "/SpindexerLogs/statorCurrent");
    supplyCurrentLog = new DoubleLogEntry(log, "/SpindexerLogs/supplyCurrent");
    statorCurrent = spindexerMotor.getStatorCurrent();
    supplyCurrent = spindexerMotor.getSupplyCurrent();
    spindexerRPS = spindexerMotor.getVelocity();
  }

  public void spindexerConfig() {
    // DigitalInput m_sensor = new DigitalInput(HardwareConstants.digitalInputChannel);

    TalonFXConfigurator cfg = spindexerMotor.getConfigurator();
    TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();

    // Inverting motor output direction
    talonFXConfiguration.MotorOutput.Inverted =
        (RobotType.isAlpha())
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    // Setting the motor to brake when not moving
    talonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // enabling current limits
    talonFXConfiguration.CurrentLimits.StatorCurrentLimit = 50;
    talonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    talonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 30;
    talonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    talonFXConfiguration.CurrentLimits.SupplyCurrentLowerLimit = 0;

    talonFXConfiguration.Slot0.kV = 11.2 / 90.7;
    talonFXConfiguration.Slot0.kP = 0.6;

    cfg.apply(talonFXConfiguration);
  }

  public void runVelocity() {
    if (TUNABLE_ENABLE.get()) {
      spindexerMotor.setControl(
          TARGET_VELOCITY.withVelocity(TARGET_RPS.get()).withAcceleration(TARGET_ACCEL.get()));
    } else {
      spindexerMotor.setControl(
          TARGET_VELOCITY.withVelocity(D_TARGET_RPS).withAcceleration(D_TARGET_ACCEL));
    }
  }

  public void setVelocity(double rps) {
    if (TUNABLE_ENABLE.get()) {
      spindexerMotor.setControl(TARGET_VELOCITY.withVelocity(TARGET_RPS.get()));
    } else {
      spindexerMotor.setControl(TARGET_VELOCITY.withVelocity(rps));
    }
  }

  public void stopMotor() {
    spindexerMotor.stopMotor();
  }

  @Override
  public void simulationPeriodic() {
    motorSim.setInput(spindexerMotor.getSimState().getMotorVoltage());
    motorSim.update(TimedRobot.kDefaultPeriod); // every 20 ms
  }

  @Override
  public void periodic() {
    StatusSignal.refreshAll(statorCurrent, supplyCurrent, spindexerRPS);
    SmartDashboard.putNumber("SpindexerSubsystem/VelocityRPS", spindexerRPS.getValueAsDouble());
    // Log stuff
    statorCurrentLog.append(statorCurrent.getValueAsDouble());
    supplyCurrentLog.append(supplyCurrent.getValueAsDouble());
  }
}
