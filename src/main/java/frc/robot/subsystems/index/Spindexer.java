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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;
import frc.robot.util.robotType.RobotType;
import frc.robot.util.tuning.NtTunableBoolean;
import frc.robot.util.tuning.NtTunableDouble;

public class Spindexer extends SubsystemBase {
  private final TalonFX spindexerMotor;

  private final double D_TARGET_RPS = 5;
  private final double D_TARGET_ACCEL = 1000; // Rotations /s /s
  private final NtTunableBoolean TUNABLE_ENABLE =
      new NtTunableBoolean("SmartDashboard/Tunables/TuneSpindexer", false);
  private final NtTunableDouble TARGET_ACCEL =
      new NtTunableDouble("SmartDashboard/SpindexerSubsystem/TargetAccelRPS", D_TARGET_ACCEL);
  private final NtTunableDouble TARGET_RPS =
      new NtTunableDouble("SmartDashboard/SpindexerSubsystem/TargetVelocityRPS", D_TARGET_RPS);
  private final VelocityVoltage TARGET_VELOCITY = new VelocityVoltage(D_TARGET_RPS); // Rotations/s

  private final FlywheelSim motorSim;

  // Oscillate settings
  private static final double AMPLITUDE = 0.5;
  private static final double VERTICAL_SHIFT = 0.5;
  private static final double FREQUENCY = Math.PI * 2;
  private static final double MAX_RPS_OFFSET = 1.5;
  // Logs
  private final StatusSignal<AngularVelocity> spindexerRPS;
  private final DoubleLogEntry statorCurrentLog;
  private final DoubleLogEntry supplyCurrentLog;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;

  public Spindexer() {
    spindexerMotor = new TalonFX(Hardware.SPINDEXER_MOTOR_ID);
    spindexerConfig();

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
    double baseRps = TUNABLE_ENABLE.get() ? TARGET_RPS.get() : D_TARGET_RPS;
    double accel = TUNABLE_ENABLE.get() ? TARGET_ACCEL.get() : D_TARGET_ACCEL;
    double velocity = baseRps - MAX_RPS_OFFSET * oscillate(Timer.getFPGATimestamp());

    spindexerMotor.setControl(TARGET_VELOCITY.withVelocity(velocity).withAcceleration(accel));
  }

  public void stopMotor() {
    spindexerMotor.stopMotor();
  }

  private static double oscillate(double time) {
    return AMPLITUDE * Math.cos(FREQUENCY * time) + VERTICAL_SHIFT;
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
