package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;
import frc.robot.generated.AlphaTunerConstants;
import frc.robot.util.robotType.RobotType;
import frc.robot.util.tuning.NtTunableBoolean;
import frc.robot.util.tuning.NtTunableDouble;

public class IntakeRollers extends SubsystemBase {
  // motors
  private final TalonFX leftRoller;
  private final TalonFX rightRoller;

  // networktables and sim
  private DoubleTopic leftRollerTopic;
  private DoubleTopic rightRollerTopic;
  private DoublePublisher leftRollerPub;
  private DoublePublisher rightRollerPub;
  private RollerSim rollerSim;

  public final double TARGET_RPS = 68;
  public final double AGITATE_RPS = TARGET_RPS / 2;
  private final NtTunableBoolean TUNABLE_ENABLE =
      new NtTunableBoolean("SmartDashboard/Tunables/TuneIntakeRollers", false);
  private final NtTunableDouble NT_TARGET_RPS =
      new NtTunableDouble("SmartDashboard/intake/TargetVelocityRPS", TARGET_RPS);
  private final VelocityTorqueCurrentFOC velocityRequest = new VelocityTorqueCurrentFOC(0);

  // status signals
  private final StatusSignal<AngularVelocity> SS_roller1;
  private final StatusSignal<AngularVelocity> SS_roller2;

  public IntakeRollers() {
    // define motors and configs
    leftRoller =
        new TalonFX(
            Hardware.INTAKE_MOTOR_ONE_ID,
            (RobotType.isAlpha() ? AlphaTunerConstants.kCANBus : CANBus.roboRIO()));
    rightRoller = new TalonFX(Hardware.INTAKE_MOTOR_TWO_ID);
    motorConfigs();
    leftRoller.clearStickyFaults();
    rightRoller.clearStickyFaults();
    networktables();

    // sim creator
    if (RobotBase.isSimulation()) {
      rollerSim = new RollerSim(leftRoller, rightRoller);
    }

    SS_roller1 = leftRoller.getVelocity();
    SS_roller2 = rightRoller.getVelocity();
  }

  // roller configs
  private void motorConfigs() {
    var talonFXConfigs = new TalonFXConfiguration();
    talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast; // KEEP TS AT COAST
    talonFXConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    talonFXConfigs.CurrentLimits.StatorCurrentLimit = 80;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimit = 40;
    talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

    talonFXConfigs.Slot0.kP = RobotType.isAlpha() ? 5.0 : 4.0;
    talonFXConfigs.Slot0.kS = RobotType.isAlpha() ? 5.0 : 1.0;
    talonFXConfigs.Slot0.kA = 0.2;

    // configurator
    leftRoller.getConfigurator().apply(talonFXConfigs);
    talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rightRoller.getConfigurator().apply(talonFXConfigs);
  }

  // configure networktables
  private void networktables() {
    var nt = NetworkTableInstance.getDefault();
    this.leftRollerTopic = nt.getDoubleTopic("intake/leftRollerSpeed");
    this.leftRollerPub = leftRollerTopic.publish();

    this.rightRollerTopic = nt.getDoubleTopic("intake/rightRollerSpeed");
    this.rightRollerPub = rightRollerTopic.publish();

    // default values
    leftRollerPub.set(0);
    rightRollerPub.set(0);
  }

  public void runRollers(double velocity) {
    if (TUNABLE_ENABLE.get() && velocity == TARGET_RPS) {
      leftRoller.setControl(velocityRequest.withVelocity(NT_TARGET_RPS.get()));
      rightRoller.setControl(velocityRequest.withVelocity(NT_TARGET_RPS.get()));
    } else {
      leftRoller.setControl(velocityRequest.withVelocity(velocity));
      rightRoller.setControl(velocityRequest.withVelocity(velocity));
    }
  }

  public void stopMotor() {
    leftRoller.stopMotor();
    rightRoller.stopMotor();
  }

  @Override
  // update networktables
  public void periodic() {
    StatusSignal.refreshAll(SS_roller1, SS_roller2);
    leftRollerPub.set(SS_roller1.getValueAsDouble());
    rightRollerPub.set(SS_roller2.getValueAsDouble());
  }

  // update sim
  public void simulationPeriodic() {
    if (rollerSim != null) {
      rollerSim.updateRollers();
    }
  }
}
