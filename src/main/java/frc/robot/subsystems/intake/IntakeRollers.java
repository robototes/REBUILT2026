package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
  private final Follower followRequest =
      new Follower(Hardware.INTAKE_MOTOR_ONE_ID, MotorAlignmentValue.Opposed);
  private final VoltageOut voltReq = new VoltageOut(0).withEnableFOC(false);
  public static final double INTAKE_VOLTAGE = 8;
  public static final double AGITATE_VOLTAGE = 4;

  // networktables and sim
  private DoubleTopic leftRollerTopic;
  private DoubleTopic rightRollerTopic;
  private DoublePublisher leftRollerPub;
  private DoublePublisher rightRollerPub;
  private RollerSim rollerSim;

  private final double D_TARGET_RPS = 5;
  private final double D_TARGET_ACCEL = 10; // Rotations /s /s
  private final double D_AGITATE_TARGET_RPS = 2.5;
  private final NtTunableDouble TARGET_AGITATE =
      new NtTunableDouble("SmartDashboard/intake/TargetAgitateRPS", D_AGITATE_TARGET_RPS);
  private final NtTunableDouble TARGET_ACCEL =
      new NtTunableDouble("SmartDashboard/intake/TargetAccelRPS", D_TARGET_ACCEL);
  private final NtTunableBoolean TUNABLE_ENABLE =
      new NtTunableBoolean("SmartDashboard/Tunables/TuneIntakeRollers", false);
  private final NtTunableDouble TARGET_RPS =
      new NtTunableDouble("SmartDashboard/intake/TargetVelocityRPS", D_TARGET_RPS);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(D_TARGET_RPS); // Rotations/s

  public IntakeRollers() {
    // define motors and configs
    leftRoller =
        new TalonFX(
            Hardware.INTAKE_MOTOR_ONE_ID,
            (RobotType.isAlpha() ? AlphaTunerConstants.kCANBus : CANBus.roboRIO()));
    rightRoller = new TalonFX(Hardware.INTAKE_MOTOR_TWO_ID);
    motorConfigs();
    networktables();

    // sim creator
    if (RobotBase.isSimulation()) {
      rollerSim = new RollerSim(leftRoller, rightRoller);
    }
  }

  // roller configs
  private void motorConfigs() {
    var talonFXConfigs = new TalonFXConfiguration();
    talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast; // KEEP TS AT COAST
    talonFXConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // motor limits idk if i need to add anymore
    talonFXConfigs.CurrentLimits.StatorCurrentLimit = 60;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimit = 30;
    talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

    talonFXConfigs.Slot0.kA = 0.0;
    talonFXConfigs.Slot0.kV = 10.7 / 83;

    // configurator
    leftRoller.getConfigurator().apply(talonFXConfigs);
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

  // Using Voltage
  public Command runRollers(double voltage) {
    return Commands.runEnd(
        () -> {
          leftRoller.setControl(voltReq.withOutput(voltage));
          rightRoller.setControl(followRequest);
        },
        () -> {
          leftRoller.stopMotor();
          rightRoller.stopMotor();
        });
  }

  public void setRollerVolt(double voltage) {
    leftRoller.setControl(voltReq.withOutput(voltage));
    rightRoller.setControl(followRequest);
  }

  public void setReverseRollerVolt(double voltage) {
    leftRoller.setControl(voltReq.withOutput(-voltage));
    rightRoller.setControl(followRequest);
  }

  // Using velocity
  public Command runRollersVelocity(boolean reverse) {
    return Commands.runEnd(
        () -> runRollersVelocityVoid(reverse),
        () -> {
          leftRoller.stopMotor();
          rightRoller.stopMotor();
        },
        this);
  }

  public void runRollersVelocityVoid(boolean reverse) {
    double accel;
    double vel;
    if (TUNABLE_ENABLE.get()) {
      accel = reverse ? -TARGET_ACCEL.get() : TARGET_ACCEL.get();
      vel = reverse ? -TARGET_RPS.get() : TARGET_RPS.get();
    } else {
      accel = reverse ? -D_TARGET_ACCEL : D_TARGET_ACCEL;
      vel = reverse ? -D_TARGET_RPS : D_TARGET_RPS;
    }
    setVelocity(vel, accel);
  }

  public void setVelocity(double velocity, double acceleration) {
    leftRoller.setControl(velocityRequest.withVelocity(velocity).withAcceleration(acceleration));
    rightRoller.setControl(followRequest);
  }

  public void setVelocity(double velocity) {
    leftRoller.setControl(velocityRequest.withVelocity(velocity).withAcceleration(D_TARGET_ACCEL));
    rightRoller.setControl(followRequest);
  }

  public void stopMotor() {
    leftRoller.stopMotor();
    rightRoller.stopMotor();
  }

  public void runAgitateVelocity() {
    setVelocity(TARGET_AGITATE.get());
  }

  @Override
  // update networktables
  public void periodic() {
    leftRollerPub.set(leftRoller.getVelocity().getValueAsDouble());
    rightRollerPub.set(rightRoller.getVelocity().getValueAsDouble());
  }

  // update sim
  public void simulationPeriodic() {
    if (rollerSim != null) {
      rollerSim.updateRollers();
    }
  }
}
