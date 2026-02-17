package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDouble;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Hardware;
import frc.robot.util.NtTunableDouble;
import java.util.function.DoubleSupplier;

public class Flywheels extends SubsystemBase {
  private final TalonFX FlywheelOne; // left
  private final TalonFX FlywheelTwo; // right
  private final DoubleTopic currentTopic; // supply current in amps
  private final DoubleTopic velocityTopic; // velocity in rps
  private final DoublePublisher currentPub;
  private final DoublePublisher velocityPub;

  private FlywheelsSim flywheelSim;

  private final MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(0);
  private final Follower follow =
      new Follower(Hardware.FLYWHEEL_ONE_ID, MotorAlignmentValue.Opposed);

  public NtTunableDouble targetVelocity;
  private long lastPositionUpdateTime = 0;

  public final double FLYWHEEL_TOLERANCE = 5; // RPS
  public final boolean TUNER_CONTROLLED =
      false; // boolean to decide if it should be controlled using the NtTunableDouble

  // Constructor
  public Flywheels() {
    FlywheelOne = new TalonFX(Hardware.FLYWHEEL_ONE_ID);
    FlywheelTwo = new TalonFX(Hardware.FLYWHEEL_TWO_ID);

    targetVelocity = new NtTunableDouble("/launcher/flywheelTuner", 0.0);
    configureMotors();

    var nt = NetworkTableInstance.getDefault();
    velocityTopic = nt.getDoubleTopic("/launcher/velocity");
    currentTopic = nt.getDoubleTopic("/launcher/current");
    velocityPub = velocityTopic.publish();
    currentPub = currentTopic.publish();
    velocityPub.set(0.0);
    currentPub.set(0.0);

    if (RobotBase.isSimulation()) {
      flywheelSim = new FlywheelsSim(FlywheelOne, FlywheelTwo);
    }
  }

  private void configureMotors() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    TalonFXConfigurator flConfigurator = FlywheelOne.getConfigurator();
    // set current limits
    config.CurrentLimits.SupplyCurrentLimit = 20;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 40;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    // create coast mode for motors
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // create PID gains
    config.Slot0.kP = 0.1;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    config.Slot0.kA = 0.0;
    config.Slot0.kV = 11.825 / 99;
    config.Slot0.kS = 0.0;
    config.Slot0.kG = 0.0;

    config.MotionMagic.MotionMagicAcceleration = 40; // RPS^2

    flConfigurator.apply(config);
  }

  public Command setVelocityCommand(double rps) {
    return runEnd(
            () -> {
              request.Velocity = rps;
              FlywheelOne.setControl(request);
              FlywheelTwo.setControl(follow);
            },
            () -> {
              FlywheelOne.stopMotor();
              FlywheelTwo.stopMotor();
            })
        .withName("Set Flywheel Velocity");
  }

  public Command suppliedSetVelocityCommand(DoubleSupplier rps) {
    return runOnce(
            () -> {
              request.Velocity = rps.getAsDouble();
              FlywheelOne.setControl(request);
              FlywheelTwo.setControl(follow);
            })
        .withName("Set Flywheel Velocity");
  }

  public void setVelocityRPS(double rps) {
    request.Velocity = rps;
    FlywheelOne.setControl(request);
    FlywheelTwo.setControl(follow);
  }

  public Command stopCommand() {
    return runOnce(
            () -> {
              FlywheelOne.stopMotor();
              FlywheelTwo.stopMotor();
            })
        .withName("Stop Flywheels");
  }

  public boolean atTargetVelocity(double targetRPS, double toleranceRPS) {
    double velocity = (FlywheelOne.getVelocity().getValueAsDouble());

    boolean atTarget = Math.abs(velocity - targetRPS) <= toleranceRPS;
    return atTarget;
  }

  public Trigger atTargetVelocityTrigger(double targetRPS, double toleranceRPS) {
    return new Trigger(() -> atTargetVelocity(targetRPS, toleranceRPS));
  }

  @Override
  public void simulationPeriodic() {
    if (flywheelSim != null) {
      flywheelSim.update();
    }
  }

  @Override
  public void periodic() {
    velocityPub.set(FlywheelOne.getVelocity().getValueAsDouble());
    currentPub.set(FlywheelOne.getSupplyCurrent().getValueAsDouble());
    if (TUNER_CONTROLLED) {
      if (targetVelocity.hasChangedSince(lastPositionUpdateTime)) {
        TimestampedDouble currentTarget = targetVelocity.getAtomic();
        setVelocityRPS(currentTarget.value);
        lastPositionUpdateTime = currentTarget.timestamp;
      }
    }
  }
}
