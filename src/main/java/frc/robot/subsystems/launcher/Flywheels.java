package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDouble;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Hardware;
import frc.robot.util.tuning.NtTunableBoolean;
import frc.robot.util.tuning.NtTunableDouble;
import java.util.function.DoubleSupplier;

public class Flywheels extends SubsystemBase {
  private final TalonFX FlywheelOne; // left
  private final TalonFX FlywheelTwo; // right
  private final DoubleTopic currentTopic; // supply current in amps
  private final DoubleTopic velocityTopic; // velocity in rps
  private final DoublePublisher currentPub;
  private final DoublePublisher velocityPub;

  private FlywheelsSim flywheelSim;

  private final MotionMagicVelocityVoltage motionMagicRequest = new MotionMagicVelocityVoltage(0);
  private final Follower follow =
      new Follower(Hardware.FLYWHEEL_TWO_ID, MotorAlignmentValue.Opposed);

  public NtTunableDouble targetVelocity;
  private long lastPositionUpdateTime = 0;

  public final double FLYWHEEL_TOLERANCE =
      15; // RPS // increased on drive practice 3/18 from 5 -> 10 //Increased to 15 by TD 3/18
  public final NtTunableBoolean TUNER_CONTROLLED =
      new NtTunableBoolean("/SmartDashboard/Tunables/Flywheels", false);

  // Status signals
  private StatusSignal<AngularVelocity> flywheelOneRPS;

  private final double HAS_SHOT_BALL_RPS = 8; // Rps

  // Cache
  private double cachedLastBallLaunch = 0;
  private boolean hasReachedTerminalVelocity = false;
  private boolean hasShotOnce = false;

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

    flywheelOneRPS = FlywheelOne.getVelocity();
  }

  private void configureMotors() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    TalonFXConfigurator flConfigurator = FlywheelOne.getConfigurator();
    TalonFXConfigurator frConfigurator = FlywheelTwo.getConfigurator();
    // set current limits
    config.CurrentLimits.SupplyCurrentLimit = 60;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = 0;
    config.CurrentLimits.StatorCurrentLimit = 80;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    // create coast mode for motors
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // create PID gains
    config.Slot0.kP = 1;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    config.Slot0.kA = 0.0;
    config.Slot0.kV = 8.73 / 74;
    config.Slot0.kS = 0.0;
    config.Slot0.kG = 0.0;

    config.MotionMagic.MotionMagicAcceleration = 1000; // RPS^2

    flConfigurator.apply(config);
    frConfigurator.apply(config);
  }

  public Command setVelocityCommand(double rps) {
    return runEnd(
            () -> {
              motionMagicRequest.Velocity = rps;
              FlywheelTwo.setControl(motionMagicRequest);
              FlywheelOne.setControl(follow);
            },
            () -> {
              FlywheelOne.stopMotor();
              FlywheelTwo.stopMotor();
            })
        .withName("Set Flywheel Velocity");
  }

  public Command suppliedSetVelocityCommand(DoubleSupplier rps) {
    return runEnd(
            () -> {
              motionMagicRequest.Velocity = rps.getAsDouble();
              FlywheelTwo.setControl(motionMagicRequest);
              FlywheelOne.setControl(follow);
            },
            () -> {
              FlywheelOne.stopMotor();
              FlywheelTwo.stopMotor();
            })
        .withName("Set Flywheel Supplied Velocity");
  }

  public void setVelocityRPS(double rps) {
    motionMagicRequest.Velocity = rps;
    FlywheelTwo.setControl(motionMagicRequest);
    FlywheelOne.setControl(follow);
  }

  public Command stopCommand() {
    return runOnce(
            () -> {
              FlywheelOne.stopMotor();
              FlywheelTwo.stopMotor();
            })
        .withName("Stop Flywheels");
  }

  public void stopVoid() {
    FlywheelOne.stopMotor();
    FlywheelTwo.stopMotor();
  }

  public boolean atTargetVelocity(double targetRPS, double toleranceRPS) {
    double velocity = (FlywheelOne.getVelocity().getValueAsDouble());

    boolean atTarget = Math.abs(velocity - targetRPS) <= toleranceRPS;
    return atTarget;
  }

  public Trigger atTargetVelocityTrigger(double targetRPS, double toleranceRPS) {
    return new Trigger(() -> atTargetVelocity(targetRPS, toleranceRPS));
  }

  public double lastBallLaunch() {
    if (MathUtil.applyDeadband(
            targetVelocity.get() - flywheelOneRPS.getValueAsDouble(), HAS_SHOT_BALL_RPS)
        == 0) {
      if (!hasShotOnce) {
        hasShotOnce = true;
      }
      cachedLastBallLaunch = Timer.getFPGATimestamp();
      return cachedLastBallLaunch;
    } else if (hasShotOnce
        && MathUtil.applyDeadband(
                targetVelocity.get() - flywheelOneRPS.getValueAsDouble(), FLYWHEEL_TOLERANCE)
            == 0.0) {
      cachedLastBallLaunch = Timer.getFPGATimestamp();
      return cachedLastBallLaunch;
    }
    return cachedLastBallLaunch;
  }

  public boolean stoppedShooting(double HAS_SHOT_MAX_TIME) {
    if (MathUtil.applyDeadband(
            targetVelocity.get() - flywheelOneRPS.getValueAsDouble(), FLYWHEEL_TOLERANCE)
        == 0.0) {
      hasReachedTerminalVelocity = true;
    }
    double lastTime = lastBallLaunch();
    if (!hasReachedTerminalVelocity || !hasShotOnce) {
      return false;
    }

    double currentTime = Timer.getFPGATimestamp();
    if (currentTime - lastTime >= HAS_SHOT_MAX_TIME) {
      resetCachedValues();
      return true;
    }
    return false;
  }

  public void resetCachedValues() {
    hasReachedTerminalVelocity = false;
    hasShotOnce = false;
    cachedLastBallLaunch = 0;
  }

  @Override
  public void simulationPeriodic() {
    if (flywheelSim != null) {
      flywheelSim.update();
    }
  }

  @Override
  public void periodic() {
    flywheelOneRPS.refresh();
    velocityPub.set(flywheelOneRPS.getValueAsDouble());
    currentPub.set(FlywheelOne.getSupplyCurrent().getValueAsDouble());
    if (TUNER_CONTROLLED.get()) {
      if (targetVelocity.hasChangedSince(lastPositionUpdateTime)) {
        TimestampedDouble currentTarget = targetVelocity.getAtomic();
        setVelocityRPS(currentTarget.value);
        lastPositionUpdateTime = currentTarget.timestamp;
      }
    }
  }
}
