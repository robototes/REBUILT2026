package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDouble;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;
import frc.robot.Robot;
import frc.robot.util.robotType.RobotType;
import frc.robot.util.tuning.NtTunableBoolean;
import frc.robot.util.tuning.NtTunableDouble;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;

public class Hood extends SubsystemBase {
  private final TalonFX hood;
  private HoodSim hoodSim;

  private DoublePublisher positionPub; // hood pose in rotations
  private DoublePublisher goalPub; // hood pose in rotations
  private BooleanPublisher zeroPublisher;

  @Getter private boolean hoodZeroed = false; // is hood Zeroed

  private final MotionMagicVoltage request = new MotionMagicVoltage(0);
  private final VoltageOut voltageRequest = new VoltageOut(0).withIgnoreSoftwareLimits(true);

  public NtTunableDouble targetPosition;
  private long lastPositionUpdateTime = 0;

  // mechanism gear ratio = 104.65278
  private static final double TARGET_TOLERANCE = 0.23; // tolerance in motor rotations
  public static final double VOLTAGE_MANUAL_CONTROL =
      1; // voltage/speed to control the motor for manual control
  private static final double STATOR_CURRENT_LIMIT = 30; // stator limit in amps
  // both forward and backward soft limits are in Rotor rotations
  private static final double FORWARD_SOFT_LIMIT =
      11.628; // LIMITED TO 19 - March 14th Physical limit
  private static final double BACKWARD_SOFT_LIMIT = -0.01224; // -0.02 rotations, past zeroing point

  public final NtTunableBoolean TUNER_CONTROLLED =
      new NtTunableBoolean("/SmartDashboard/Tunables/Hood", false);

  // Mechanism tuning required !! TODO: TUNE
  private static final double AUTO_ZERO_VOLTAGE = -3;

  public Hood() {
    hood = new TalonFX(Hardware.HOOD_MOTOR_ID);

    configureMotor();
    initializeNT();
    if (RobotBase.isSimulation()) {
      hoodSim = new HoodSim(hood);
    }
  }

  public void initializeNT() {
    var nt = NetworkTableInstance.getDefault();
    positionPub = nt.getDoubleTopic("/hood/position").publish();
    positionPub.set(0);
    goalPub = nt.getDoubleTopic("/hood/goal").publish();
    goalPub.set(request.Position);
    zeroPublisher = nt.getBooleanTopic("/Zero/hoodZero").publish();
    zeroPublisher.set(false);
    targetPosition = new NtTunableDouble("/launcher/hoodTuner", 0.0);
  }

  public void configureMotor() {
    // Motor configuration code goes here
    TalonFXConfiguration config = new TalonFXConfiguration();
    TalonFXConfigurator hood_Configurator = hood.getConfigurator();

    // set current limits
    config.CurrentLimits.SupplyCurrentLimit = 20;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    // create brake mode for motors
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = FORWARD_SOFT_LIMIT;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = BACKWARD_SOFT_LIMIT;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    // IRL PID gains
    var irlPID = new Slot0Configs();
    irlPID.kP = 40;
    irlPID.kI = 0.0;
    irlPID.kD = 0;
    irlPID.kA = 0.0;
    irlPID.kV = 0;
    irlPID.kS = 0.155;
    irlPID.kG = 0.0;

    // SIM PID gains, requires different gains because sim takes less things into account
    var simPID = new Slot0Configs();
    simPID.kP = 5;
    simPID.kI = 0.0;
    simPID.kD = 0;
    simPID.kA = 0.0;
    simPID.kV = 0;
    simPID.kS = 0;
    simPID.kG = 0.0;

    config.MotionMagic.MotionMagicCruiseVelocity = RobotType.isAlpha() ? 5 : 60;
    config.MotionMagic.MotionMagicAcceleration = RobotType.isAlpha() ? 5 : 600;
    config.MotionMagic.MotionMagicJerk = RobotType.isAlpha() ? 0 : 6000;

    config.Slot0 = (Robot.isSimulation()) ? simPID : irlPID;

    hood_Configurator.apply(config);
  }

  @Override
  public void periodic() {
    positionPub.set(hood.getPosition().getValueAsDouble());
    goalPub.set(request.Position);
    if (TUNER_CONTROLLED.get()) {
      if (targetPosition.hasChangedSince(lastPositionUpdateTime)) {
        TimestampedDouble currentTarget = targetPosition.getAtomic();
        setHoodPosition(currentTarget.value);
        lastPositionUpdateTime = currentTarget.timestamp;
      }
    }
  }

  public double getHoodPosition() {
    return hood.getPosition().getValueAsDouble();
  }

  public void setHoodPosition(double positionRotations) {
    hood.setControl(request.withPosition(positionRotations));
  }

  public Command hoodPositionCommand(double positionRotations) {
    return runEnd(() -> setHoodPosition(positionRotations), () -> hood.stopMotor())
        .withName("Setting Hood position")
        .onlyIf(() -> hoodZeroed);
  }

  public Command suppliedHoodPositionCommand(DoubleSupplier positionRotations) {
    return runEnd(() -> setHoodPosition(positionRotations.getAsDouble()), () -> hood.stopMotor())
        .withName("Setting hood position - Supplied")
        .onlyIf(() -> hoodZeroed);
  }

  public void zero() {
    hood.setPosition(0);
    hoodZeroed = true;
    zeroPublisher.set(true);
  }

  public Command zeroHoodCommand() {
    return runOnce(() -> zero()).withName("Zeroing Hood");
  }

  public boolean atTargetPosition() {
    return DriverStation.isEnabled()
        && Math.abs(hood.getPosition().getValueAsDouble() - request.Position) < TARGET_TOLERANCE;
  }

  public Command voltageControl(Supplier<Voltage> voltageSupplier) {
    return runEnd(
            () -> {
              // hood.setVoltage(voltageSupplier.get().in(Units.Volts));
              hood.setControl(voltageRequest.withOutput(voltageSupplier.get()));
            },
            () -> {
              hood.stopMotor();
            })
        .withName("Voltage Control");
  }

  public Command autoZeroCommand() {
    if (Robot.isSimulation()) {
      return zeroHoodCommand();
    }
    return Commands.sequence(
            voltageControl(() -> Volts.of(AUTO_ZERO_VOLTAGE))
                .withDeadline(
                    Commands.waitSeconds(0.5)
                        .andThen(
                            Commands.waitUntil(
                                () ->
                                    hood.getStatorCurrent().getValueAsDouble()
                                        >= (STATOR_CURRENT_LIMIT - 1)))),
            zeroHoodCommand())
        .withTimeout(3)
        .withName("Automatic Zero Hood");
  }

  @Override
  public void simulationPeriodic() {
    if (hoodSim != null) {
      hoodSim.update();
    }
  }
}
