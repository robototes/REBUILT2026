package frc.robot.subsystems.Launcher;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;
import frc.robot.Robot;
import frc.robot.util.NtTunableDouble;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;

public class Hood extends SubsystemBase {
  private TalonFX hood;
  private HoodSim hoodSim;

  private DoubleTopic positionTopic; // hood pose in rotations
  private DoublePublisher positionPub;
  private DoubleTopic goalTopic; // hood pose in rotations
  private DoublePublisher goalPub;
  @Getter private boolean hoodZeroed = false; // is hood Zeroed

  private final MotionMagicVoltage request = new MotionMagicVoltage(0);
  private final VoltageOut voltageRequest = new VoltageOut(0).withIgnoreSoftwareLimits(true);

  public NtTunableDouble targetPosition;

  private static final double TARGET_TOLERANCE = 0.1; // tolerance in motor rotations
  public static final double VOLTAGE_MANUAL_CONTROL =
      1; // voltage/speed to control the motor for manual control
  private static final double STATOR_CURRENT_LIMIT = 10; // stator limit in amps
  // GEAR_RATIO = 2.90909;
  private static final double FORWARD_SOFT_LIMIT = 1.72; // 1.72 rotations
  private static final double BACKWARD_SOFT_LIMIT = -0.02; // -0.02 rotations, past zeroing point

  public final boolean TUNER_CONTROLLED = false; // boolean to check if tuner control is being used

  // Mechanism tuning required !! TODO: TUNE
  private static final double AUTO_ZERO_VOLTAGE = -0.5;

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
    positionTopic = nt.getDoubleTopic("/hood/position");
    positionPub = positionTopic.publish();
    positionPub.set(0);
    goalTopic = nt.getDoubleTopic("/hood/goal");
    goalPub = goalTopic.publish();
    goalPub.set(request.Position);

    targetPosition = new NtTunableDouble("/hood/targetPosition", 0.0);
  }

  public void configureMotor() {
    // Motor configuration code goes here
    TalonFXConfiguration config = new TalonFXConfiguration();
    TalonFXConfigurator hood_Configurator = hood.getConfigurator();

    // set current limits
    config.CurrentLimits.SupplyCurrentLimit = 5;
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
    irlPID.kP = 45;
    irlPID.kI = 0.0;
    irlPID.kD = 0.0;
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

    config.MotionMagic.MotionMagicCruiseVelocity = 32;
    config.MotionMagic.MotionMagicAcceleration = 64;

    config.Slot0 = (Robot.isSimulation()) ? simPID : irlPID;

    hood_Configurator.apply(config);
  }

  @Override
  public void periodic() {
    setHoodPosition(targetPosition.get());
    positionPub.set(hood.getPosition().getValueAsDouble());
    goalPub.set(request.Position);
  }

  public double getHoodPosition() {
    return hood.getPosition().getValueAsDouble();
  }

  public void setHoodPosition(double positionRotations) {
    if (!hoodZeroed) {
      System.out.println("Hood not zero'd!");
      return;
    }
    hood.setControl(request.withPosition(positionRotations));
  }

  public Command hoodPositionCommand(double positionRotations) {
    return runOnce(() -> setHoodPosition(positionRotations)).withName("setting Hood position");
  }

  public Command suppliedHoodPositionCommand(DoubleSupplier positionRotations) {
    return runOnce(() -> setHoodPosition(positionRotations.getAsDouble()))
        .withName("Setting hood position - Supplied");
  }

  public void zero() {
    hood.setPosition(0);
    hoodZeroed = true;
  }

  public Command zeroHoodCommand() {
    return runOnce(this::zero).withName("Zeroing Hood");
  }

  public boolean atTargetPosition() {
    return DriverStation.isEnabled()
        && hoodZeroed
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
    return Commands.parallel(voltageControl(() -> Volts.of(AUTO_ZERO_VOLTAGE)))
        .until(() -> hood.getStatorCurrent().getValueAsDouble() >= (STATOR_CURRENT_LIMIT - 1))
        .andThen(zeroHoodCommand())
        .withTimeout(3)
        .withName("Automatic Zero hood");
  }

  @Override
  public void simulationPeriodic() {
    if (hoodSim != null) {
      hoodSim.update();
    }
  }
}
