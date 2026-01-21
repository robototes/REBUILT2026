package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedBoolean;
import edu.wpi.first.networktables.TimestampedDouble;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;
import frc.robot.util.NtTunableBoolean;
import frc.robot.util.NtTunableDouble;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

public class Hood extends SubsystemBase {
  // Hood subsystem implementation goes here
  private TalonFX hood;
  private DoubleTopic positionTopic;
  private DoublePublisher positionPub;

  public boolean hoodZeroed = false;

  private final MotionMagicVoltage request = new MotionMagicVoltage(0);

  private long lastPositionUpdateTime = 0;
  private long lastTunerControlledupdateTime = 0;
  private NtTunableDouble targetPosition;
  private NtTunableBoolean tunerControlled;

  private static final double TARGET_TOLERANCE = 0.1;
  public static final double VOLTAGE_CONTROL = 1;
  private static final double STATOR_CURRENT_LIMIT = 20;

  public Hood() {
    hood = new TalonFX(Hardware.HOOD_MOTOR_ID);

    configureMotor();
    initializeNT();
  }

  public void initializeNT() {
    var nt = NetworkTableInstance.getDefault();
    positionTopic = nt.getDoubleTopic("/hood/position");
    positionPub = positionTopic.publish();
    positionPub.set(0);

    targetPosition = new NtTunableDouble("/hood/targetPosition", 0.0);
    tunerControlled = new NtTunableBoolean("/hood/tunerControlled", false);
  }

  public void configureMotor() {
    // Motor configuration code goes here
    TalonFXConfiguration config = new TalonFXConfiguration();
    TalonFXConfigurator hood_Configurator = hood.getConfigurator();

    // set current limits
    config.CurrentLimits.SupplyCurrentLimit = 10;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    // create brake mode for motors
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // create PID gains
    config.Slot0.kP = 0.00;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    config.Slot0.kA = 0.0;
    config.Slot0.kV = 0;
    config.Slot0.kS = 0.0;
    config.Slot0.kG = 0.0;

    config.MotionMagic.MotionMagicCruiseVelocity = 80;
    config.MotionMagic.MotionMagicAcceleration = 160;

    hood_Configurator.apply(config);
  }

  @Override
  public void periodic() {

    positionPub.set(hood.getPosition().getValueAsDouble());

    if (tunerControlled.hasChangedSince(lastTunerControlledupdateTime)) {
      TimestampedBoolean currentTarget = tunerControlled.getAtomic();
      lastTunerControlledupdateTime = currentTarget.timestamp;
      setHoodPosition(targetPosition.get());
    }
    if (targetPosition.hasChangedSince(lastPositionUpdateTime)) {
      TimestampedDouble currentTarget = targetPosition.getAtomic();
      if (tunerControlled.get()) {
        setHoodPosition(currentTarget.value);
      }
      lastPositionUpdateTime = currentTarget.timestamp;
    }
  }

  public double getHoodPosition() {
    return hood.getPosition().getValueAsDouble();
  }

  public void setHoodPosition(double positionRotations) {
    if (!hoodZeroed) {
      return;
    }
    hood.setControl(request.withPosition(positionRotations));
  }

  public Command hoodPositionCommand(double positionRotations) {
    return runOnce(() -> setHoodPosition(positionRotations));
  }

  private void zero() {
    hood.setPosition(0);
    hoodZeroed = true;
  }

  public Command zeroHoodCommand() {
    return runOnce(this::zero);
  }

  public boolean atTargetPosition() {
    return DriverStation.isEnabled()
        && hoodZeroed
        && Math.abs(hood.getPosition().getValueAsDouble() - request.Position) < TARGET_TOLERANCE;
  }

  public Command voltageControl(Supplier<Voltage> voltageSupplier) {
    return runEnd(
        () -> {
          hood.setVoltage(voltageSupplier.get().in(Units.Volts));
        },
        () -> {
          hood.stopMotor();
        });
  }

  public Command autoZeroCommand(){
    return Commands.parallel(voltageControl(() -> Volts.of(-0.5))).until(() -> hood.getStatorCurrent().getValueAsDouble() > STATOR_CURRENT_LIMIT).andThen(zeroHoodCommand());
  }
}
