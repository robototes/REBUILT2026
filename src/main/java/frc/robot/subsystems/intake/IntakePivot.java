package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;
import frc.robot.Robot;
import frc.robot.generated.CompTunerConstants;
import java.util.function.Supplier;

public class IntakePivot extends SubsystemBase {
  private final TalonFX pivotMotor;
  private final MotionMagicVoltage request = new MotionMagicVoltage(0);
  private final VoltageOut voltageRequest = new VoltageOut(0).withIgnoreSoftwareLimits(true);
  private static final double AUTO_ZERO_VOLTAGE = 8;
  private final Timer timer = new Timer();

  // Positions
  // -0.17 Vertical Pos.
  private double targetPos;
  public static final double DEPLOYED_POS = -0.43;
  public static final double LAUNCH_POS_IN = -0.13;
  public static final double LAUNCH_POS_OUT = -0.292;
  public static final double RETRACTED_POS = 0.0;
  public static final double EXTAKE_POS = -0.3;
  public static final double SHOTBLOCK_POS = -0.25;

  // PID variables
  private static final double kP = 40;
  private static final double kI = 0;
  private static final double kD = 1;
  private static final double kG = 0.4;
  private static final double kS = 0.2;
  private static final double kV = 0;
  private static final double kA = 0;

  // Current limits
  private static final int STATOR_CURRENT_LIMIT = 60; // amps
  private static final int SUPPLY_CURRENT_LIMIT = 30; // amps

  // Motion Magic Config
  private static final double CRUISE_VELOCITY = 100;
  private static final double ACCELERATION = 400;
  private static final double JERK = 0;

  // Gear Ratio
  private static final double GEAR_RATIO = 35;

  // Soft Limits
  public static final double PIVOT_MIN = -0.45; // rotations
  public static final double PIVOT_MAX = 0.0;

  // Simulator and NetworkTables
  private PivotSim pivotSim;
  private DoublePublisher currentPosPub;
  private DoublePublisher targetPosPub;
  private BooleanPublisher zeroPublisher;

  // Status Signals
  private final StatusSignal<Current> SS_intakePivotCurrent;
  private final StatusSignal<Angle> SS_position;

  public IntakePivot() {
    pivotMotor = new TalonFX(Hardware.INTAKE_PIVOT_MOTOR_ID, CompTunerConstants.kCANBus);
    pivotConfig();
    pivotMotor.clearStickyFaults();
    networktables();
    if (Robot.isSimulation()) {
      pivotSim = new PivotSim(pivotMotor);
    }
    SS_intakePivotCurrent = pivotMotor.getStatorCurrent();
    SS_position = pivotMotor.getPosition();
  }

  public void pivotConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

    config.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = PIVOT_MAX;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = PIVOT_MIN;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    config.MotionMagic.MotionMagicCruiseVelocity = CRUISE_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = ACCELERATION;
    config.MotionMagic.MotionMagicJerk = JERK;

    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    config.Slot0.kG = kG;
    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kA = kA;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    pivotMotor.getConfigurator().apply(config);
  }

  private void networktables() {
    var nt = NetworkTableInstance.getDefault();
    this.currentPosPub = nt.getDoubleTopic("intake/pivotCurrentPosition").publish();
    this.targetPosPub = nt.getDoubleTopic("intake/pivotTargetPosition").publish();
    this.zeroPublisher =
        NetworkTableInstance.getDefault().getBooleanTopic("/Zero/intakePivotZero").publish();

    currentPosPub.set(0.0); // default value
    targetPosPub.set(0.0); // default value
    zeroPublisher.set(false);
  }

  public void setPivotPosition(double pos) {
    targetPos = pos;
    pivotMotor.setControl(request.withPosition(pos));
  }

  public void restartTimer() {
    timer.restart();
  }

  public void oscillatePivot() {
    double offset = (1 + Math.cos(2 * Math.PI * (timer.get() / 0.5) + Math.PI)) / 2;
    double pos = LAUNCH_POS_OUT + offset * (LAUNCH_POS_IN - LAUNCH_POS_OUT);
    targetPos = pos;
    setPivotPosition(pos);
  }

  public Command zeroPivot() {
    return runOnce(
            () -> {
              pivotMotor.setPosition(RETRACTED_POS);
              targetPos = RETRACTED_POS;
              zeroPublisher.set(true);
            })
        .withName("Zero Pivot");
  }

  public Command voltageControl(Supplier<Voltage> voltageSupplier) {
    return runEnd(
            () -> {
              pivotMotor.setControl(voltageRequest.withOutput(voltageSupplier.get()));
            },
            () -> {
              pivotMotor.stopMotor();
            })
        .withName("Voltage Control");
  }

  public double getPivotPosition() {
    return SS_position.getValueAsDouble();
  }

  public double getPivotTargetPosition() {
    return targetPos;
  }

  public boolean isAtTarget(double degreeTolerance, double pose) {
    return Math.abs(SS_position.getValueAsDouble() - pose)
        < Units.degreesToRotations(degreeTolerance);
  }

  public boolean isAtTarget(double degreeTolerance) {
    return isAtTarget(degreeTolerance, targetPos);
  }

  public Command autoZeroCommand() {
    if (Robot.isSimulation()) {
      return zeroPivot();
    }
    return Commands.sequence(
            voltageControl(() -> Volts.of(AUTO_ZERO_VOLTAGE))
                .withDeadline(
                    Commands.waitSeconds(0.5)
                        .andThen(
                            Commands.run(() -> SS_intakePivotCurrent.refresh())
                                .until(
                                    () ->
                                        SS_intakePivotCurrent.getValueAsDouble()
                                            >= (STATOR_CURRENT_LIMIT - 1)))),
            zeroPivot())
        .withTimeout(3)
        .withName("Automatic Zero Pivot");
  }

  @Override
  // update simulation
  public void periodic() {
    StatusSignal.refreshAll(SS_position);
    currentPosPub.set(getPivotPosition());
    targetPosPub.set(getPivotTargetPosition());
  }

  public void simulationPeriodic() {
    if (pivotSim != null) {
      pivotSim.updateArm();
    }
  }
}
