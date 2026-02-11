package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Hardware;
import frc.robot.Robot;
import frc.robot.generated.CompTunerConstants;

public class IntakeSubsystem extends SubsystemBase {
  // motor
  private static final double INTAKE_SPEED = 1.0;
  private double pivotPos;
  private double targetPos;
  private final TalonFX pivotMotor;
  private final TalonFX leftRollers;
  private final TalonFX rightRollers;
  private final MotionMagicVoltage pivotRequest = new MotionMagicVoltage(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final Follower followerRequest =
      new Follower(Hardware.INTAKE_MOTOR_ONE_ID, MotorAlignmentValue.Opposed);
  private static final double PIVOT_GEAR_RATIO = 36.0;
  public static final double PIVOT_DEPLOYED_POS = (120.0 / 360.0) * PIVOT_GEAR_RATIO;
  public static final double PIVOT_RETRACTED_POS = 0.0 * PIVOT_GEAR_RATIO;
  public static final double POS_TOLERANCE = Units.degreesToRotations(3) * PIVOT_GEAR_RATIO;
  private boolean pivotIsRetracted = true;
  // sim
  private final DoubleTopic leftRollerTopic;
  private final DoubleTopic rightRollerTopic;
  private final DoubleTopic pivotTopic;
  private final DoublePublisher leftRollerPub;
  private final DoublePublisher rightRollerPub;
  private final DoublePublisher pivotPublisher;
  private IntakeSim intakeSim;

  public IntakeSubsystem() {
    // define the motors
    pivotMotor = new TalonFX(Hardware.INTAKE_PIVOT_MOTOR_ID, CompTunerConstants.kCANBus);
    leftRollers = new TalonFX(Hardware.INTAKE_MOTOR_ONE_ID, CompTunerConstants.kCANBus);
    rightRollers = new TalonFX(Hardware.INTAKE_MOTOR_TWO_ID);
    TalonFXPivotConfigs();
    TalonFXRollerConfigs();

    // intake sim
    if (RobotBase.isSimulation()) {
      intakeSim = new IntakeSim(leftRollers, rightRollers, pivotMotor);
    }

    var nt = NetworkTableInstance.getDefault();
    this.leftRollerTopic = nt.getDoubleTopic("left intake status/speed in RPM");
    this.leftRollerPub = leftRollerTopic.publish();

    this.rightRollerTopic = nt.getDoubleTopic("right intake status/speed in RPM");
    this.rightRollerPub = rightRollerTopic.publish();

    this.pivotTopic = nt.getDoubleTopic("pivot position/position");
    this.pivotPublisher = pivotTopic.publish();

    // default values
    leftRollerPub.set(0);
    rightRollerPub.set(0);
    pivotPublisher.set(0);
  }

  // configs
  private void TalonFXPivotConfigs() {
    var talonFXConfigs = new TalonFXConfiguration();
    var simConfigs = talonFXConfigs.Slot0;
    talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    talonFXConfigs.CurrentLimits.StatorCurrentLimit = 60;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimit = 30;
    talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

    // pivot configs

    simConfigs.kV = 3.0;
    simConfigs.kA = 0.0;
    simConfigs.kP = 0.0;
    simConfigs.kI = 0.0;
    simConfigs.kD = 0.0;
    simConfigs.kG = 0.0;

    var irlConfigs = talonFXConfigs.Slot0;
    irlConfigs.kP = 45;
    irlConfigs.kI = 0.0;
    irlConfigs.kD = 0.0;
    irlConfigs.kA = 0.0;
    irlConfigs.kV = 0;
    irlConfigs.kS = 0.155;
    irlConfigs.kG = 0.0;

    var pivotMotionMagicConfigs = talonFXConfigs.MotionMagic;
    pivotMotionMagicConfigs.MotionMagicAcceleration = 25;
    pivotMotionMagicConfigs.MotionMagicJerk = 0;

    talonFXConfigs.Slot0 = (Robot.isSimulation()) ? simConfigs : irlConfigs;
    pivotMotor.getConfigurator().apply(talonFXConfigs);
  }

  // rollers configs
  private void TalonFXRollerConfigs() {
    var rollerConfigs = new TalonFXConfiguration();
    rollerConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rollerConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    rollerConfigs.CurrentLimits.StatorCurrentLimit = 60;
    rollerConfigs.CurrentLimits.SupplyCurrentLimit = 30;
    rollerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    rollerConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

    leftRollers.getConfigurator().apply(rollerConfigs);
    rightRollers.getConfigurator().apply(rollerConfigs);
  }

  public Command runIntake() {
    return Commands.startEnd(
        () -> {
          leftRollers.set(INTAKE_SPEED);
          rightRollers.setControl(followerRequest); // opposite direction as left rollers
        },
        () -> {
          leftRollers.stopMotor();
          rightRollers.stopMotor();
        });
  }

  public double getPivotPos() {
    return pivotMotor.getPosition().getValueAsDouble() * PIVOT_GEAR_RATIO;
  }

  private Command setPivotPos(double pos) {
    return runOnce(
        () -> {
          pivotMotor.setControl(pivotRequest.withPosition(pos));
          targetPos = pos;
        });
  }

  public Command togglePivot() {
    double targetPos = pivotIsRetracted ? PIVOT_DEPLOYED_POS : PIVOT_RETRACTED_POS;
    pivotIsRetracted = !pivotIsRetracted;
    return Commands.runOnce(
        () -> {
          setPivotPos(targetPos).andThen(Commands.waitUntil(atPosition(targetPos)));
        });
  }

  public Trigger atPosition(double position) {
    return new Trigger(() -> Math.abs(getPivotPos() - position) < POS_TOLERANCE);
  }

  public Command stop() {
    return Commands.runOnce(
        () -> {
          pivotMotor.stopMotor();
        });
  }

  public Command deployIntake() {
    return Commands.runOnce(
        () -> {
          if (Math.abs(getPivotPos() - PIVOT_RETRACTED_POS) < POS_TOLERANCE) {
            pivotMotor.setControl(pivotRequest.withPosition(PIVOT_DEPLOYED_POS));
          } else {
            pivotMotor.setControl(pivotRequest.withPosition(PIVOT_RETRACTED_POS));
          }
        });
  }

  @Override
  public void periodic() {
    leftRollerPub.set(leftRollers.getVelocity().getValueAsDouble());
    rightRollerPub.set(rightRollers.getVelocity().getValueAsDouble());
    pivotPublisher.set(getPivotPos());
  }

  @Override
  public void simulationPeriodic() {
    if (intakeSim != null) {
      intakeSim.update();
    }
  }
}
