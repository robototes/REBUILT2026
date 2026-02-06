package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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
import frc.robot.generated.CompTunerConstants;

public class IntakeSubsystem extends SubsystemBase {
  private static final double INTAKE_SPEED = 1.0;
  public double pivotPos;
  private final TalonFX pivotMotor;
  private final TalonFX leftRollers;
  private final TalonFX rightRollers;
  private final MotionMagicVoltage pivotRequest = new MotionMagicVoltage(0);
  private final Follower followerRequest =
      new Follower(Hardware.INTAKE_MOTOR_ONE_ID, MotorAlignmentValue.Opposed);
  private static final double PIVOT_DEPLOYED_POS = 0;
  private static final double PIVOT_GEAR_RATIO = 36; // keep this incase for later
  private static final double PIVOT_RETRACTED_POS =
      200 * PIVOT_GEAR_RATIO; // change this value and ln 44 if u wanna change position
  private final DoubleTopic leftRollerTopic;
  private final DoubleTopic rightRollerTopic;
  private final DoublePublisher leftRollerPub;
  private final DoublePublisher rightRollerPub;

  private IntakeSim intakeSim;

  public IntakeSubsystem() {
    pivotMotor = new TalonFX(Hardware.INTAKE_PIVOT_MOTOR_ID);
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

    // default values
    leftRollerPub.set(0);
    rightRollerPub.set(0);
  }

  // configs
  private void TalonFXPivotConfigs() {
    var talonFXConfigs = new TalonFXConfiguration();
    var slot0Configs = talonFXConfigs.Slot0;
    talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // pivot configs

    slot0Configs.kS = 0.9;
    slot0Configs.kV = 0;
    slot0Configs.kA = 0;
    slot0Configs.kP = 40;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;
    slot0Configs.kG =
        0.048; // change PID values during testing, these are placeholders from last year's robot

    var pivotMotionMagicConfigs = talonFXConfigs.MotionMagic;
    pivotMotionMagicConfigs.MotionMagicAcceleration = 50;
    pivotMotionMagicConfigs.MotionMagicJerk = 0;

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
          pivotMotor.setControl(pivotRequest.withPosition(PIVOT_DEPLOYED_POS));
        },
        () -> {
          leftRollers.stopMotor();
          rightRollers.stopMotor();
          pivotMotor.setControl(
              pivotRequest.withPosition(PIVOT_RETRACTED_POS)); // hopefully this retracts the intake
        });
  }

  // public double getPivotPos() {
  //   pivotPos = pivotMotor.getPosition().getValueAsDouble();
  //   return pivotPos;
  // }

  // public Command deployIntake() {
  //   return Commands.runOnce(
  //     () -> {
  //       if (pivotPos == PIVOT_RETRACTED_POS) {
  //         pivotMotor.setControl(pivotRequest.withPosition(PIVOT_DEPLOYED_POS));
  //       } else {
  //         pivotMotor.setControl(pivotRequest.withPosition(PIVOT_RETRACTED_POS));
  //       }
  //     }
  //     );
  // }

  public Command temporaryRunIntake(double speed) { // for testing
    return Commands.runEnd(
        () -> {
          leftRollers.set(speed);
        },
        () -> {
          leftRollers.stopMotor();
        });
  }

  @Override
  public void periodic() {
    leftRollerPub.set(leftRollers.getVelocity().getValueAsDouble());
    rightRollerPub.set(rightRollers.getVelocity().getValueAsDouble());
  }

  @Override
  public void simulationPeriodic() {
    if (intakeSim != null) {
      intakeSim.update();
    }
  }
}
