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
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;

public class IntakeSubsystem extends SubsystemBase {
  private static final double speed2 = 1;
  private final TalonFX pivotMotor;
  private final TalonFX leftRollers; // one
  private final TalonFX rightRollers; // two
  private final MotionMagicVoltage pivotRequest = new MotionMagicVoltage(0);
  private final Follower followerRequest =
      new Follower(Hardware.INTAKE_MOTOR_ONE_ID, MotorAlignmentValue.Opposed);
  private static final double PIVOT_DEPLOYED_POS = 0;
  private static final double PIVOT_RETRACTED_POS =
      1000; // change this value and ln 44 if u wanna change position
  private final DoubleTopic leftRollerTopic;
  private final DoubleTopic rightRollerTopic;
  private final DoublePublisher leftRollerPub;
  private final DoublePublisher rightRollerPub;

  private static final double PIVOT_GEAR_RATIO = 36; // keep this incase for later

  // arm length is 10.919 meters, there was a variable here for it but i replaced it with this
  // comment

  private IntakeSim intakeSim;

  public IntakeSubsystem(Mechanism2d mech) {
    pivotMotor = new TalonFX(Hardware.INTAKE_PIVOT_MOTOR_ID);
    leftRollers = new TalonFX(Hardware.INTAKE_MOTOR_ONE_ID);
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
    pivotMotionMagicConfigs.MotionMagicCruiseVelocity = 0;
    pivotMotionMagicConfigs.MotionMagicJerk = 0;

    pivotMotor.getConfigurator().apply(talonFXConfigs);
    System.out.println("configure pivot");
  }

  // rollers configs
  private void TalonFXRollerConfigs() {
    var rollerConfigs = new TalonFXConfiguration();
    rollerConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rollerConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    leftRollers.getConfigurator().apply(rollerConfigs);
    rightRollers.getConfigurator().apply(rollerConfigs);
    System.out.println("configure rollers");
  }

  public Command runIntake() {
    return Commands.runEnd(
        () -> {
          System.out.println("left roll go"); // for testing
          leftRollers.set(speed2);
          rightRollers.setControl(followerRequest); // opposite direction as left rollers
          pivotMotor.setControl(pivotRequest.withPosition(PIVOT_DEPLOYED_POS));
          System.out.println("intake speed " + speed2);
        },
        () -> {
          System.out.println("left roll stop"); // for testing
          leftRollers.stopMotor();
          rightRollers.stopMotor();
          pivotMotor.setControl(
              pivotRequest.withPosition(PIVOT_RETRACTED_POS)); // hopefully this retracts the intake
        });
  }
  ;

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
