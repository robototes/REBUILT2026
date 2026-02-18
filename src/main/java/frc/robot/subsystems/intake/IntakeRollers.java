package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
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

public class IntakeRollers extends SubsystemBase {
  // motors
  private final TalonFX leftRoller;
  private final TalonFX rightRoller;
  private final Follower followRequest =
      new Follower(Hardware.INTAKE_MOTOR_ONE_ID, MotorAlignmentValue.Opposed);
  private static final double INTAKE_SPEED = 1.0; // full speed

  // networktables and sim
  private DoubleTopic leftRollerTopic;
  private DoubleTopic rightRollerTopic;
  private DoublePublisher leftRollerPub;
  private DoublePublisher rightRollerPub;
  private RollerSim rollerSim;

  public IntakeRollers() {
    // define motors and configs
    leftRoller = new TalonFX(Hardware.INTAKE_MOTOR_ONE_ID, CompTunerConstants.kCANBus);
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
    talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // motor limits idk if i need to add anymore
    talonFXConfigs.CurrentLimits.StatorCurrentLimit = 60;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimit = 30;
    talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

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

  public Command runRollers() {
    return Commands.runEnd(
        () -> {
          leftRoller.set(INTAKE_SPEED);
          rightRoller.setControl(followRequest);
        },
        () -> {
          leftRoller.stopMotor();
          rightRoller.stopMotor();
        });
  }

  public void stopMotor() {
    leftRoller.stopMotor();
    rightRoller.stopMotor();
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
