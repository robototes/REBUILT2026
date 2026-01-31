package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;

public class IntakeSubsystem extends SubsystemBase {
  private TalonFX pivotMotor;
  private final TalonFX leftRollers; // one
  private final TalonFX rightRollers; // two
  final MotionMagicVoltage pivot_request1 = new MotionMagicVoltage(0);
  double speed;
  private static final double PIVOT_DEPLOYED_POS = 1000;
  private static final double PIVOT_RETRACTED_POS = 0;
  private final DoubleTopic leftRollerTopic;
  private final DoubleTopic rightRollerTopic;
  private final DoublePublisher leftRollerPub;
  private final DoublePublisher rightRollerPub;
  private final FlywheelSim leftRollerSim;
  private final FlywheelSim rightRollerSim;
  private final SingleJointedArmSim pivotSimV2;
  LinearSystem rollerSystem = LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), 1, 1);
  LinearSystem pivotSystem = LinearSystemId.createElevatorSystem(DCMotor.getKrakenX60(1), 40, 1, 1);

  private final double PIVOT_GEAR_RATIO =
      (16.0 / 60.0)
          * (34.0 / 68.0)
          * (18.0 / 40.0); // this is a complete guess that i took from the demo/training repo

  Mechanism2d mech = new Mechanism2d(1, 1);
  MechanismRoot2d pivotShoulder = mech.getRoot("Shoulder", 0.178 / 2.0, 0.2);
  MechanismLigament2d pivotArm =
      pivotShoulder.append(new MechanismLigament2d("arm", Units.inchesToMeters(10.919), 0));

  public IntakeSubsystem(boolean intakepivotEnabled, boolean intakerollersEnabled) {
    speed = 0;

    if (intakepivotEnabled) {
      pivotMotor = new TalonFX(Hardware.INTAKE_PIVOT_MOTOR_ID);

    } else {
      pivotMotor = null;
    }
    if (intakerollersEnabled) {
      leftRollers = new TalonFX(Hardware.INTAKE_MOTOR_ONE_ID);
      rightRollers = new TalonFX(Hardware.INTAKE_MOTOR_TWO_ID);
    } else {
      leftRollers = null;
      rightRollers = null;
    }
    // intake sim

    if (RobotBase.isSimulation()) {
      leftRollerSim = new FlywheelSim(rollerSystem, DCMotor.getKrakenX60(1));
      rightRollerSim = new FlywheelSim(rollerSystem, DCMotor.getKrakenX60(1));

      pivotSimV2 =
          new SingleJointedArmSim(
              DCMotor.getKrakenX60(1),
              0.5,
              0.01,
              1,
              Units.degreesToRadians(-90),
              Units.degreesToRadians(90),
              false,
              Units.degreesToRadians(0));
    } else {
      leftRollerSim = null;
      rightRollerSim = null;
      pivotSimV2 = null;
    }
    var nt = NetworkTableInstance.getDefault();
    this.leftRollerTopic = nt.getDoubleTopic("left intake status/speed in RPM");
    this.leftRollerPub = leftRollerTopic.publish();

    this.rightRollerTopic = nt.getDoubleTopic("right intake status/speed in RPM");
    this.rightRollerPub = rightRollerTopic.publish();

    SmartDashboard.putData("Intake Arm", mech);
  }

  // configs
  private void TalonFXPivotConfigs() {
    var talonFXConfigs1 = new TalonFXConfiguration();
    var slot0Configs = talonFXConfigs1.Slot0;
    talonFXConfigs1.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // pivot configs

    slot0Configs.kS = 0.9;
    slot0Configs.kV = 4;
    slot0Configs.kA = 0;
    slot0Configs.kP = 40;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;
    slot0Configs.kG = 0.048; // change PID values during testing, these are placeholders from last year's robot

    var pivotMotionMagicConfigs = talonFXConfigs1.MotionMagic; // these values i also guessed
    pivotMotionMagicConfigs.MotionMagicAcceleration = 200;

    pivotMotor.getConfigurator().apply(talonFXConfigs1);
    pivotMotor.getConfigurator().apply(pivotMotionMagicConfigs);
  }

  // rollers configs
  private void TalonFXRollerConfigs() {
    var talonFXConfigs2 = new TalonFXConfiguration();
    TalonFXConfigurator rollersCfg = leftRollers.getConfigurator();
    TalonFXConfigurator rollersCFG2 = rightRollers.getConfigurator();
    talonFXConfigs2.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    talonFXConfigs2.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    rollersCfg.apply(talonFXConfigs2);
    rollersCFG2.apply(talonFXConfigs2);
  }

  public double getShoulderRotations() {
    return pivotMotor.getPosition().getValueAsDouble() * PIVOT_GEAR_RATIO;
  }

  public Command runIntake(double speed) {
    return Commands.runEnd(
        () -> {
          pivotMotor.setControl(
              pivot_request1.withPosition(
                  PIVOT_DEPLOYED_POS)); // placeholder value, change during testing
          leftRollers.set(speed);
          rightRollers.setControl(
              new Follower(
                  Hardware.INTAKE_MOTOR_ONE_ID,
                  MotorAlignmentValue.Opposed)); // opposite direction as left rollers
        },
        () -> {
          leftRollers.stopMotor();
          rightRollers.stopMotor();
          pivotMotor.setControl(
              pivot_request1.withPosition(
                  PIVOT_RETRACTED_POS)); // hopefully this retracts the intake
        });
  }

  @Override
  public void simulationPeriodic() {
    leftRollerSim.setInput(leftRollers.getSimState().getMotorVoltage());
    leftRollerSim.update(0.020);
    rightRollerSim.setInput(rightRollers.getSimState().getMotorVoltage());
    rightRollerSim.update(0.020);
    pivotSimV2.setInput(pivotMotor.getSimState().getMotorVoltage());
    pivotSimV2.update(0.020);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            leftRollerSim.getCurrentDrawAmps()
                + rightRollerSim.getCurrentDrawAmps()
                + pivotSimV2.getCurrentDrawAmps()));

    leftRollers
        .getSimState()
        .setRotorVelocity(RadiansPerSecond.of(leftRollerSim.getAngularVelocityRPM()));
    rightRollers
        .getSimState()
        .setRotorVelocity(RadiansPerSecond.of(rightRollerSim.getAngularVelocityRPM()));
    pivotMotor
        .getSimState()
        .setRawRotorPosition(
            Radians.of(pivotSimV2.getAngleRads() * PIVOT_GEAR_RATIO).in(Rotations));
    pivotMotor
        .getSimState()
        .setRotorVelocity(
            RadiansPerSecond.of(pivotSimV2.getVelocityRadPerSec() * PIVOT_GEAR_RATIO)
                .in(RotationsPerSecond));
  }

  @Override
  public void periodic() {
    leftRollerPub.set(leftRollers.getVelocity().getValueAsDouble());
    rightRollerPub.set(rightRollers.getVelocity().getValueAsDouble());

    pivotArm.setAngle(Units.radiansToDegrees(pivotSimV2.getAngleRads()));
  }
}
