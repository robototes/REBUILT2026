package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;

public class IntakeSubsystem extends SubsystemBase {
  private final TalonFX pivotMotor;
  private final TalonFX leftRollers; // one
  private final TalonFX rightRollers; // two
  private final MotionMagicVoltage pivotRequest = new MotionMagicVoltage(0);
  private final Follower followerRequest =
      new Follower(Hardware.INTAKE_MOTOR_ONE_ID, MotorAlignmentValue.Opposed);
  private static final double PIVOT_DEPLOYED_POS = 0;
  private static final double PIVOT_RETRACTED_POS =
      1000; // change this value and ln 44 if u wanna change position
  private static final double updateSim = 0.02;
  private final DoubleTopic leftRollerTopic;
  private final DoubleTopic rightRollerTopic;
  private final DoublePublisher leftRollerPub;
  private final DoublePublisher rightRollerPub;
  private final FlywheelSim leftRollerSim;
  private final FlywheelSim rightRollerSim;
  private final SingleJointedArmSim pivotSimV2;
  LinearSystem rollerSystem = LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), 1, 1);

  private static final double PIVOT_GEAR_RATIO = (54.0 / 12.0) * (54.0 / 18.0) * (48.0 / 18.0);
  private static final double ARM_LENGTH_METERS = Units.inchesToMeters(10.919);

  private final MechanismRoot2d pivotShoulder;
  private final MechanismLigament2d pivotArm;

  public IntakeSubsystem(Mechanism2d mechanism2d) {
    pivotShoulder = mechanism2d.getRoot("Shoulder", 0.178 / 2.0, 0.2);
    pivotArm = pivotShoulder.append(new MechanismLigament2d("arm", ARM_LENGTH_METERS, 90));
    pivotMotor = new TalonFX(Hardware.INTAKE_PIVOT_MOTOR_ID);
    TalonFXPivotConfigs();
    leftRollers = new TalonFX(Hardware.INTAKE_MOTOR_ONE_ID);
    rightRollers = new TalonFX(Hardware.INTAKE_MOTOR_TWO_ID);
    TalonFXRollerConfigs();

    // intake sim

    if (RobotBase.isSimulation()) {
      leftRollerSim = new FlywheelSim(rollerSystem, DCMotor.getKrakenX60(1));
      rightRollerSim = new FlywheelSim(rollerSystem, DCMotor.getKrakenX60(1));

      pivotSimV2 =
          new SingleJointedArmSim(
              DCMotor.getKrakenX60(1),
              PIVOT_GEAR_RATIO,
              SingleJointedArmSim.estimateMOI(ARM_LENGTH_METERS, 2),
              Units.inchesToMeters(ARM_LENGTH_METERS),
              Units.degreesToRadians(0), // minimum arm angle
              Units.degreesToRadians(120), // maximum arm angle
              false,
              Units.degreesToRadians(
                  120)); // starting arm angle, keep this the same value as max arm angle.
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
  }

  // configs
  private void TalonFXPivotConfigs() {
    var talonFXConfigs = new TalonFXConfiguration();
    var slot0Configs = talonFXConfigs.Slot0;
    talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // pivot configs

    slot0Configs.kS = 0.9;
    slot0Configs.kV = 0;
    slot0Configs.kA = 0;
    slot0Configs.kP = 40;
    slot0Configs.kI = 5;
    slot0Configs.kD = 0;
    slot0Configs.kG =
        0.048; // change PID values during testing, these are placeholders from last year's robot

    var pivotMotionMagicConfigs = talonFXConfigs.MotionMagic;
    pivotMotionMagicConfigs.MotionMagicAcceleration = 3000;
    pivotMotionMagicConfigs.MotionMagicJerk = 10;

    pivotMotor.getConfigurator().apply(talonFXConfigs);
  }

  // rollers configs
  private void TalonFXRollerConfigs() {
    var rollerConfigs = new TalonFXConfiguration();
    rollerConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rollerConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    System.out.println("configuring roller");
    leftRollers.getConfigurator().apply(rollerConfigs);
    rightRollers.getConfigurator().apply(rollerConfigs);
  }

  public double getShoulderRotations() {
    return pivotMotor.getPosition().getValueAsDouble() / PIVOT_GEAR_RATIO;
  }

  public void runRollers() {
    System.out.println("left roll go");
    leftRollers.set(1);

    rightRollers.setControl(followerRequest); // opposite direction as left rollers
  }

  public Command runIntake() {
    return Commands.runEnd(
        () -> {
          pivotMotor.setControl(pivotRequest.withPosition(PIVOT_DEPLOYED_POS));
          runRollers();
        },
        () -> {
          System.out.println("left roll stop");
          leftRollers.stopMotor();

          rightRollers.stopMotor();
          pivotMotor.setControl(
              pivotRequest.withPosition(PIVOT_RETRACTED_POS)); // hopefully this retracts the intake
        });
  }
  ;

  public Command temporaryRunIntake(double speed) {
    return Commands.runEnd(
        () -> {
          leftRollers.set(speed);
        },
        () -> {
          leftRollers.stopMotor();
        });
  }

  @Override
  public void simulationPeriodic() {
    if (leftRollerSim == null) {
      return;
    } else {
      leftRollerSim.setInput(leftRollers.getSimState().getMotorVoltage());
      leftRollerSim.update(updateSim);
    }
    if (rightRollerSim == null) {
      return;
    } else {
      rightRollerSim.setInput(rightRollers.getSimState().getMotorVoltage());
      rightRollerSim.update(updateSim);
    }
    if (pivotSimV2 == null) {
      return;
    } else {
      pivotSimV2.setInput(pivotMotor.getSimState().getMotorVoltage());
      pivotSimV2.update(updateSim);
    }

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
    pivotArm.setAngle(Units.radiansToDegrees(pivotSimV2.getAngleRads()));
  }

  @Override
  public void periodic() {
    leftRollerPub.set(leftRollers.getVelocity().getValueAsDouble());

    rightRollerPub.set(rightRollers.getVelocity().getValueAsDouble());

    if (RobotBase.isSimulation() && pivotSimV2 != null) {
      pivotArm.setAngle(Units.radiansToDegrees(pivotSimV2.getAngleRads()));
    } else if (!RobotBase.isSimulation()) {
      // Update arm angle from the actual motor position on the real robot.
      pivotArm.setAngle(getShoulderRotations() * 360);
    }
  }
}
