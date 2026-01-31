package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
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
import frc.robot.Controls;
import frc.robot.Hardware;
import frc.robot.Robot;

public class IntakeSubsystem extends SubsystemBase {
    private TalonFX pivotMotor;
    private double currentPivotPos;
    private final TalonFX leftRollers; // one
    private final TalonFX rightRollers; // two
    final MotionMagicVoltage pivot_request1 = new MotionMagicVoltage(0);
    final MotionMagicVelocityVoltage roller_request1 = new MotionMagicVelocityVoltage(0);
    double speed;

    private DoublePublisher pivotPub;
    private final DoubleTopic pivotTopic;
    private DoubleSubscriber pivotSub;
    private final DoubleTopic leftRollerTopic;
    private final DoubleTopic rightRollerTopic;
    private final DoublePublisher leftRollerPub;
    private final DoublePublisher rightRollerPub;
    private final FlywheelSim leftRollerSim;
    private final FlywheelSim rightRollerSim;
    private final ElevatorSim pivotSim;
    private final SingleJointedArmSim pivotSimV2;
    LinearSystem rollerSystem = LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), 1, 1);
    LinearSystem pivotSystem = LinearSystemId.createElevatorSystem(DCMotor.getKrakenX60(1), 40, 1, 1);


    double pivotAngle;
    private final double PIVOT_GEAR_RATIO =
    (16.0/60.0) * (34.0/68.0) * (18.0/40.0); // this is a complete guess that i took from the demo branch

    Mechanism2d mech = new Mechanism2d(1, 1);
    MechanismRoot2d pivotShoulder = mech.getRoot("Shoulder", 0.178/2.0, 0.0);
    MechanismLigament2d pivotArm = pivotShoulder.append(
        new MechanismLigament2d("pivot", Units.inchesToMeters(10.919), 2));


    public IntakeSubsystem(boolean intakepivotEnabled, boolean intakerollersEnabled) {
        speed = 0;
        pivotAngle = 0;

        if (intakepivotEnabled) {
            pivotMotor = new TalonFX(Hardware.PIVOT_MOTOR_ID);

        } else {
            pivotMotor = null;
        }
        if (intakerollersEnabled) {
            leftRollers = new TalonFX(Hardware.INTAKE_ONE_MOTOR_ID);
            rightRollers = new TalonFX(Hardware.INTAKE_TWO_MOTOR_ID);
        } else {
            leftRollers = null;
            rightRollers = null;
        }
        // intake roller sim

        if (RobotBase.isSimulation()) {
            leftRollerSim =
                new FlywheelSim(rollerSystem, DCMotor.getKrakenX60(1));
            rightRollerSim =
                new FlywheelSim(rollerSystem, DCMotor.getKrakenX60(1));
            pivotSim =
                new ElevatorSim(pivotSystem, DCMotor.getKrakenX60(1), 0, 5, intakepivotEnabled, 1);

            pivotSimV2 = new SingleJointedArmSim(DCMotor.getKrakenX60(1)
            , 0.5
            , 0.01
            , 1
            , Units.degreesToRadians(-90)
            , Units.degreesToRadians(90)
            , false
            , Units.degreesToRadians(0)
            );
        } else {
            leftRollerSim = null;
            rightRollerSim = null;
            pivotSim = null;
            pivotSimV2 = null;
        }


        var nt = NetworkTableInstance.getDefault();
        this.leftRollerTopic = nt.getDoubleTopic("left intake status/speed in RPM");
        this.leftRollerPub = leftRollerTopic.publish();

        this.rightRollerTopic = nt.getDoubleTopic("right intake status/speed in RPM");
        this.rightRollerPub = rightRollerTopic.publish();


        // old pivot sim
        // TODO: remove this later
        var nt2 = NetworkTableInstance.getDefault();
        this.pivotTopic = nt2.getDoubleTopic("pivot position/position");
        this.pivotPub = pivotTopic.publish();
        this.pivotSub = pivotTopic.subscribe(0);

        // new pivot sim
            SmartDashboard.putData("Intake Arm", mech);
            // angle is a guess cuz idk how to use cad tools

    }
    // configs
    public void TalonFXPivotConfigs() {
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
        pivotMotionMagicConfigs.MotionMagicCruiseVelocity = 0;
        pivotMotionMagicConfigs.MotionMagicAcceleration = 200;
        pivotMotor.getConfigurator().apply(talonFXConfigs1);
        pivotMotor.getConfigurator().apply(pivotMotionMagicConfigs);
    }
        // rollers configs
    public void TalonFXRollerConfigs() {
        var talonFXConfigs2 = new TalonFXConfiguration();
        TalonFXConfigurator rollersCfg = leftRollers.getConfigurator();
        TalonFXConfigurator rollersCFG2 = rightRollers.getConfigurator();
        talonFXConfigs2.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonFXConfigs2.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        rollersCfg.apply(talonFXConfigs2);
        rollersCFG2.apply(talonFXConfigs2);
    }
        // public double getCurrentPivotPos() {
        //     currentPivotPos = pivotMotor.getPosition().getValueAsDouble();
        //     return currentPivotPos;
        // }
        public double getShoulderRotations() {
            return pivotMotor.getPosition().getValueAsDouble() * PIVOT_GEAR_RATIO;
        }

    public Command runIntake(double speed) {
        return Commands.runEnd(
            () -> {
            pivotMotor.setControl(pivot_request1.withPosition(1000)); // placeholder value, change during testing
            leftRollers.set(speed); // placeholder
            rightRollers.setControl(new Follower(Hardware.INTAKE_ONE_MOTOR_ID, MotorAlignmentValue.Opposed)); // opposite direction as left rollers
            },
            () -> {
            leftRollers.stopMotor();
            rightRollers.stopMotor();
            pivotMotor.setControl(pivot_request1.withPosition(0)); // hopefully this retracts the intake
            }
            );
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
                BatterySim.calculateDefaultBatteryLoadedVoltage(leftRollerSim.getCurrentDrawAmps()));

            leftRollers.getSimState().setRotorVelocity(
                RadiansPerSecond.of(leftRollerSim.getAngularVelocityRPM()));
            rightRollers.getSimState().setRotorVelocity(
                RadiansPerSecond.of(rightRollerSim.getAngularVelocityRPM()));
            pivotMotor.getSimState().setRawRotorPosition(
                Radians.of(pivotSimV2.getAngleRads() / PIVOT_GEAR_RATIO).in(Rotations));
            pivotMotor.getSimState().setRotorVelocity(
                RadiansPerSecond.of(pivotSimV2.getVelocityRadPerSec() / PIVOT_GEAR_RATIO).in(RotationsPerSecond));


        }
        @Override
        public void periodic() {
            leftRollerPub.set(rightRollers.getVelocity().getValueAsDouble());
            rightRollerPub.set(rightRollers.getVelocity().getValueAsDouble());

            pivotArm.setAngle(Rotation2d.fromRotations(getShoulderRotations()));
}
}
