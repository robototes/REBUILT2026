package frc.robot.subsystems;



import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
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
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Controls;
import frc.robot.Hardware;

public class IntakeSubsystem extends SubsystemBase {
    private TalonFX pivotMotor;
    private double currentPivotPos;
    private final TalonFX rollers;
    final MotionMagicVoltage m_request1 = new MotionMagicVoltage(0);
    double speed;


    private DoublePublisher pivotPub;
    private final DoubleTopic pivotTopic;
    private DoubleSubscriber pivotSub;
    private final BooleanTopic rollerTopic;
    private final BooleanPublisher rollerPub;

    private final FlywheelSim rollerSim;
    private final ElevatorSim pivotSim;
    private final NetworkTableEntry table1;
    private final NetworkTableEntry table2;
    LinearSystem rollerSystem = LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), 1, 1);
    LinearSystem pivotSystem = LinearSystemId.createElevatorSystem(DCMotor.getKrakenX60(1), 40, 1, 1);

    double pivotAngle;



    public IntakeSubsystem(Mechanism2d mech, boolean intakepivotEnabled, boolean intakerollersEnabled) {
        speed = 0;
        pivotAngle = 0;
        if (intakepivotEnabled) {
            pivotMotor = new TalonFX(Hardware.PIVOT_MOTOR_ID);
        } else {
            pivotMotor = null;
        }
        if (intakerollersEnabled) {
            rollers = new TalonFX(Hardware.INTAKE_MOTOR_ID);
        } else {
            rollers = null;
        }
        // intake roller sim

        if (RobotBase.isSimulation()) {
            rollerSim = new FlywheelSim(rollerSystem, DCMotor.getKrakenX60(1), 0.0);
            pivotSim = new ElevatorSim(pivotSystem, DCMotor.getKrakenX60(1), 0, 5, intakepivotEnabled, 1, 0.0);
        } else {
            rollerSim = null;
            pivotSim = null;
        }

        table1 = NetworkTableInstance.getDefault().getTable("Flywheel").getEntry("SimRPM");
        table2 = NetworkTableInstance.getDefault().getTable("Pivot").getEntry("SimRPM");

        var nt = NetworkTableInstance.getDefault();
        this.rollerTopic = nt.getBooleanTopic("intake status/enabled");
        this.rollerPub = rollerTopic.publish();


        // pivot sim
        var nt2 = NetworkTableInstance.getDefault();
        this.pivotTopic = nt2.getDoubleTopic("pivot position/position");
        this.pivotPub = pivotTopic.publish();
        this.pivotSub = pivotTopic.subscribe(getCurrentPivotPos());
    }



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

        var motionMagicConfigs = talonFXConfigs1.MotionMagic; // these values i also guessed
        motionMagicConfigs.MotionMagicCruiseVelocity = 100;
        motionMagicConfigs.MotionMagicAcceleration = 200;
        motionMagicConfigs.MotionMagicJerk = 2000;

        pivotMotor.getConfigurator().apply(talonFXConfigs1);
        pivotMotor.getConfigurator().apply(motionMagicConfigs);
    }
        // rollers configs
    public void TalonFXRollerConfigs() {
        var talonFXConfigs2 = new TalonFXConfiguration();
        TalonFXConfigurator rollersCfg = rollers.getConfigurator();
        talonFXConfigs2.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        talonFXConfigs2.Slot0.kS = 0;
        talonFXConfigs2.Slot0.kV = 0;
        talonFXConfigs2.Slot0.kA = 0;
        talonFXConfigs2.Slot0.kP = 0;
        talonFXConfigs2.Slot0.kI = 0;
        talonFXConfigs2.Slot0.kD = 0;
        talonFXConfigs2.Slot0.kG = 0; /* change these values during testing as well
                                         these values will be different from the pivot
                                         thats why it's not reused in the same method */


        rollersCfg.apply(talonFXConfigs2);

    }
        public double getCurrentPivotPos() {
            currentPivotPos = pivotMotor.getPosition().getValueAsDouble();
            return currentPivotPos;
        }
        public boolean intakeRunning() {
            if (Math.abs(rollers.get()) > 0.1) {
                return true;
            } else {
                return false;
            }
        }


    public Command runIntake(double speed) {
        return Commands.runEnd(
            () -> {
            pivotMotor.setControl(m_request1.withPosition(1000)); // placeholder value, change during testing
            rollers.set(speed);
            },
            () -> {
            rollers.set(0);
            pivotMotor.setControl(m_request1.withPosition(0)); // hopefully this retracts the intake
            }
            );
        }


        @Override
        public void simulationPeriodic() {
            rollerPub.set(intakeRunning());
            pivotPub.set(getCurrentPivotPos());

            rollerSim.setInput(rollers.getSimState().getMotorVoltage());
            rollerSim.update(0.020);
            pivotSim.setInput(pivotMotor.getSimState().getMotorVoltage());
            pivotSim.update(0.020);
            double rpm1 = rollerSim.getAngularVelocityRPM();
            table1.setDouble(rpm1);
            double rpm2 = pivotSim.getCurrentDrawAmps();
            table2.setDouble(rpm2);


            // RoboRioSim.setVInVoltage(
            // BatterySim.calculateDefaultBatteryLoadedVoltage(rollerSim.getCurrentDrawAmps())
            // );
        }
}

// max was here


// no he wasn't i did most of the work (pete)
