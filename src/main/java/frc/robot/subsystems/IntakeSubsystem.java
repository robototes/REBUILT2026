package frc.robot.subsystems;

import java.net.NetworkInterface;
import java.util.EnumSet;
import java.util.concurrent.Flow.Subscriber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Controls;

public class IntakeSubsystem extends SubsystemBase {
    private TalonFX pivotMotor;
    private double currentPivotPos;
    private final TalonFX rollers;
    final MotionMagicVoltage m_request1 = new MotionMagicVoltage(0);
    double speed;

    Mechanism2d mech1;
    Mechanism2d mech2;
    MechanismRoot2d root1;
    MechanismRoot2d root2;
    private BooleanPublisher ter;
    private DoublePublisher pivotPub;
    private final DoubleTopic pivotTopic;
    private DoubleSubscriber pivotSub;
    private final BooleanTopic rollerTopic;
    private final BooleanPublisher rollerPub;


    double pivotAngle;
    int connListenerHandle;
    int valueListenerHandle;
    int topicListenerHandle;

    private final double MAX_ANGULAR_RATE = Controls.MaxAngularRate;



    public IntakeSubsystem(Mechanism2d mech, boolean intakepivotEnabled, boolean intakerollersEnabled) {
        speed = 0;
        pivotAngle = 0;
        if (intakepivotEnabled) {
            pivotMotor = new TalonFX(Constants.HardwareConstants.kPivotMotorID);
        } else {
            pivotMotor = null;
        }
        if (intakerollersEnabled) {
            rollers = new TalonFX(Constants.HardwareConstants.kRollersID);
        } else {
            rollers = null;
        }
        // intake roller sim
        var nt = NetworkTableInstance.getDefault();
        this.rollerTopic = nt.getBooleanTopic("intake status/enabled");
        this.rollerPub = rollerTopic.publish();


        // pivot sim
        var nt2 = NetworkTableInstance.getDefault();
        this.pivotTopic = nt2.getDoubleTopic("pivot position/position");
        this.pivotPub = pivotTopic.publish();
        this.pivotSub = pivotTopic.subscribe(getCurrentPivotPos());
    }
    public void TalonFXConfigs() {
        var talonFXConfigs = new TalonFXConfiguration();
        var slot0Configs = talonFXConfigs.Slot0;
        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        TalonFXConfigurator rollersCfg = rollers.getConfigurator();
        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();

        // pivot configs

        slot0Configs.kS = 0.9;
        slot0Configs.kV = 4;
        slot0Configs.kA = 0;
        slot0Configs.kP = 40;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;
        slot0Configs.kG = 0.048; // change PID values during testing, these are placeholders from last year's robot

        var motionMagicConfigs = talonFXConfigs.MotionMagic; // these values i also guessed
        motionMagicConfigs.MotionMagicCruiseVelocity = 100;
        motionMagicConfigs.MotionMagicAcceleration = 200;
        motionMagicConfigs.MotionMagicJerk = 2000;

        // rollers configs

        talonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        talonFXConfiguration.Slot0.kS = 0;
        talonFXConfiguration.Slot0.kV = 0;
        talonFXConfiguration.Slot0.kA = 0;
        talonFXConfiguration.Slot0.kP = 0;
        talonFXConfiguration.Slot0.kI = 0;
        talonFXConfiguration.Slot0.kD = 0;
        talonFXConfiguration.Slot0.kG = 0; // change these values during testing as well
                                           // these values will be different from the pivot
                                           // thats why it's not reused in the same method

        pivotMotor.getConfigurator().apply(talonFXConfigs);
        rollersCfg.apply(talonFXConfiguration);
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
            pivotMotor.setControl(m_request1.withPosition(500)); // placeholder value, change during testing
            rollers.set(speed);
            },
            () -> {
            rollers.set(0);
            pivotMotor.setControl(m_request1.withPosition(0)); // hopefully this retracts the intake
            }
            );
        }


        @Override
        public void periodic() {
           rollerPub.set(intakeRunning());
           pivotPub.set(getCurrentPivotPos());
        }
}
