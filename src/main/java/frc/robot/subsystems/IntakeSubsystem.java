package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private TalonFX pivotMotor;
    private final TalonFX rollers;
    final MotionMagicVoltage m_request1 = new MotionMagicVoltage(0);
    double speed;
    
    public IntakeSubsystem() {
        pivotMotor = new TalonFX(Constants.HardwareConstants.PivotMotorID);
        rollers = new TalonFX(Constants.HardwareConstants.kRollersID);
        speed = 0;
        
        Mechanism2d intakeSim = new Mechanism2d(3, 3);
        MechanismRoot2d root = intakeSim.getRoot("Intake Test", 2, 0);
    }
    public void TalonFXConfigs() {
        var talonFXConfigs = new TalonFXConfiguration();
        var slot0Configs = talonFXConfigs.Slot0;
        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        slot0Configs.kS = 0.9;
        slot0Configs.kV = 4;
        slot0Configs.kA = 0;
        slot0Configs.kP = 40;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;
        slot0Configs.kG = 0.048;
        // change PID values during testing, these are placeholders from last year's robot

        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 100;
        motionMagicConfigs.MotionMagicAcceleration = 200;
        motionMagicConfigs.MotionMagicJerk = 2000;

        pivotMotor.getConfigurator().apply(talonFXConfigs);
        }
    public Command runPivot() {
        return run( () -> {
            pivotMotor.setControl(m_request1.withPosition(500)); // placeholder position value, change during testing
        });
    }
    
    public void rollerConfig() {
        TalonFXConfigurator rollersCfg = rollers.getConfigurator();

        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();

        talonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        rollersCfg.apply(talonFXConfiguration);
    }
    public Command runIntake(double speed) {
        return Commands.runEnd(
            () -> {
            pivotMotor.setControl(m_request1.withPosition(500));
            rollers.set(speed);
            },
            () -> {
            rollers.set(0);
            }
            );
        }


    public Command spinRollers(double speed) {
        return Commands.runOnce(
            () -> {
                rollers.set(speed);
            });
        }

    public Command stopRollers() {
        return Commands.runOnce(
            () -> {
                rollers.set(0);
    });}
}

