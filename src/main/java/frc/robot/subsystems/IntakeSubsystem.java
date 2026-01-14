package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private TalonFX pivotMotor;
    final MotionMagicVoltage m_request1 = new MotionMagicVoltage(0);
    
    public IntakeSubsystem() {
        pivotMotor = new TalonFX(Constants.MotorConstants.PivotMotorID);

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

        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 100;
        motionMagicConfigs.MotionMagicAcceleration = 200;
        motionMagicConfigs.MotionMagicJerk = 2000;

        pivotMotor.getConfigurator().apply(talonFXConfigs);
        }
    public Command runPivot() {
        return run( () -> {
            pivotMotor.setControl(m_request1.withPosition(500));
        });
    }
    
    
}
