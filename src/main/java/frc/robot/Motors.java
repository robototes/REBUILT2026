package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.generated.AlphaTunerConstants;
import frc.robot.util.robotType.RobotType;

public class Motors {
    private final TalonFX motor;
    public Motors(int id, MotorConfigs configs) {
        motor = new TalonFX(id, (RobotType.isAlpha() ? AlphaTunerConstants.kCANBus : CANBus.roboRIO()));
        motor.clearStickyFaults();
        var talonFXConfigs = new TalonFXConfiguration();
        talonFXConfigs.MotorOutput.NeutralMode = configs.neutralMode; // KEEP TS AT COAST
        talonFXConfigs.MotorOutput.Inverted = configs.inverted;

        talonFXConfigs.CurrentLimits.StatorCurrentLimit = configs.statorCurrentLimit;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = configs.supplyCurrentLimit;
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = configs.statorCurrentLimitEnable;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = configs.supplyCurrentLimitEnable;

        talonFXConfigs.Slot0.kP = configs.kP;
        talonFXConfigs.Slot0.kS = configs.kS;
        talonFXConfigs.Slot0.kA = configs.kA;

        motor.getConfigurator().apply(talonFXConfigs);
    }
    public record MotorConfigs(double kP, double kS, double kA, NeutralModeValue neutralMode, InvertedValue inverted, double statorCurrentLimit, double supplyCurrentLimit, boolean statorCurrentLimitEnable, boolean supplyCurrentLimitEnable) {}
}
