package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX rollers;

    public IntakeSubsystem() {
        rollers = new TalonFX(Constants.HardwareConstants.kRollersID);
    }

    public void rollerConfig() {
        TalonFXConfigurator rollersCfg = rollers.getConfigurator();

        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();

        talonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        rollersCfg.apply(talonFXConfiguration);
    }

    public Command spinRollers(double speed) {
        return Commands.runOnce(
            () -> {
                rollers.set(speed);
    });}

    public Command stopRollers() {
        return Commands.runOnce(
            () -> {
                rollers.set(0);
    });}
}
