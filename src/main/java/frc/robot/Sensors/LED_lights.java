package frc.robot.Sensors;

import com.ctre.phoenix6.configs.CANdleConfigurator;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.controls.SolidColor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants;
import frc.robot.util.SoloScoringMode;

import java.util.function.Supplier;

public class LED_lights {
    private CANdle candle;
    private CANdleConfigurator candleConfigurator;

    private RGBWColor defaultColor = new RGBWColor(255, 0, 0);
    private RGBWColor intakeColor = new RGBWColor(255, 255, 0);
    private RGBWColor shootingColor = new RGBWColor(0, 255, 0);
    private RGBWColor climbColor = new RGBWColor(0, 0, 255);
    private CommandXboxController m_driverController;

    public LED_lights() {
        candle = new CANdle(Constants.OperatorConstants.LED_ID); // replace id with actual id
        candleConfigurator = candle.getConfigurator();

    }

    private void updateLEDs(RGBWColor color) {
        SolidColor solid = new SolidColor(0, 71);
        solid.withColor(color);
        candle.setControl(solid);
    }

    public void customBindings(){
        m_driverController.a().onTrue(showIntakeColor());
        m_driverController.b().onTrue(showClimbColor());
        m_driverController.x().onTrue(showShootingColor());
        m_driverController.y().onTrue(showDefaultColor());
    }
    public Command showIntakeColor(){
        return Commands.run(() -> {
            updateLeds(intakeColor);
        }
    }
    public Command showClimbColor(){
        return Commands.run(() -> {
            updateLeds(climbColor);
        }
    }
    public Command showShootingColor(){
        return Commands.run(() -> {
            updateLeds(shootingColor);
        }
    }
    public Command showDefaultColor(){
        return Commands.run(() -> {
            updateLeds(defaultColor);
        }
    }
}
