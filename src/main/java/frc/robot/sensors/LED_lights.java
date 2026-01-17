package frc.robot.sensors;

import com.ctre.phoenix6.configs.CANdleConfigurator;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.controls.SolidColor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Constants;
import frc.robot.util.LEDMode;

import java.util.function.Supplier;

public class LED_lights {
    private CANdle candle;
    private CANdleConfigurator candleConfigurator;

    private RGBWColor defaultColor = new RGBWColor(255, 0, 0);
    private RGBWColor intakeColor = new RGBWColor(255, 255, 0);
    private RGBWColor outtakeColor = new RGBWColor(0, 255, 0);
    private RGBWColor climbColor = new RGBWColor(0, 0, 255);

    public LED_lights() {
        candle = new CANdle(7); // replace id with actual id
        candleConfigurator = candle.getConfigurator();

    }

    private void updateLEDs(RGBWColor color) {
        SolidColor solid = new SolidColor(0, 71);
        solid.withColor(color);
        candle.setControl(solid);
    }

    public Command LED_Mode(Supplier<LEDMode> mode){
        return Commands.run(()->{
            LEDMode currentMode = mode.get();
            if (currentMode == LEDMode.DEFAULT){
                showDefaultColor();
            } else if (currentMode == LEDMode.OUTTAKE_IN_PROGRESS){
                showOuttakeColor();
            } else if (currentMode == LEDMode.INTAKE_IN_PROGRESS){
                showIntakeColor();
            } else if (currentMode == LEDMode.CLIMB_IN_PROGRESS){
                showClimbColor();
            }
        });
    }

    public Command showIntakeColor(){
        return Commands.run(() -> {
            updateLEDs(intakeColor);
        });
    }
    public Command showClimbColor(){
        return Commands.run(() -> {
            updateLEDs(climbColor);
        });
    }
    public Command showOuttakeColor(){
        return Commands.run(() -> {
            updateLEDs(outtakeColor);
        });
    }
    public Command showDefaultColor(){
        return Commands.run(() -> {
            updateLEDs(defaultColor);
        });
    }
}
