package frc.robot.Sensors;

import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.hardware.CANdle.*;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Constants;

public class LED_lights {
    private CANdle candle;

    // private String curAnimation = "default";
    private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(71);
    private final AddressableLEDBufferView[] sections = { buffer.createView(0, 70) };

    public LED_lights() {
        candle = new CANdle(Constants.OperatorConstants.LED_ID); // replace id with actual id

    }

    public Command animate(LEDPattern animation, String name) {
        return Commands.runOnce(() -> {
            updateLEDs(animation);
        })
                .withName("Animation: " + name);
    }

    private void helperMethod(){
        
    }

    private void updateLEDs(LEDPattern animation) {
        for (AddressableLEDBufferView section : sections) {
            animation.applyTo(section);
        }
        // candle.setLEDs
    }
}
