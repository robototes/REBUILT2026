package frc.robot.Sensors;

import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.CANdleConfigurator;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.hardware.CANdle.*;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.controls.SolidColor;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants;
import frc.robot.util.SoloScoringMode;

import java.util.function.Supplier;
import java.util.ArrayList;

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

    public Command showScoringModeColor(Supplier<SoloScoringMode> scoringMode) {
        return Commands.run(() -> {
            SoloScoringMode current_Mode = scoringMode.get();
            if (current_Mode == SoloScoringMode.INTAKE_IN_PROGRESS) {
                updateLEDs(intakeColor);
            } else if (current_Mode == SoloScoringMode.CLIMB_IN_PROGRESS) {
                updateLEDs(climbColor);
            } else if (current_Mode == SoloScoringMode.SHOOTING_IN_PROGRESS) {
                updateLEDs(shootingColor);
            } else if (current_Mode == SoloScoringMode.DEFAULT) {
                updateLEDs(defaultColor);
            }
        });
    }
}
