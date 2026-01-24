package frc.robot.sensors;

import com.ctre.phoenix6.configs.CANdleConfigurator;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LEDMode;
import java.util.function.Supplier;

public class LED_lights extends SubsystemBase {
  public CANdle candle;
  private CANdleConfigurator candleConfigurator;

  public RGBWColor defaultColor = new RGBWColor(255, 0, 0); // red
  public RGBWColor intakeColor = new RGBWColor(255, 255, 0); // yellow
  public RGBWColor outtakeColor = new RGBWColor(0, 255, 0); // green
  public RGBWColor climbColor = new RGBWColor(0, 0, 255); // blue

  public LED_lights() {
    candle = new CANdle(15); // replace id with actual id
    candleConfigurator = candle.getConfigurator();
  }

  private void updateLEDs(RGBWColor color) {
    SolidColor solid = new SolidColor(0, 71);
    solid.withColor(color);
    candle.setControl(solid);
  }

  public Command LED_Mode(Supplier<LEDMode> mode) {
    return Commands.run(
        () -> {
          LEDMode currentMode = mode.get();
          if (currentMode == LEDMode.DEFAULT) {
            showDefaultColor();
          } else if (currentMode == LEDMode.OUTTAKE_IN_PROGRESS) {
            showOuttakeColor();
          } else if (currentMode == LEDMode.INTAKE_IN_PROGRESS) {
            showIntakeColor();
          } else if (currentMode == LEDMode.CLIMB_IN_PROGRESS) {
            showClimbColor();
          }
        });
  }

  public Command showIntakeColor() {
    return Commands.run(
        () -> {
          updateLEDs(intakeColor);
        });
  }

  public Command showClimbColor() {
    return Commands.run(
        () -> {
          updateLEDs(climbColor);
        });
  }

  public Command showOuttakeColor() {
    return Commands.run(
        () -> {
          updateLEDs(outtakeColor);
        });
  }

  public Command showDefaultColor() {
    return Commands.run(
        () -> {
          updateLEDs(defaultColor);
        });
  }

  public Command alternateColors(RGBWColor colorA, RGBWColor colorB) {
    return new RunCommand(
        () -> {
          // Divide system time by 1000 to get seconds, % 2 to alternate
          long seconds = System.currentTimeMillis() / 1000;
          if (seconds % 2 == 0) {
            updateLEDs(colorA); // colorA shows on even seconds
          } else {
            updateLEDs(colorB); // colorB shows on odd seconds
          }
        },
        this);
  }
}
