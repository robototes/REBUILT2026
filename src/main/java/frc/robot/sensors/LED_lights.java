package frc.robot.sensors;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CANdleConfigurator;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LEDMode;
import java.util.function.Supplier;

public class LED_lights extends SubsystemBase {
  private CANdle candle = new CANdle(15);
  private CANdleConfigurator candleConfigurator;

  public RGBWColor defaultColor = new RGBWColor(255, 0, 0); // red
  public RGBWColor intakeColor = new RGBWColor(255, 255, 0); // yellow
  public RGBWColor outtakeColor = new RGBWColor(0, 255, 0); // green
  public RGBWColor climbColor = new RGBWColor(0, 0, 255); // blue
  public RGBWColor offColor = new RGBWColor(0, 0, 0); // should be off? maybe?

  private final RainbowAnimation m_slot0Animation = new RainbowAnimation(0, 7);

  private boolean rainbowOn = false;

  public LED_lights() {
    candleConfigurator = candle.getConfigurator();
    setRainbowAnimation(0, 1, AnimationDirectionValue.Forward, Hertz.of(100));
  }

  public void setRainbowAnimation(
      int slot, int brightness, AnimationDirectionValue direction, Frequency frameRate) {
    m_slot0Animation
        .withSlot(slot)
        .withBrightness(brightness)
        .withDirection(direction)
        .withFrameRate(frameRate);
  }

  /* to be used in the future */
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

  /**
   * This function is used to turn off the leds (color is 0,0,0)
   *
   * @return
   */
  public Command turnOffLEDS() {
    return Commands.run(
        () -> {
          updateLEDs(offColor);
        });
  }

  /**
   * This function is used to show the intake color (255,255,0)
   *
   * @return
   */
  public Command showIntakeColor() {
    return Commands.run(
        () -> {
          updateLEDs(intakeColor);
        });
  }

  /**
   * This function is used to show the climb color (0,0,255)
   *
   * @return
   */
  public Command showClimbColor() {
    return Commands.run(
        () -> {
          updateLEDs(climbColor);
        });
  }

  /**
   * This function is used to show the outtake color (0,255,0)
   *
   * @return
   */
  public Command showOuttakeColor() {
    return Commands.run(
        () -> {
          updateLEDs(outtakeColor);
        });
  }

  /**
   * This function is ued to show the default color (255,0,0)
   *
   * @return
   */
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

  private void updateLEDs(RGBWColor color) {
    SolidColor solid = new SolidColor(0, 71);
    solid.withColor(color);
    candle.setControl(solid);
  }

  public Command toggleRainbow() {
    return new InstantCommand(
        () -> {
          if (!rainbowOn) {
            // Start rainbow
            candle.setControl(m_slot0Animation);
          } else {
            // Stop rainbow
            candle.setControl(new EmptyAnimation(0));
          }
          rainbowOn = !rainbowOn; // flip state
        },
        this);
  }

  public Command startRainbow() {
    return new InstantCommand(() -> candle.setControl(m_slot0Animation), this);
  }

  public Command stopRainbow() {
    return new InstantCommand(() -> candle.setControl(new EmptyAnimation(0)), this);
  }
}
