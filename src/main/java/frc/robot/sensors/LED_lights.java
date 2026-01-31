package frc.robot.sensors;

import static edu.wpi.first.units.Units.*;

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
  /** Constants goes here */
  private final int CAN_ID = 15;

  private final int END_INDEX = 7;
  private final RainbowAnimation m_slot0Animation = new RainbowAnimation(0, END_INDEX);
  private CANdle candle = new CANdle(CAN_ID);

  public RGBWColor defaultColor = new RGBWColor(255, 0, 0); // red
  public RGBWColor intakeColor = new RGBWColor(255, 255, 0); // yellow
  public RGBWColor outtakeColor = new RGBWColor(0, 255, 0); // green
  public RGBWColor climbColor = new RGBWColor(0, 0, 255); // blue
  public RGBWColor offColor = new RGBWColor(0, 0, 0); // off

  private boolean rainbowOn = false;

  public LED_lights() {
    setRainbowAnimation(0, 1, AnimationDirectionValue.Forward, Hertz.of(100));
  }

  /**
   * Configures a {@link RainbowAnimation} on the specified animation slot.
   *
   * <p>This method sets the {@code slot}, {@code brightness}, {@code direction}, and {@code frame
   * rate} for the rainbow animation.
   *
   * @param slot the animation slot to run the rainbow animation on
   * @param brightness the brightness level of the LEDs (typically 0–255)
   * @param direction the direction the rainbow animation moves
   * @param frameRate the update frequency of the animation
   */
  public void setRainbowAnimation(
      int slot, int brightness, AnimationDirectionValue direction, Frequency frameRate) {

    m_slot0Animation
        .withSlot(slot)
        .withBrightness(brightness)
        .withDirection(direction)
        .withFrameRate(frameRate);
  }

  /**
   * Sets Leds at specified {@link frc.robot.sensors.LED_lights#CAN_ID id} to color
   *
   * @param color
   */
  public Command updateLEDs(RGBWColor color) {
    return Commands.run(
            () -> {
              SolidColor solid = new SolidColor(0, END_INDEX);
              solid.withColor(color);
              candle.setControl(solid);
            })
        .withName("Set leds to specified color");
  }

  /* to be used in the future */
  public Command LED_Mode(Supplier<LEDMode> mode) {
    return Commands.run(
        () -> {
          LEDMode currentMode = mode.get();
          if (currentMode == LEDMode.DEFAULT) {
            updateLEDs(defaultColor);
          } else if (currentMode == LEDMode.OUTTAKE_IN_PROGRESS) {
            updateLEDs(outtakeColor);
          } else if (currentMode == LEDMode.INTAKE_IN_PROGRESS) {
            updateLEDs(intakeColor);
          } else if (currentMode == LEDMode.CLIMB_IN_PROGRESS) {
            updateLEDs(climbColor);
          }
        });
  }

  /**
   * Alternates the LED strip between two colors once per second.
   *
   * <p>The active color is determined by the system time: {@code colorA} is shown on even-numbered
   * seconds and {@code colorB} is shown on odd-numbered seconds.
   *
   * @param colorA the color displayed on even seconds
   * @param colorB the color displayed on odd seconds
   * @return a {@link Command} that continuously alternates LED colors
   */
  public Command alternateColors(RGBWColor colorA, RGBWColor colorB) {
    return new RunCommand(
        () -> {
          long seconds = System.currentTimeMillis() / 1000;
          if (seconds % 2 == 0) {
            updateLEDs(colorA);
          } else {
            updateLEDs(colorB);
          }
        },
        this);
  }

  /**
   * Toggles the rainbow animation on or off.
   *
   * <p>If the rainbow animation is not currently running, this command starts it. If it is already
   * running, the animation is stopped.
   *
   * @return a {@link Command} that toggles the rainbow animation state
   */
  public Command toggleRainbow() {
    return new InstantCommand(
        () -> {
          if (!rainbowOn) {
            candle.setControl(m_slot0Animation);
          } else {
            candle.setControl(new EmptyAnimation(0));
          }
          rainbowOn = !rainbowOn;
        },
        this);
  }

  /**
   * Starts the rainbow animation.
   *
   * @return a {@link Command} that starts the rainbow animation
   */
  public Command startRainbow() {
    return new InstantCommand(() -> candle.setControl(m_slot0Animation), this);
  }

  /**
   * Stops the rainbow animation.
   *
   * @return a {@link Command} that stops the currently running rainbow animation
   */
  public Command stopRainbow() {
    return new InstantCommand(() -> candle.setControl(new EmptyAnimation(0)), this);
  }
}
