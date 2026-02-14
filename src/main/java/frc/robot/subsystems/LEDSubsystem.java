package frc.robot.subsystems;

import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;

public class LEDSubsystem extends SubsystemBase {
  /** Constants goes here */
  /** CAN ID for the CANdle LED controller. */
  private static final int CAN_ID = Hardware.CANdle_ID;

  /**
   * Last LED index in the strip (inclusive).
   *
   * <p>Currently set for testing with a single module (8 LEDs total), which are zero-indexed (0–7).
   */
  private static final int END_INDEX = 7;

  /** CTRE CANdle LED controller instance. */
  private final CANdle candle = new CANdle(CAN_ID);

  /**
   * Solid color control object.
   *
   * <p>Configured to control LEDs from index 0 through {@link #END_INDEX}.
   */
  private final SolidColor solid = new SolidColor(0, END_INDEX);

  /**
   * Preconfigured rainbow animation assigned to animation slot 0.
   *
   * <p>Applies to LEDs from index 0 through {@link #END_INDEX}.
   */
  private final RainbowAnimation rainbowAnimation = new RainbowAnimation(0, END_INDEX);

  /** Empty animation used to clear animation slot 0 and stop active animations. */
  private final EmptyAnimation emptyAnimation = new EmptyAnimation(0);

  /** Default robot LED color (red). */
  public static final RGBWColor DEFAULT_COLOR = new RGBWColor(255, 0, 0);

  /** LED color used while intaking (blue). */
  public static final RGBWColor INTAKE_COLOR = new RGBWColor(0, 0, 255);

  /** LED color used while outtaking (green). */
  public static final RGBWColor OUTTAKE_COLOR = new RGBWColor(0, 255, 0);

  /** LED color used during climb mode (cyan). */
  public static final RGBWColor CLIMB_COLOR = new RGBWColor(0, 255, 255);

  /** LED color representing LEDs off. */
  public static final RGBWColor OFF_COLOR = new RGBWColor(0, 0, 0);

  public LEDSubsystem() {
    setRainbowAnimation(0, 0.1, AnimationDirectionValue.Forward, Units.Hertz.of(100));
  }

  /**
   * Configures a {@link RainbowAnimation} on the specified animation slot.
   *
   * <p>This method sets the {@code slot}, {@code brightness}, {@code direction}, and {@code frame
   * rate} for the rainbow animation.
   *
   * @param slot the animation slot to run the rainbow animation on
   * @param brightness the brightness level of the LEDs (0.0-1.0)
   * @param direction the direction the rainbow animation moves
   * @param frameRate the update frequency of the animation
   */
  public void setRainbowAnimation(
      int slot, double brightness, AnimationDirectionValue direction, Frequency frameRate) {

    rainbowAnimation
        .withSlot(slot)
        .withBrightness(brightness)
        .withDirection(direction)
        .withFrameRate(frameRate);
  }

  /**
   * Sets the physical CANdle LED controller to a solid color.
   *
   * <p>This directly updates the hardware output and should generally only be called internally by
   * commands that manage LED state.
   *
   * @param color the {@link RGBWColor} to apply to the LED strip
   */
  public void setHardwareColor(RGBWColor color) {
    solid.withColor(color);
    candle.setControl(solid);
  }

  /**
   * Creates a command that sets the LEDs to the specified color while scheduled.
   *
   * <p>This command requires the {@link LEDSubsystem}, ensuring no other LED commands can run
   * simultaneously.
   *
   * <p>This command uses the {@link #setHardwareColor} function
   *
   * @param color the {@link RGBWColor} to display
   * @return a {@link Command} that sets the LEDs to the given color once
   */
  public Command setLEDsCommand(RGBWColor color) {
    return Commands.runOnce(() -> setHardwareColor(color), this)
        .withName("SetLEDs"); // Good practice to name commands
  }

  /**
   * Alternates the LED strip between two colors, switching every 0.5 seconds.
   *
   * <p>This command will run indefinitely, repeatedly showing {@code colorA} for 0.5s, then {@code
   * colorB} for 0.5s, creating a flashing effect.
   *
   * @param colorA the first color in the sequence
   * @param colorB the second color in the sequence
   * @return a {@link Command} that continuously alternates LED colors
   */
  public Command alternateColors(RGBWColor colorA, RGBWColor colorB, double interval) {
    return Commands.sequence(
            Commands.runOnce(() -> setHardwareColor(colorA), this),
            Commands.waitSeconds(interval),
            Commands.runOnce(() -> setHardwareColor(colorB), this),
            Commands.waitSeconds(interval))
        .repeatedly();
  }

  /**
   * Creates a command that cycles through an array of colors, switching to the next color every
   * {@code interval} seconds.
   *
   * @param colors an array of {@link RGBWColor} objects to cycle through
   * @param interval the time in seconds to wait before switching to the next color
   * @return a {@link Command} that cycles through the given colors
   */
  public Command cycleColors(RGBWColor[] colors, double interval) {
    switch (colors.length) {
      case 0:
        throw new IllegalArgumentException("Colors array must contain at least one color");

      case 1:
        return setLEDsCommand(colors[0]);

      case 2:
        return alternateColors(colors[0], colors[1], interval);

      default:
        Command sequenceBuilder = Commands.sequence();

        for (RGBWColor color : colors) {
          sequenceBuilder =
              sequenceBuilder
                  .andThen(Commands.runOnce(() -> setHardwareColor(color), this))
                  .andThen(Commands.waitSeconds(interval));
        }

        return sequenceBuilder.repeatedly();
    }
  }

  /**
   * Enables or disables the rainbow animation on the CANdle.
   *
   * <p>If {@code enabled} is true, the preconfigured rainbow animation is applied. If false, the
   * active animation is cleared.
   *
   * @param enabled true to start the rainbow animation, false to stop it
   */
  public void setRainbowEnabled(boolean enabled) {
    if (enabled) {
      candle.setControl(rainbowAnimation);
    } else {
      candle.setControl(emptyAnimation);
    }
  }
}
