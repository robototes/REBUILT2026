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
  /** CAN ID for the CANdle LED controller. */
  private static final int CAN_ID = Hardware.CANdle_ID;

  /**
   * Last LED index in the strip (inclusive).
   *
   * <p>Currently set for testing with a single module (8 LEDs total), which are zero-indexed (0–7).
   */
  private static final int END_INDEX = 7;

  /** Default brightness applied to all SolidColor operations (0.0–1.0). */
  public static final double DEFAULT_BRIGHTNESS = 0.25;

  /** Instance of the CTRE CANdle LED controller used to control the LED strip. */
  private final CANdle candle = new CANdle(CAN_ID);

  /**
   * Solid color control object.
   *
   * <p>Configured to control LEDs from index 0 through {@link #END_INDEX}. Use this object to
   * directly set a single color across the strip.
   */
  private final SolidColor solid = new SolidColor(0, END_INDEX);

  /**
   * Preconfigured rainbow animation assigned to animation slot 0.
   *
   * <p>Applies to LEDs from index 0 through {@link #END_INDEX}. This can be enabled or disabled to
   * create animated rainbow effects.
   */
  private final RainbowAnimation rainbowAnimation = new RainbowAnimation(0, END_INDEX);

  /** Empty animation used to clear animation slot 0 and stop active animations. */
  private final EmptyAnimation emptyAnimation = new EmptyAnimation(0);

  /** Default robot LED color (Red). */
  public static final RGBWColor DEFAULT_COLOR = new RGBWColor(255, 0, 0);

  /** LED color used while intaking (Blue). */
  public static final RGBWColor INTAKE_COLOR = new RGBWColor(0, 0, 255);

  /** LED color used while outtaking (Green). */
  public static final RGBWColor OUTTAKE_COLOR = new RGBWColor(0, 255, 0);

  /** LED color used during climb mode (Cyan). */
  public static final RGBWColor CLIMB_COLOR = new RGBWColor(0, 255, 255);

  /** LED color representing LEDs turned off. */
  public static final RGBWColor OFF_COLOR = new RGBWColor(0, 0, 0);

  /**
   * NetworkTable topics and publishers for LED state information.
   *
   * <p>These were commented out for now but can be used to publish LED states to Shuffleboard or
   * other dashboards.
   */
  /** NetworkTable topic that indicates whether the rainbow animation is currently enabled. */
  // private final BooleanTopic isRainbow;

  /** Publisher for {@link #isRainbow}, used to send the current rainbow state to NetworkTables. */
  // private final BooleanPublisher isRainbowPub;

  /** NetworkTable topic that stores the currently displayed LED color as a string. */
  // private final StringTopic currentColor;

  /** Publisher for {@link #currentColor}, used to send the current LED color to dashboards. */
  // private final StringPublisher currentColorPub;

  /** NetworkTable topic that stores the name of the currently running LED animation. */
  // private final StringTopic currentAnimation;

  /**
   * Publisher for {@link #currentAnimation}, used to send the current animation name to dashboards.
   */
  // private final StringPublisher currentAnimationPub;

  /**
   * NetworkTable topic that stores the two alternating colors for flashing or cycling effects.
   * Formatted as: "A:R{red},G{green},B{blue},W{white} | B:R{red},G{green},B{blue},W{white}"
   */
  // private final StringTopic alternatingColors;

  /**
   * Publisher for {@link #alternatingColors}, used to send the current alternating colors to
   * dashboards.
   */
  // private final StringPublisher alternatingColorsPub;

  /** */
  public LEDSubsystem() {
    setRainbowAnimation(0, 0.25, AnimationDirectionValue.Forward, Units.Hertz.of(100));

    // var nt = NetworkTableInstance.getDefault();

    // isRainbow = nt.getBooleanTopic("/color/isRainbow");
    // isRainbowPub = isRainbow.publish();
    // isRainbowPub.set(false);

    // currentColor = nt.getStringTopic("/color/currentColor");
    // currentColorPub = currentColor.publish();
    // currentColorPub.set("None");

    // currentAnimation = nt.getStringTopic("/color/currentAnimation");
    // currentAnimationPub = currentAnimation.publish();
    // currentAnimationPub.set("None");

    // alternatingColors = nt.getStringTopic("/color/alternatingColors");
    // alternatingColorsPub = alternatingColors.publish();
    // alternatingColorsPub.set("A:None | B:None");
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
  public void setHardwareColor(RGBWColor color, double brightness) {
    RGBWColor scaled = scaleBrightness(color, brightness);

    // currentColorPub.set(scaled.toHexString());
    System.out.println("Color: " + scaled.toHexString());

    solid.withColor(scaled);
    candle.setControl(solid);
  }

  public void setHardwareColor(RGBWColor color) {
    setHardwareColor(color, DEFAULT_BRIGHTNESS);
  }

  /**
   * Creates a command that sets the LEDs to the specified color while scheduled.
   *
   * <p>This command uses the {@link #setHardwareColor} function
   *
   * @param color the {@link RGBWColor} to display
   * @param brightness the brightness  to set the CANdle
   * @return a {@link Command} that sets the LEDs to the given color once
   */
  public Command setLEDsCommand(RGBWColor color, double brightness) {
    return Commands.runOnce(
            () -> {
              RGBWColor scaled = scaleBrightness(color, brightness);
              setHardwareColor(scaled);
            },
            this)
        .withName("SetLEDsWithBrightness");
  }
  /**
   * Creates a command that sets the LEDs to the specified color while scheduled.
   *
   * <p>This command uses the {@link #setHardwareColor} function
   *
   * @param color the {@link RGBWColor} to display
   * @return a {@link Command} that sets the LEDs to the given color once
   */
  public Command setLEDsCommand(RGBWColor color) {
    return Commands.runOnce(
            () -> {
              RGBWColor scaled = scaleBrightness(color);
              setHardwareColor(scaled);
            },
            this)
        .withName("Set LEDS with default Brightness");
  }
  /**
   * Scales the brightness of a given RGBW color.
   *
   * <p>This method multiplies each color channel (Red, Green, Blue, White) by the specified
   * brightness factor. The brightness factor is clamped between 0.0 (off) and 1.0 (full brightness)
   * to prevent invalid values.
   *
   * <p>Example usage:
   *
   * <pre>{@code
   * RGBWColor red = new RGBWColor(255, 0, 0, 0);
   * RGBWColor dimRed = scaleBrightness(red, 0.5); // Results in (127, 0, 0, 0)
   * }</pre>
   *
   * @param color the original {@link RGBWColor} to scale
   * @param brightness a value from 0.0 to 1.0 representing the desired brightness
   * @return a new {@link RGBWColor} with each channel scaled by the brightness factor
   */
  private RGBWColor scaleBrightness(RGBWColor color, double brightness) {
    brightness = Math.max(0.0, Math.min(1.0, brightness)); // clamp 0–1

    return new RGBWColor(
        (int) (color.Red * brightness),
        (int) (color.Green * brightness),
        (int) (color.Blue * brightness),
        (int) (color.White * brightness));
  }

  // public void publishAlternateColors(RGBWColor colorA, RGBWColor colorB) {
  //   String value =
  //       String.format(
  //           "A:R%d,G%d,B%d,W%d | B:R%d,G%d,B%d,W%d",
  //           colorA.Red,
  //           colorA.Green,
  //           colorA.Blue,
  //           colorA.White,
  //           colorB.Red,
  //           colorB.Green,
  //           colorB.Blue,
  //           colorB.White);
  //   alternatingColorsPub.set(value);
  // }

  /**
   * Alternates the LED strip between two colors, switching every {@code interval} seconds.
   *
   * <p>This command will repeatedly alternating between {@code colorA} for {@code interval} seconds
   * and {@code colorB} for {@code interval} seconds, creating a flashing effect.
   *
   * @param colorA the first color in the sequence
   * @param colorB the second color in the sequence
   * @param interval time between each color in seconds
   * @return a {@link Command} that continuously alternates LED colors
   */
  public Command alternateColors(RGBWColor colorA, RGBWColor colorB, double interval) {
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  System.out.println("Switching to Color A: " + colorA);
                  setHardwareColor(colorA);
                },
                this),
            // Commands.runOnce(() -> publishAlternateColors(colorA, colorB), this),
            Commands.waitSeconds(interval),
            Commands.runOnce(
                () -> {
                  System.out.println("Switching to Color B: " + colorB);
                  setHardwareColor(colorB);
                },
                this),
            // Commands.runOnce(() -> publishAlternateColors(colorA, colorB), this),
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
    if (colors.length == 0) {
      throw new IllegalArgumentException("Colors array must contain at least one color");
    }

    Command sequence = Commands.sequence();

    for (RGBWColor color : colors) {
      sequence =
          sequence
              .andThen(
                  Commands.runOnce(
                      () -> {
                        System.out.println("Setting color: " + color);
                        setHardwareColor(color);
                      },
                      this))
              .andThen(Commands.waitSeconds(interval));
    }

    return sequence.repeatedly();
  }

  /**
   * Enables or disables the rainbow animation on the CANdle.
   *
   * <p>If {@code enabled} is true, the preconfiguRed rainbow animation is applied. If false, the
   * active animation is cleaRed.
   *
   * @param enabled true to start the rainbow animation, false to stop it
   */
  public void setRainbowEnabled(boolean enabled) {
    if (enabled) {
      candle.setControl(rainbowAnimation);
      // isRainbowPub.set(true);
      // currentAnimationPub.set("Rainbow");
      // currentColorPub.set("None"); / always set to None
    } else {
      candle.setControl(emptyAnimation);
      // isRainbowPub.set(false);
      // currentAnimationPub.set("None");
    }
  }
}
