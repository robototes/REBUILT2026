package frc.robot.subsystems;

import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;

public class LedSubsystem extends SubsystemBase {
  public enum LedMode {
    CLIMB_IN_PROGRESS(),
    INTAKE_IN_PROGRESS(),
    SHOOT_IN_PROGRESS(),
    DEFAULT();
  }

  /** CAN device ID for the {@link CANdle} LED controller. */
  private static final int CAN_ID = Hardware.CANdle_ID;

  /**
   * Last valid LED index in the strip (inclusive).
   *
   * <p>For example, if the strip contains 8 LEDs, valid indices range from {@code 0} to {@code 7}.
   */
  private static final int END_INDEX = 7;

  /**
   * Default brightness applied to {@link #candle} operations.
   *
   * <p>Valid range: {@code 0.0} (off) to {@code 1.0} (full brightness).
   */
  public static final double DEFAULT_BRIGHTNESS = 1.0;

  /** Animation slot index used for LED animations. */
  private static final int SLOT = 0;

  /** Instance of the {@link CANdle} LED controller used to control the LED strip. */
  private final CANdle candle = new CANdle(CAN_ID);

  /**
   * Solid color controller applied to LEDs from index {@code 0} through {@link #END_INDEX}.
   *
   * <p>Used to set a single static color across the entire strip.
   */
  private final SolidColor solid = new SolidColor(0, END_INDEX);

  /**
   * Preconfigured rainbow animation assigned to {@link #SLOT}.
   *
   * <p>Applies to LEDs from index {@code 0} through {@link #END_INDEX}. Can be enabled or disabled
   * to create an animated rainbow effect.
   */
  private final RainbowAnimation rainbowAnimation =
      new RainbowAnimation(0, END_INDEX).withSlot(SLOT);

  /** Animation used to stop animations on {@link #SLOT}. */
  private final EmptyAnimation emptyAnimation = new EmptyAnimation(SLOT);

  /** Default robot LED color (red). */
  public static final RGBWColor DEFAULT_COLOR = new RGBWColor(255, 0, 0);

  /** LED color used while intaking (blue). */
  public static final RGBWColor INTAKE_COLOR = new RGBWColor(0, 0, 255);

  /** LED color used while outtaking (green). */
  public static final RGBWColor SHOOT_COLOR = new RGBWColor(0, 255, 0);

  /** LED color used during climb mode (cyan). */
  public static final RGBWColor CLIMB_COLOR = new RGBWColor(0, 255, 255);

  /** LED color representing LEDs turned off. */
  public static final RGBWColor OFF_COLOR = new RGBWColor(0, 0, 0);

  /**
   * NetworkTable topics and publishers for LED state information.
   */

  /** NetworkTable topic that indicates whether the rainbow animation is currently enabled. */
  private BooleanTopic isRainbow;

  /** Publisher for {@link #isRainbow}, used to send the current rainbow state to NetworkTables. */
  private BooleanPublisher isRainbowPub;

  /** NetworkTable topic that stores the currently displayed LED color as a string. */
  private StringTopic currentColor;

  /** Publisher for {@link #currentColor}, used to send the current LED color to dashboards. */
  private StringPublisher currentColorPub;

  /** NetworkTable topic that stores the name of the currently running LED animation. */
  private StringTopic currentAnimation;

  /**
   * Publisher for {@link #currentAnimation}, used to send the current animation name to dashboards.
   */
  private StringPublisher currentAnimationPub;

  /**
   * NetworkTable topic that stores the two alternating colors for flashing or cycling effects.
   * Formatted as: "A:R{red},G{green},B{blue},W{white} | B:R{red},G{green},B{blue},W{white}"
   */
  private StringTopic alternatingColors;

  /**
   * Publisher for {@link #alternatingColors}, used to send the current alternating colors to
   * dashboards.
   */
  private StringPublisher alternatingColorsPub;
  public LedSubsystem() {
  rainbowAnimation.withBrightness(DEFAULT_BRIGHTNESS).withDirection(AnimationDirectionValue.Forward).withFrameRate(Units.Hertz.of(100));


  NetworkTableInstance nt = NetworkTableInstance.getDefault();

  isRainbow = nt.getBooleanTopic("/color/isRainbow");
  isRainbowPub = isRainbow.publish();
  isRainbowPub.set(false);

  currentColor = nt.getStringTopic("/color/currentColor");
  currentColorPub = currentColor.publish();
  currentColorPub.set("None");

  currentAnimation = nt.getStringTopic("/color/currentAnimation");
  currentAnimationPub = currentAnimation.publish();
  currentAnimationPub.set("None");

  alternatingColors = nt.getStringTopic("/color/alternatingColors");
  alternatingColorsPub = alternatingColors.publish();
  alternatingColorsPub.set("A:None | B:None");
  }
  /**
   * Sets the CANdle LED controller to a solid color at the specified brightness.
   *
   * <p>This method directly updates the physical hardware output. It is intended to be called
   * internally by higher-level commands that manage LED behavior.
   *
   * @param color the {@link RGBWColor} to apply to the LED strip
   * @param brightness the brightness scalar (0.0–1.0) applied to the color
   */
  public void setHardwareColor(RGBWColor color, double brightness) {
    RGBWColor scaled = scaleBrightness(color, brightness);

    // currentColorPub.set(scaled.toHexString());
    System.out.println("Color: " + scaled.toHexString());

    solid.withColor(scaled);
    candle.setControl(solid);
  }

  /**
   * Sets the CANdle LED controller to a solid color using the default brightness.
   *
   * @param color the {@link RGBWColor} to apply to the LED strip
   */
  public void setHardwareColor(RGBWColor color) {
    setHardwareColor(color, DEFAULT_BRIGHTNESS);
  }

  /**
   * Creates a {@link Command} that sets the LEDs to the specified color and brightness.
   *
   * <p>The returned command runs once and immediately updates the hardware using {@link
   * #setHardwareColor(RGBWColor, double)}.
   *
   * @param color the {@link RGBWColor} to display
   * @param brightness the brightness scalar (0.0–1.0)
   * @return a command that sets the LEDs once when scheduled
   */
  public Command setLEDsCommand(RGBWColor color, double brightness) {
    return Commands.runOnce(() -> setHardwareColor(color, brightness), this)
        .withName("SetLEDsWithBrightness");
  }

  /**
   * Creates a {@link Command} that sets the LEDs to the specified color using full (1.0)
   * brightness.
   *
   * <p>The returned command runs once and immediately updates the hardware.
   *
   * @param color the {@link RGBWColor} to display
   * @return a command that sets the LEDs once when scheduled
   */
  public Command setLEDsCommand(RGBWColor color) {
    return Commands.runOnce(() -> setHardwareColor(color), this)
        .withName("SetLEDsDefaultBrightness");
  }

  /**
   * Scales the brightness of a given RGBW color.
   * <p>Each channel (red, green, blue, white) is multiplied by the {@code brightness} factor
   *
   * @param color the original {@link RGBWColor} to scale
   * @param brightness a value ideally between {@code 0.0} and {@code 1.0}
   * @return a new {@link RGBWColor} with each channel scaled by the clamped brightness factor
   */
  private RGBWColor scaleBrightness(RGBWColor color, double brightness) {
    return new RGBWColor(
        (int) (color.Red * brightness),
        (int) (color.Green * brightness),
        (int) (color.Blue * brightness),
        (int) (color.White * brightness));
  }

  public void publishAlternateColors(RGBWColor colorA, RGBWColor colorB) {
    String value =
        String.format(
            "A:R%d,G%d,B%d,W%d | B:R%d,G%d,B%d,W%d",
            colorA.Red,
            colorA.Green,
            colorA.Blue,
            colorA.White,
            colorB.Red,
            colorB.Green,
            colorB.Blue,
            colorB.White);
    alternatingColorsPub.set(value);
  }

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
      isRainbowPub.set(true);
      currentAnimationPub.set("Rainbow");
      currentColorPub.set("None"); // always set to None
    } else {
      candle.setControl(emptyAnimation);
      isRainbowPub.set(false);
      currentAnimationPub.set("None");
    }
  }
}
