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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import frc.robot.util.LEDMode;
// import java.util.function.Supplier;

public class LEDSubsystem extends SubsystemBase {
  /** Constants goes here */
  private static final int CAN_ID = 22;

  private static final int END_INDEX =
      7; // temporary for testing (only using 1 module which is 8 leds)

  private final CANdle candle = new CANdle(CAN_ID);
  private final SolidColor solid = new SolidColor(0, END_INDEX);
  private final RainbowAnimation slot0Animation = new RainbowAnimation(0, END_INDEX);
  private final EmptyAnimation emptyAnimation = new EmptyAnimation(0);

  public static final RGBWColor DEFAULT_COLOR = new RGBWColor(255, 0, 0); // red
  public static final RGBWColor INTAKE_COLOR = new RGBWColor(255, 255, 0); // yellow
  public static final RGBWColor OUTTAKE_COLOR = new RGBWColor(0, 255, 0); // green
  public static final RGBWColor CLIMB_COLOR = new RGBWColor(0, 0, 255); // blue
  public static final RGBWColor OFF_COLOR = new RGBWColor(0, 0, 0); // off

  private RGBWColor activeSolidColor = OFF_COLOR;

  public LEDSubsystem() {
    setRainbowAnimation(0, 1, AnimationDirectionValue.Forward, Units.Hertz.of(100));
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

    slot0Animation
        .withSlot(slot)
        .withBrightness(brightness)
        .withDirection(direction)
        .withFrameRate(frameRate);
  }

  /* to be used in the future */
  // public Command LED_Mode(Supplier<LEDMode> mode) {
  //   return Commands.run(
  //       () -> {
  //         LEDMode currentMode = mode.get();
  //         switch (currentMode) {
  //           case DEFAULT:
  //             setLEDsCommand(defaultColor);
  //             break;
  //           case OUTTAKE_IN_PROGRESS:
  //             setLEDsCommand(outtakeColor);
  //             break;
  //           case INTAKE_IN_PROGRESS:
  //             setLEDsCommand(intakeColor);
  //             break;
  //           case CLIMB_IN_PROGRESS:
  //             setLEDsCommand(climbColor);
  //             break;
  //         }
  //       }, this);
  // }

  /**
   * Sets Leds at specified {@link frc.robot.subsystems.LEDSubsystem#CAN_ID id} to color
   *
   * @param color
   */
  public void setHardwareColor(RGBWColor color) {
    solid.withColor(color);
    candle.setControl(solid);
  }

  public Command setLEDsCommand(RGBWColor color) {
    return Commands.run(() -> setHardwareColor(color), this)
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
  public Command alternateColors(RGBWColor colorA, RGBWColor colorB) {
    return Commands.sequence(
            Commands.runOnce(() -> setHardwareColor(colorA), this),
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> setHardwareColor(colorB), this),
            Commands.waitSeconds(0.5))
        .repeatedly();
  }

  /**
   * Toggles between {@code otherColor} with {@link frc.robot.subsystems.LEDSubsystem#OFF_COLOR}
   *
   * @param otherColor the color to toggle with
   * @return a {@link Command} that toggles the rainbow animation state
   */
  public Command toggleColor(RGBWColor otherColor) {
    return new InstantCommand(
        () -> {
          if (activeSolidColor.equals(otherColor)) {
            // If the requested color is already active, turn it off.
            setHardwareColor(OFF_COLOR);
            activeSolidColor = OFF_COLOR;
          } else {
            // Otherwise, set the new color.
            setHardwareColor(otherColor);
            activeSolidColor = otherColor;
          }
        },
        this);
  }

  // Start rainbow animation
  public void startRainbow() {
    candle.setControl(slot0Animation);
  }

  // Stop rainbow animation
  public void stopRainbow() {
    candle.setControl(emptyAnimation);
  }
}
