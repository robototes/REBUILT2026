package frc.robot.sensors;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED_lights extends SubsystemBase {

  private final CANdle candle = new CANdle(15);
  private static final int END_LED_INDEX = 7;

  // Colors
  public final RGBWColor defaultColor = new RGBWColor(255, 0, 0); // red
  public final RGBWColor intakeColor = new RGBWColor(255, 255, 0); // yellow
  public final RGBWColor outtakeColor = new RGBWColor(0, 255, 0); // green
  public final RGBWColor climbColor = new RGBWColor(0, 0, 255); // blue

  // Cached LED controls (NO runtime allocation)
  private final SolidColor solidColor = new SolidColor(0, END_LED_INDEX);

  private final RainbowAnimation rainbowAnimation =
      new RainbowAnimation(0, END_LED_INDEX)
          .withSlot(0)
          .withBrightness(1)
          .withDirection(AnimationDirectionValue.Forward)
          .withFrameRate(Hertz.of(100));


  private void updateLEDs(RGBWColor color) {
    solidColor.withColor(color);
    candle.setControl(solidColor);
  }

  // ---------------- COMMAND FACTORIES ----------------

  private Command showColor(RGBWColor color) {
    return Commands.run(() -> updateLEDs(color), this).beforeStarting(dontDoRainbowAnimation);
  }

  public Command alternateColors(RGBWColor colorA, RGBWColor colorB) {
    return new RunCommand(
            () -> {
              long seconds = System.currentTimeMillis() / 1000;
              updateLEDs((seconds & 1) == 0 ? colorA : colorB);
            },
            this)
        .beforeStarting(dontDoRainbowAnimation);
  }

  public final Command showIntakeColor = showColor(intakeColor);
  public final Command showOuttakeColor = showColor(outtakeColor);
  public final Command showClimbColor = showColor(climbColor);
  public final Command showDefaultColor = showColor(defaultColor);

  public final Command alternateClimbIntake = alternateColors(climbColor, intakeColor);

  public final Command doRainbowAnimation =
      Commands.run(() -> candle.setControl(rainbowAnimation), this);

  public final Command dontDoRainbowAnimation =
      Commands.runOnce(() -> candle.setControl((RainbowAnimation) null), this);
}
