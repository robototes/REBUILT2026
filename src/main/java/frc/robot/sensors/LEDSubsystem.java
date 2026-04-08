package frc.robot.sensors;

import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;

public class LEDSubsystem extends SubsystemBase {
  public enum LEDMode {
    CLIMB,
    INTAKE,
    LAUNCH,
    LAUNCHING,
    RAINBOW,
    DISABLED,
    DEFAULT;
  }

  private enum LEDPattern {
    SOLID,
    ALTERNATE,
    RAINBOW
  }

  private LEDMode currentMode = LEDMode.DEFAULT;
  private LEDPattern currentPattern = LEDPattern.SOLID;
  private static final int CAN_ID = Hardware.CANDLE_ID;
  private static final int END_INDEX = 200;

  /** Animation slot index used for LED animations. */
  private static final int SLOT = 0;

  private final CANdle candle = new CANdle(CAN_ID);
  private final SolidColor solidController = new SolidColor(0, END_INDEX);
  private final EmptyAnimation emptyAnimation = new EmptyAnimation(SLOT);
  private final RainbowAnimation rainbowAnimation =
      new RainbowAnimation(0, END_INDEX).withSlot(SLOT);

  public static final RGBWColor OFF = new RGBWColor(0, 0, 0);
  // Purple
  public static final RGBWColor DEFAULT_COLOR = new RGBWColor(255, 0, 255);
  // Blue
  public static final RGBWColor INTAKE_COLOR = new RGBWColor(0, 0, 255);
  // Orange
  public static final RGBWColor CLIMB_COLOR = new RGBWColor(255, 128, 0);
  // Green
  public static final RGBWColor LAUNCH_COLOR = new RGBWColor(0, 255, 0);
  // Yellow
  public static final RGBWColor LAUNCH_PREP_COLOR = new RGBWColor(255, 255, 0);
  // Red
  public static final RGBWColor LAUNCH_PREP_COLOR_TWO = new RGBWColor(255, 0, 0);

  private RGBWColor primaryColor;
  private RGBWColor secondaryColor;

  private double interval = 0.25;
  private double lastToggleTime = 0;
  private boolean showingPrimary = true;

  private final StringPublisher currentColorPub;
  private final StringPublisher currentPatternPub;
  private final StringPublisher currentModePub;
  private final StringPublisher alternatingColorsPub;

  public LEDSubsystem() {
    rainbowAnimation
        .withBrightness(1.0)
        .withDirection(AnimationDirectionValue.Forward)
        .withFrameRate(Units.Hertz.of(100));

    NetworkTableInstance nt = NetworkTableInstance.getDefault();

    currentColorPub = nt.getStringTopic("/color/currentColor").publish();
    currentColorPub.set("None");

    currentPatternPub = nt.getStringTopic("/color/currentPattern").publish();
    currentPatternPub.set("None");

    currentModePub = nt.getStringTopic("/color/currentMode").publish();
    currentModePub.set("None");

    alternatingColorsPub = nt.getStringTopic("/color/alternatingColors").publish();
    alternatingColorsPub.set("A:None | B:None");
  }

  private void publishState() {
    currentPatternPub.set(currentPattern.name());
    currentModePub.set(currentMode != null ? currentMode.name() : "NONE");

    if (currentPattern == LEDPattern.SOLID) {
      currentColorPub.set(primaryColor.toHexString());
      alternatingColorsPub.set("A:None | B:None");
    } else if (currentPattern == LEDPattern.ALTERNATE) {
      currentColorPub.set("NONE");
      alternatingColorsPub.set(
          "A:" + primaryColor.toHexString() + " | B:" + secondaryColor.toHexString());
    } else {
      currentColorPub.set("NONE");
      alternatingColorsPub.set("A:None | B:None");
    }
  }

  public void setHardwareColor(RGBWColor color, double brightness) {
    RGBWColor scaled = color.scaleBrightness(brightness);

    solidController.withColor(scaled);
    candle.setControl(solidController.clone());
  }

  public void setHardwareColor(RGBWColor color) {
    setHardwareColor(color, 1.0);
  }

  public Command setLEDsCommand(RGBWColor color, double brightness) {
    return Commands.runOnce(() -> setHardwareColor(color, brightness))
        .withName("Set LEDs With Brightness");
  }

  public Command setLEDsCommand(RGBWColor color) {
    return Commands.runOnce(() -> setHardwareColor(color)).withName("Set LEDs color");
  }

  private void setSolid(RGBWColor color) {
    candle.setControl(emptyAnimation);
    currentPattern = LEDPattern.SOLID;
    primaryColor = color;

    setHardwareColor(color);
    publishState();
  }

  private void setAlternating(RGBWColor a, RGBWColor b, double interval) {
    candle.setControl(emptyAnimation);
    currentPattern = LEDPattern.ALTERNATE;

    primaryColor = a;
    secondaryColor = b;

    this.interval = interval;
    showingPrimary = true;
    lastToggleTime = Timer.getFPGATimestamp();

    setHardwareColor(a);
    publishState();
  }

  private void setRainbow() {
    currentPattern = LEDPattern.RAINBOW;
    candle.setControl(rainbowAnimation);
    publishState();
  }

  public void setMode(LEDMode mode) {
    if (mode == currentMode) return;

    currentMode = mode;

    switch (mode) {
      case INTAKE -> setSolid(INTAKE_COLOR);
      case CLIMB -> setSolid(CLIMB_COLOR);
      case DEFAULT -> setSolid(DEFAULT_COLOR);
      case LAUNCH -> setSolid(LAUNCH_COLOR);
      case LAUNCHING -> setAlternating(LAUNCH_PREP_COLOR, LAUNCH_PREP_COLOR_TWO, 0.25);
      case DISABLED -> setSolid(OFF);
      case RAINBOW -> setRainbow();
    }

    publishState();
  }

  public Command flashCommand(RGBWColor color, int times, double interval) {
    return Commands.sequence(
            Commands.repeatingSequence(
                    Commands.runOnce(() -> setHardwareColor(color)),
                    Commands.waitSeconds(interval),
                    Commands.runOnce(() -> setHardwareColor(OFF)),
                    Commands.waitSeconds(interval))
                .withTimeout(times * interval * 2),
            Commands.runOnce(
                () -> {
                  LEDMode saved = currentMode;
                  currentMode = null;
                  setMode(saved);
                }))
        .withName("Flash LEDs");
  }

  @Override
  public void periodic() {
    if (RobotState.isDisabled()) {
      if (currentMode != LEDMode.DISABLED) {
        setMode(LEDMode.DISABLED);
      }
      return;
    }

    if (currentPattern == LEDPattern.ALTERNATE) {
      if (Timer.getFPGATimestamp() - lastToggleTime > interval) {
        showingPrimary = !showingPrimary;
        setHardwareColor(showingPrimary ? primaryColor : secondaryColor);
        lastToggleTime = Timer.getFPGATimestamp();
      }
    }
  }
}
