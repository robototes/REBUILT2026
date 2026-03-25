package frc.robot.sensors;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import frc.robot.subsystems.launcher.TurretSubsystem;
import frc.robot.util.AllianceUtils;

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

  private static final int RING_START_INDEX = 8;
  private static final int RING_COUNT = 60;
  private static final double RING_ZERO_ANGLE_DEG = 90.0; // LED 0 points LEFT
  private static final boolean RING_CLOCKWISE = true;
  private static final int HUB_LED_COUNT = 7;
  private static final int TURRET_LED_COUNT = 3;

  private final CANdle candle = new CANdle(CAN_ID);
  private final SolidColor solidController = new SolidColor(0, END_INDEX);
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
  public static final RGBWColor RED = new RGBWColor(255, 0, 0);
  // White
  public static final RGBWColor WHITE = new RGBWColor(255, 255, 255);

  private RGBWColor primaryColor;
  private RGBWColor secondaryColor;

  private double interval = 0.25;
  private double lastToggleTime = 0;
  private boolean showingPrimary = true;

  private CommandSwerveDrivetrain driveTrain;
  private TurretSubsystem turret;

  private final StringPublisher currentColorPub;
  private final StringPublisher currentPatternPub;
  private final StringPublisher currentModePub;
  private final StringPublisher alternatingColorsPub;

  public LEDSubsystem() {
    rainbowAnimation
        .withBrightness(1.0)
        .withDirection(AnimationDirectionValue.Forward)
        .withFrameRate(Hertz.of(100));

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

  public void setTargetingDependencies(CommandSwerveDrivetrain driveTrain, TurretSubsystem turret) {
    this.driveTrain = driveTrain;
    this.turret = turret;
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
    currentPattern = LEDPattern.SOLID;
    primaryColor = color;
    setHardwareColor(color);

    currentPatternPub.set("SOLID");
    currentColorPub.set(color.toHexString());
    alternatingColorsPub.set("A:None | B:None");
  }

  private void setAlternating(RGBWColor a, RGBWColor b, double interval) {
    currentPattern = LEDPattern.ALTERNATE;

    primaryColor = a;
    secondaryColor = b;

    this.interval = interval;
    showingPrimary = true;
    lastToggleTime = Timer.getFPGATimestamp();

    currentPatternPub.set("ALTERNATING");
    currentColorPub.set(a.toHexString());
    alternatingColorsPub.set("A:" + a.toHexString() + " | B:" + b.toHexString());

    setHardwareColor(a);
  }

  private void setRainbow() {
    currentPattern = LEDPattern.RAINBOW;

    // Update publishers
    currentPatternPub.set("RAINBOW");
    currentColorPub.set("NONE");
    alternatingColorsPub.set("A:None | B:None");

    candle.setControl(rainbowAnimation);
  }

  public void setMode(LEDMode mode) {
    if (mode == currentMode) return;

    currentMode = mode;

    switch (mode) {
      case INTAKE -> setSolid(INTAKE_COLOR);
      case CLIMB -> setSolid(CLIMB_COLOR);
      case DEFAULT -> setSolid(DEFAULT_COLOR);
      case LAUNCH -> setSolid(LAUNCH_COLOR);
      case LAUNCHING -> setAlternating(LAUNCH_PREP_COLOR, RED, 0.25);
      case DISABLED -> setSolid(OFF);
      case RAINBOW -> setRainbow();
    }

    currentModePub.set(mode.toString());
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

  private int angleToLEDIndex(double robotRelativeDeg) {
    double delta = robotRelativeDeg - RING_ZERO_ANGLE_DEG;
    if (RING_CLOCKWISE) delta = -delta;
    delta = ((delta % 360) + 360) % 360;
    return (int) Math.round(delta * (RING_COUNT / 360.0)) % RING_COUNT;
  }

  /**
   * Returns true if the given ring-relative LED index falls within the overlay (hub white band or
   * turret red band).
   */
  private boolean isOverlayIndex(int ringIdx, int hubCenter, int turretCenter) {
    int distHub =
        Math.min(Math.abs(ringIdx - hubCenter), RING_COUNT - Math.abs(ringIdx - hubCenter));
    int distTurret =
        Math.min(Math.abs(ringIdx - turretCenter), RING_COUNT - Math.abs(ringIdx - turretCenter));
    return distHub <= HUB_LED_COUNT / 2 || distTurret <= TURRET_LED_COUNT / 2;
  }

  /**
   * Writes the hub (white) and turret (red) overlay LEDs directly using per-segment SolidColor
   * controls, skipping indices handled by the current mode pattern.
   */
  private void applyOverlay(int hubCenter, int turretCenter) {
    for (int i = 0; i < RING_COUNT; i++) {
      int distHub = Math.min(Math.abs(i - hubCenter), RING_COUNT - Math.abs(i - hubCenter));
      int distTurret =
          Math.min(Math.abs(i - turretCenter), RING_COUNT - Math.abs(i - turretCenter));

      // Turret takes priority over hub
      if (distTurret <= TURRET_LED_COUNT / 2) {
        candle.setControl(new SolidColor(RING_START_INDEX + i, 1).withColor(RED));
      } else if (distHub <= HUB_LED_COUNT / 2) {
        candle.setControl(new SolidColor(RING_START_INDEX + i, 1).withColor(WHITE));
      }
    }
  }

  /**
   * Writes the alternating pattern for the ring, skipping overlay indices so the hub and turret
   * LEDs never flash.
   */
  private void applyAlternatingWithOverlay(RGBWColor color, int hubCenter, int turretCenter) {
    for (int i = 0; i < RING_COUNT; i++) {
      if (!isOverlayIndex(i, hubCenter, turretCenter)) {
        candle.setControl(new SolidColor(RING_START_INDEX + i, 1).withColor(color));
      }
    }
  }

  private void updateOverlay() {
    if (driveTrain == null || turret == null) return;

    Translation2d robotPos = driveTrain.getState().Pose.getTranslation();
    Translation2d toHub = AllianceUtils.getHubTranslation2d().minus(robotPos);
    double hubFieldDeg = Math.toDegrees(Math.atan2(toHub.getY(), toHub.getX()));
    double robotHeadingDeg = driveTrain.getState().Pose.getRotation().getDegrees();
    double hubRobotDeg = hubFieldDeg - robotHeadingDeg;
    double turretDeg = Units.rotationsToDegrees(turret.getTurretPosition());

    int hubCenter = angleToLEDIndex(hubRobotDeg);
    int turretCenter = angleToLEDIndex(turretDeg);

    if (currentPattern == LEDPattern.ALTERNATE) {
      // Rewrite the non-overlay ring LEDs with the current flash color,
      // then paint the overlay on top
      RGBWColor flashColor = showingPrimary ? primaryColor : secondaryColor;
      applyAlternatingWithOverlay(flashColor, hubCenter, turretCenter);
    }

    // Always paint overlay last so it's always on top
    applyOverlay(hubCenter, turretCenter);
  }

  @Override
  public void periodic() {
    if (RobotState.isDisabled()) {
      setMode(LEDMode.DISABLED);
      return;
    }

    if (currentPattern == LEDPattern.ALTERNATE) {
      if (Timer.getFPGATimestamp() - lastToggleTime > interval) {
        showingPrimary = !showingPrimary;
        setHardwareColor(showingPrimary ? primaryColor : secondaryColor);
        lastToggleTime = Timer.getFPGATimestamp();
      }
    }

    updateOverlay();
  }
}
