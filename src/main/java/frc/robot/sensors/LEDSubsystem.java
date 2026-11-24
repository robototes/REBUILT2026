package frc.robot.sensors;

import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.AutoLogOutput;

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

    public enum LEDPattern {
        SOLID,
        ALTERNATE,
        RAINBOW
    }

    private LEDMode currentMode = LEDMode.DEFAULT;
    private LEDPattern currentPattern = LEDPattern.SOLID;

    private final LEDSubsystemIO io;

    public static final RGBWColor OFF = new RGBWColor(0, 0, 0);

    // Purple
    public static final RGBWColor DEFAULT_COLOR =
        new RGBWColor(255, 0, 255);

    // Blue
    public static final RGBWColor INTAKE_COLOR =
        new RGBWColor(0, 0, 255);

    // Orange
    public static final RGBWColor CLIMB_COLOR =
        new RGBWColor(255, 128, 0);

    // Green
    public static final RGBWColor LAUNCH_COLOR =
        new RGBWColor(0, 255, 0);

    // Yellow
    public static final RGBWColor LAUNCH_PREP_COLOR =
        new RGBWColor(255, 255, 0);

    // Red
    public static final RGBWColor LAUNCH_PREP_COLOR_TWO =
        new RGBWColor(255, 0, 0);

    private RGBWColor primaryColor;
    private RGBWColor secondaryColor;

    private double interval = 0.25;
    private double lastToggleTime = 0;

    private boolean showingPrimary = true;

    public LEDSubsystem(LEDSubsystemIO io) {
        this.io = io;
    }

    // AdvantageKit Logging
    @AutoLogOutput(key = "LED/CurrentMode")
    public String getCurrentMode() {
        return String.valueOf(currentMode);
    }

    @AutoLogOutput(key = "LED/CurrentPattern")
    public String getCurrentPattern() {
        return String.valueOf(currentPattern);
    }

    @AutoLogOutput(key = "LED/PrimaryColor")
    public String getPrimaryColor() {
        return primaryColor == null
            ? "None"
            : primaryColor.toHexString();
    }

    @AutoLogOutput(key = "LED/SecondaryColor")
    public String getSecondaryColor() {
        return secondaryColor == null
            ? "None"
            : secondaryColor.toHexString();
    }

    @AutoLogOutput(key = "LED/Interval")
    public double getInterval() {
        return interval;
    }

    @AutoLogOutput(key = "LED/LastToggleTime")
    public double getLastToggleTime() {
        return lastToggleTime;
    }

    @AutoLogOutput(key = "LED/ShowingPrimary")
    public boolean getShowingPrimary() {
        return showingPrimary;
    }

    // Hardware Color Control
    public void setHardwareColor(RGBWColor color, double brightness) {
        io.setHardwareColor(color, brightness);
    }

    public void setHardwareColor(RGBWColor color) {
        setHardwareColor(color, 1.0);
    }

    // LED Commands
    public Command setLEDsCommand(RGBWColor color, double brightness) {
        return Commands.runOnce(
            () -> setHardwareColor(color, brightness),
            this
        ).withName("Set LEDs With Brightness");
    }

    public Command setLEDsCommand(RGBWColor color) {
        return Commands.runOnce(
            () -> setHardwareColor(color),
            this
        ).withName("Set LEDs color");
    }

    // LED Patterns
    private void setSolid(RGBWColor color) {
        currentPattern = LEDPattern.SOLID;
        primaryColor = color;
        secondaryColor = null;

        setHardwareColor(color);
    }

    private void setAlternating(
        RGBWColor a,
        RGBWColor b,
        double interval
    ) {
        currentPattern = LEDPattern.ALTERNATE;

        primaryColor = a;
        secondaryColor = b;

        this.interval = interval;
        showingPrimary = true;
        lastToggleTime = Timer.getFPGATimestamp();

        setHardwareColor(a);
    }

    private void setRainbow() {
        currentPattern = LEDPattern.RAINBOW;

        io.setRainbow();
    }

    // LED Modes
    public void setMode(LEDMode mode) {
        if (mode == currentMode) return;

        currentMode = mode;

        switch (mode) {
            case INTAKE -> setSolid(INTAKE_COLOR);
            case CLIMB -> setSolid(CLIMB_COLOR);
            case DEFAULT -> setSolid(DEFAULT_COLOR);
            case LAUNCH -> setSolid(LAUNCH_COLOR);
            case LAUNCHING ->
                setAlternating(
                    LAUNCH_PREP_COLOR,
                    LAUNCH_PREP_COLOR_TWO,
                    0.25
                );
            case DISABLED -> setSolid(OFF);
            case RAINBOW -> setRainbow();
        }
    }

    // Other Commands
    public Command flashCommand(
        RGBWColor color,
        int times,
        double interval
    ) {
        return Commands.sequence(

            Commands.repeatingSequence(

                Commands.runOnce(
                    () -> setHardwareColor(color),
                    this
                ),

                Commands.waitSeconds(interval),

                Commands.runOnce(
                    () -> setHardwareColor(OFF),
                    this
                ),

                Commands.waitSeconds(interval)

            ).withTimeout(times * interval * 2),

            Commands.runOnce(
                () -> {
                    LEDMode saved = currentMode;

                    currentMode = null;

                    setMode(saved);
                },
                this
            )

        ).withName("Flash LEDs");
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

            if (
                Timer.getFPGATimestamp() - lastToggleTime > interval
            ) {

                showingPrimary = !showingPrimary;

                setHardwareColor(
                    showingPrimary
                        ? primaryColor
                        : secondaryColor
                );

                lastToggleTime = Timer.getFPGATimestamp();
            }
        }
    }
}
