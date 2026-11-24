package frc.robot.sensors;

import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.units.Units;
import frc.robot.Hardware;

public class LEDSubsystemIOCANdle implements LEDSubsystemIO {

    private static final int CAN_ID = Hardware.CANDLE_ID;
    private static final int END_INDEX = 200;

    /** Animation slot index used for LED animations. */
    private static final int SLOT = 0;

    private final CANdle candle = new CANdle(CAN_ID);

    private final SolidColor solidController =
        new SolidColor(0, END_INDEX);

    private final EmptyAnimation emptyAnimation =
        new EmptyAnimation(SLOT);

    private final RainbowAnimation rainbowAnimation =
        new RainbowAnimation(0, END_INDEX).withSlot(SLOT);

    public LEDSubsystemIOCANdle() {

        rainbowAnimation
            .withBrightness(1.0)
            .withDirection(AnimationDirectionValue.Forward)
            .withFrameRate(Units.Hertz.of(100));

    }

    @Override
    public void setHardwareColor(RGBWColor color, double brightness) {

        RGBWColor scaled = color.scaleBrightness(brightness);

        candle.setControl(emptyAnimation);

        solidController.withColor(scaled);

        candle.setControl(solidController.clone());
    }

    @Override
    public void setHardwareColor(RGBWColor color) {
        setHardwareColor(color, 1.0);
    }

    @Override
    public void setRainbow() {
        candle.setControl(rainbowAnimation);
    }

}
