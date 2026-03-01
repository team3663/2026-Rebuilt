package frc.robot.subsystems.led;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RgbFadeAnimation;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.signals.LarsonBounceValue;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Led extends SubsystemBase {

    private final static double ANIMATION_SPEED = 0.001;
    private final static double LED_BRIGHTNESS = 1.0;
    private final static int NUM_LEDS = 399;
    private final static int LARSON_POCKET_SIZE = 3;

    private final LedIo io;
    private final LedInputs inputs = new LedInputs();

    private final ControlRequest startAnimation = new RgbFadeAnimation(0, NUM_LEDS).withFrameRate(ANIMATION_SPEED).withBrightness(LED_BRIGHTNESS);
    private ControlRequest currentAnimation;
    private Color currentColor = new Color();
    private Patterns currentPattern = Patterns.SOLID;

    public Led(LedIo io) {
        this.io = io;
        io.setAnimation(startAnimation);
    }

    public void periodic() {
        io.updateInputs(inputs);
    }

    public void setColor(Color color) {
        currentColor = color;
        setPattern(currentPattern);
    }

    public void setPattern(Patterns pattern) {
        switch (pattern) {
            case SOLID:
                io.setAnimation(null);
                io.setColor(currentColor);
                currentPattern = Patterns.SOLID;
                break;

            case FADE:
                io.setColor(currentColor);
                currentAnimation = new RgbFadeAnimation(0, NUM_LEDS).withSlot(2).withFrameRate(ANIMATION_SPEED).withBrightness(LED_BRIGHTNESS);
                io.setAnimation(currentAnimation);
                currentPattern = Patterns.FADE;
                break;

            case LARSON:
                io.setColor(currentColor);
                currentAnimation = new LarsonAnimation(0, NUM_LEDS).withSlot(3).withFrameRate(ANIMATION_SPEED).withColor(new RGBWColor(currentColor)).withSize(LARSON_POCKET_SIZE).withBounceMode(LarsonBounceValue.Center);
                io.setAnimation(currentAnimation);
                currentPattern = Patterns.LARSON;
                break;

            case STROBE:
                io.setColor(currentColor);
                currentAnimation = new StrobeAnimation(0, NUM_LEDS).withSlot(4).withColor(new RGBWColor(currentColor)).withFrameRate(ANIMATION_SPEED);
                io.setAnimation(currentAnimation);
                currentPattern = Patterns.STROBE;
                break;
        }
    }

    public Command setLedColor(Color color) {
        return runOnce(() -> {
                    currentColor = color;
                    io.setColor(color);
                }
        );
    }

    public enum Patterns {
        SOLID, FADE, LARSON, STROBE;
    }
}