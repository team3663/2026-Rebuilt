package frc.robot.subsystems.led;

import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RgbFadeAnimation;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.LarsonBounceValue;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.RobotMode;

import java.util.function.Supplier;
//import java.util.regex.Pattern;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

public class Led extends SubsystemBase {


    private final double ANIMATION_SPEED = 0.001;
    private final double LED_BRIGHTNESS = 1.0;
    private final int NUM_LEDS = 200;
    private final int POCKET_SIZE = 3;

    private final LedIo io;
    private final LedInputs inputs = new LedInputs();
    private CANdle candle;


    private final RgbFadeAnimation startAnimation = new RgbFadeAnimation(0, 399).withFrameRate(ANIMATION_SPEED).withBrightness(1.0);
    private StatusCode currentAnimation;
    private Color currentColor = new Color();
    private Patterns currentPattern = Patterns.SOLID;
    private LarsonAnimation larsonAnimation;
    private RgbFadeAnimation rgbFadeAnimation;
    private StrobeAnimation strobeAnimation;

    public Led(LedIo io) {
        this.io = io;
        this.candle = candle;
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
                rgbFadeAnimation = new RgbFadeAnimation(0,399).withSlot(2).withFrameRate(ANIMATION_SPEED).withBrightness(1.0);
                io.setAnimation(rgbFadeAnimation);
                currentPattern = Patterns.FADE;
                break;

            case LARSON:
                io.setColor(currentColor);
                larsonAnimation = new LarsonAnimation(0,399).withSlot(3).withFrameRate(ANIMATION_SPEED).withColor(new RGBWColor(currentColor)).withSize(15).withBounceMode(LarsonBounceValue.Center);
                io.setAnimation(larsonAnimation);
                currentPattern = Patterns.LARSON;
                break;

            case STROBE:
                io.setColor(currentColor);
                strobeAnimation = new StrobeAnimation(0,399).withSlot(4).withColor(new RGBWColor(currentColor)).withFrameRate(ANIMATION_SPEED);
                io.setAnimation(strobeAnimation);
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
    //TODO if we switch colors based on a robot mode then this is where we will put them
//    public Command signalCommand(Supplier<RobotMode> robotMode) {
//        return run(() -> {
//            switch (robotMode.get()) {
//
//            }
//        };
//    }

    public Command intakeFlash() {
        return runOnce(() -> {
            currentColor = Color.kWhite;
            setPattern(Patterns.STROBE);
        }).andThen(waitSeconds(0.5));
    }

    public enum Patterns {
        SOLID, FADE, LARSON, STROBE;
    }
}