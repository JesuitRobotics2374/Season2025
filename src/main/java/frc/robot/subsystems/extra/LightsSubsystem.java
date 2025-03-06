package frc.robot.subsystems.extra;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightsSubsystem extends SubsystemBase {

    private CANdle led;

    private ColorFlowAnimation colorFlowAnimation;
    private FireAnimation fireAnimation;
    private LarsonAnimation larsonAnimation;
    private RainbowAnimation rainbowAnimation;
    private RgbFadeAnimation rgbFadeAnimation;
    private SingleFadeAnimation singleFadeAnimation;
    private StrobeAnimation strobeAnimation;
    private TwinkleAnimation twinkleAnimation;
    private TwinkleOffAnimation twinkleOffAnimation;

    private CANdleConfiguration config;

    private int ledCount = 256; // ADJUST THIS

    public LightsSubsystem() {

        led = new CANdle(40, "rio");

        config = new CANdleConfiguration();
        config.brightnessScalar = 0.5;
        led.configAllSettings(config);

    }

    public void defaultPattern() {
        led.clearAnimation(0);
        led.setLEDs(125, 35, 250);
    }

    public void disable() {
        led.setLEDs(0, 0, 0);
        led.clearAnimation(0);
    }

    public void playColorFlowAnimation(int r, int g, int b, int w, double speed, Direction direction) {
        colorFlowAnimation = new ColorFlowAnimation(r, g, b, w, speed, ledCount, direction);
        led.animate(colorFlowAnimation);
    }

    public void playRainbowAnimation(double brightness, double speed) {
        rainbowAnimation = new RainbowAnimation(brightness, speed, ledCount);
        led.animate(rainbowAnimation);
    }

    // ... add others ...

    @Override
    public void periodic() {
    }
}
