package frc.trigon.robot.subsystems.ledstrip;

import com.ctre.phoenix.led.*;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDStrip extends SubsystemBase {
    private static final CANdle CANDLE = LEDStripConstants.CANDLE;
    private static int lastCreatedLEDStrip = 0;
    private final int id;
    private final int offset, numLEDs;

    public LEDStrip(int offset, int numLEDs) {
        this.offset = offset;
        this.numLEDs = numLEDs;
        this.id = setId();
    }

    void setLEDs(Color color) {
        CANDLE.setLEDs((int) color.red, (int) color.green, (int) color.blue, 0, offset, numLEDs);
    }

    void animateFire(double brightness, double speed, double sparking, double cooling, boolean backwards) {
        CANDLE.animate(
                new FireAnimation(
                        brightness,
                        speed,
                        this.numLEDs,
                        sparking,
                        cooling,
                        backwards,
                        this.offset
                ),
                0
        );
    }

    void animateRainbow(double brightness, double speed, boolean backwards) {
        CANDLE.animate(
                new RainbowAnimation(
                        brightness,
                        speed,
                        this.numLEDs,
                        backwards,
                        this.offset
                ),
                0
        );
    }

    void animateColourFlow(Color color, double speed, ColorFlowAnimation.Direction direction) {
        CANDLE.animate(new ColorFlowAnimation(
                        (int) color.red,
                        (int) color.green,
                        (int) color.blue,
                        0,
                        speed,
                        this.numLEDs,
                        direction,
                        this.offset
                ),
                0
        );
    }

    void animateLarson(Color color, double speed, LarsonAnimation.BounceMode mode, int size) {
        CANDLE.animate(
                new LarsonAnimation(
                        (int) color.red,
                        (int) color.green,
                        (int) color.blue,
                        0,
                        speed,
                        this.numLEDs,
                        mode,
                        size,
                        this.offset),
                0
        );
    }

    void animateRGBFade(double brightness, double speed) {
        CANDLE.animate(
                new RgbFadeAnimation(
                        brightness,
                        speed,
                        this.numLEDs,
                        this.offset),
                0
        );
    }

    void animateSingleFade(Color color, double speed) {
        CANDLE.animate(
                new SingleFadeAnimation(
                        (int) color.red,
                        (int) color.green,
                        (int) color.blue,
                        0,
                        speed,
                        this.numLEDs,
                        this.offset),
                0
        );
    }

    void animateTwinkle(Color color, double speed, TwinkleAnimation.TwinklePercent divider) {
        CANDLE.animate(
                new TwinkleAnimation(
                        (int) color.red,
                        (int) color.green,
                        (int) color.blue,
                        0,
                        speed,
                        this.numLEDs,
                        divider,
                        this.offset
                ),
                0
        );
    }

    void animateStrobe(Color color, double speed) {
        CANDLE.animate(
                new StrobeAnimation(
                        (int) color.red,
                        (int) color.green,
                        (int) color.blue,
                        0,
                        speed,
                        this.numLEDs,
                        this.offset
                ),
                0
        );
    }

    void clearAnimation() {
        CANDLE.clearAnimation(0);
    }

    private int setId() {
        lastCreatedLEDStrip++;
        return lastCreatedLEDStrip;
    }
}
