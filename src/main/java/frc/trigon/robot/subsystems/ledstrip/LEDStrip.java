package frc.trigon.robot.subsystems.ledstrip;

import com.ctre.phoenix.led.*;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDStrip extends SubsystemBase {
    private static final CANdle CANDLE = LEDStripConstants.CANDLE;
    private static int LAST_CREATED_LED_STRIP_ANIMATION_SLOT = 0;
    private final int animationSlot;
    private final int offset, numberOfLEDs;

    /**
     * Constructs a new LEDStrip.
     *
     * @param offset       the offset of how many LEDs you want the LED strip to start from
     * @param numberOfLEDs the number of LEDs in the strip
     */
    public LEDStrip(int offset, int numberOfLEDs) {
        this.offset = offset;
        this.numberOfLEDs = numberOfLEDs;
        LAST_CREATED_LED_STRIP_ANIMATION_SLOT++;
        animationSlot = LAST_CREATED_LED_STRIP_ANIMATION_SLOT;
    }

    void staticColor(Color color) {
        CANDLE.setLEDs((int) color.red, (int) color.green, (int) color.blue, 0, offset, numberOfLEDs);
    }

    void threeSectionColor(Color firstSectionColor, Color secondSectionColor, Color thirdSectionColor) {
        CANDLE.setLEDs((int) firstSectionColor.red, (int) firstSectionColor.green, (int) firstSectionColor.blue, 0, offset, numberOfLEDs / 3);
        CANDLE.setLEDs((int) secondSectionColor.red, (int) secondSectionColor.green, (int) secondSectionColor.blue, 0, offset + numberOfLEDs / 3, numberOfLEDs / 3);
        CANDLE.setLEDs((int) thirdSectionColor.red, (int) thirdSectionColor.green, (int) thirdSectionColor.blue, 0, offset + 2 * numberOfLEDs / 3, numberOfLEDs / 3);
    }

    void animateFire(double brightness, double speed, double sparking, double cooling, boolean backwards) {
        CANDLE.animate(
                new FireAnimation(
                        brightness,
                        speed,
                        this.numberOfLEDs,
                        sparking,
                        cooling,
                        backwards,
                        this.offset
                ),
                animationSlot
        );
    }

    void animateRainbow(double brightness, double speed, boolean backwards) {
        CANDLE.animate(
                new RainbowAnimation(
                        brightness,
                        speed,
                        this.numberOfLEDs,
                        backwards,
                        this.offset
                ),
                animationSlot
        );
    }

    void animateColorFlow(Color color, double speed, ColorFlowAnimation.Direction direction) {
        CANDLE.animate(new ColorFlowAnimation(
                        (int) color.red,
                        (int) color.green,
                        (int) color.blue,
                        0,
                        speed,
                        this.numberOfLEDs,
                        direction,
                        this.offset
                ),
                animationSlot
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
                        this.numberOfLEDs,
                        mode,
                        size,
                        this.offset),
                animationSlot
        );
    }

    void animateRGBFade(double brightness, double speed) {
        CANDLE.animate(
                new RgbFadeAnimation(
                        brightness,
                        speed,
                        this.numberOfLEDs,
                        this.offset),
                animationSlot
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
                        this.numberOfLEDs,
                        this.offset),
                animationSlot
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
                        this.numberOfLEDs,
                        divider,
                        this.offset
                ),
                animationSlot
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
                        this.numberOfLEDs,
                        this.offset
                ),
                animationSlot
        );
    }

    void clearAnimation() {
        CANDLE.clearAnimation(animationSlot);
    }
}
