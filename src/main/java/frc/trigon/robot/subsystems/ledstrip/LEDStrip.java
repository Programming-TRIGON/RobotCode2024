package frc.trigon.robot.subsystems.ledstrip;

import com.ctre.phoenix.led.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.trigon.robot.Robot;
import frc.trigon.robot.commands.Commands;

import java.awt.*;
import java.util.function.Function;

public class LEDStrip extends SubsystemBase {
    private static final CANdle CANDLE = LEDStripConstants.CANDLE;
    private static int LAST_CREATED_LED_STRIP_ANIMATION_SLOT = 0;
    private static final Trigger LOW_BATTERY_TRIGGER = new Trigger(() -> !DriverStation.isEnabled() && Robot.IS_REAL && RobotController.getBatteryVoltage() < LEDStripConstants.MINIMUM_BATTERY_VOLTAGE);
    private final int animationSlot;
    private final int offset, numberOfLEDs;
    private final boolean inverted;

    static {
        Commands.getDelayedCommand(1, () -> LOW_BATTERY_TRIGGER.whileTrue(LEDStripCommands.getAnimateSingleFadeCommand(Color.red, LEDStripConstants.LOW_BATTERY_FLASHING_SPEED, LEDStripConstants.LED_STRIPS))).schedule();
    }

    /**
     * Constructs a new LEDStrip.
     *
     * @param offset       the offset of how many LEDs you want the LED strip to start from
     * @param numberOfLEDs the number of LEDs in the strip
     * @param inverted     is the strip inverted
     */
    public LEDStrip(int offset, int numberOfLEDs, boolean inverted) {
        this.offset = offset;
        this.numberOfLEDs = numberOfLEDs;
        this.inverted = inverted;
        LAST_CREATED_LED_STRIP_ANIMATION_SLOT++;
        animationSlot = LAST_CREATED_LED_STRIP_ANIMATION_SLOT;
    }

    public static void setDefaultCommandForAllLEDS(Function<LEDStrip, Command> command) {
        for (LEDStrip ledStrip : LEDStripConstants.LED_STRIPS)
            ledStrip.setDefaultCommand(command.apply(ledStrip));
    }

    void staticColor(Color color) {
        CANDLE.setLEDs(color.getRed(), color.getGreen(), color.getBlue(), 0, offset, numberOfLEDs);
    }

    void threeSectionColor(Color firstSectionColor, Color secondSectionColor, Color thirdSectionColor) {
        if (!inverted) {
            CANDLE.setLEDs(firstSectionColor.getRed(), firstSectionColor.getGreen(), firstSectionColor.getBlue(), 0, offset, numberOfLEDs / 3);
            CANDLE.setLEDs(secondSectionColor.getRed(), secondSectionColor.getGreen(), secondSectionColor.getBlue(), 0, offset + (numberOfLEDs / 3), numberOfLEDs / 3);
            CANDLE.setLEDs(thirdSectionColor.getRed(), thirdSectionColor.getGreen(), thirdSectionColor.getBlue(), 0, offset + (2 * (numberOfLEDs / 3)), numberOfLEDs / 3);
        } else {
            CANDLE.setLEDs(thirdSectionColor.getRed(), thirdSectionColor.getGreen(), thirdSectionColor.getBlue(), 0, offset, numberOfLEDs / 3);
            CANDLE.setLEDs(secondSectionColor.getRed(), secondSectionColor.getGreen(), secondSectionColor.getBlue(), 0, offset + (numberOfLEDs / 3), numberOfLEDs / 3);
            CANDLE.setLEDs(firstSectionColor.getRed(), firstSectionColor.getGreen(), firstSectionColor.getBlue(), 0, offset + (2 * (numberOfLEDs / 3)), numberOfLEDs / 3);
        }
    }

    void animateFire(double brightness, double speed, double sparking, double cooling) {
        CANDLE.animate(
                new FireAnimation(
                        brightness,
                        speed,
                        this.numberOfLEDs,
                        sparking,
                        cooling,
                        inverted,
                        this.offset
                ),
                animationSlot
        );
    }

    void animateRainbow(double brightness, double speed) {
        CANDLE.animate(
                new RainbowAnimation(
                        brightness,
                        speed,
                        this.numberOfLEDs,
                        inverted,
                        this.offset
                ),
                animationSlot
        );
    }

    void animateColorFlow(Color color, double speed) {
        CANDLE.animate(new ColorFlowAnimation(
                        color.getRed(),
                        color.getGreen(),
                        color.getBlue(),
                        0,
                        speed,
                        this.numberOfLEDs,
                        inverted ? ColorFlowAnimation.Direction.Backward : ColorFlowAnimation.Direction.Forward,
                        this.offset
                ),
                animationSlot
        );
    }

    void animateLarson(Color color, double speed, LarsonAnimation.BounceMode mode, int size) {
        CANDLE.animate(
                new LarsonAnimation(
                        color.getRed(),
                        color.getGreen(),
                        color.getBlue(),
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
                        color.getRed(),
                        color.getGreen(),
                        color.getBlue(),
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
                        color.getRed(),
                        color.getGreen(),
                        color.getBlue(),
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
                        color.getRed(),
                        color.getGreen(),
                        color.getBlue(),
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
