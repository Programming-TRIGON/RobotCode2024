package frc.trigon.robot.subsystems.ledstrip;

import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

public class LEDStripCommands {
    public static Command getSetLEDsCommand(Color color, LEDStrip ledStrip) {
        return new StartEndCommand(
                () -> ledStrip.setLEDs(color),
                ledStrip::clearAnimation,
                ledStrip
        );
    }

    public static Command getAnimateFireCommand(double brightness, double speed, double sparking, double cooling, boolean backwards, LEDStrip ledStrip) {
        return new StartEndCommand(
                () -> ledStrip.animateFire(brightness, speed, sparking, cooling, backwards),
                ledStrip::clearAnimation,
                ledStrip
        );
    }

    public static Command getAnimateRainbowCommand(double brightness, double speed, boolean backwards, LEDStrip ledStrip) {
        return new StartEndCommand(
                () -> ledStrip.animateRainbow(brightness, speed, backwards),
                ledStrip::clearAnimation,
                ledStrip
        );
    }

    public static Command getAnimateColourFlowCommand(Color color, double speed, ColorFlowAnimation.Direction direction, LEDStrip ledStrip) {
        return new StartEndCommand(
                () -> ledStrip.animateColourFlow(color, speed, direction),
                ledStrip::clearAnimation,
                ledStrip
        );
    }

    public static Command getAnimateLarsonCommand(Color color, double speed, LarsonAnimation.BounceMode mode, int size, LEDStrip ledStrip) {
        return new StartEndCommand(
                () -> ledStrip.animateLarson(color, speed, mode, size),
                ledStrip::clearAnimation,
                ledStrip
        );
    }

    public static Command getAnimateRGBFadeCommand(double brightness, double speed, LEDStrip ledStrip) {
        return new StartEndCommand(
                () -> ledStrip.animateRGBFade(brightness, speed),
                ledStrip::clearAnimation,
                ledStrip
        );
    }

    public static Command getAnimateSingleFadeCommand(Color color, double speed, LEDStrip ledStrip) {
        return new StartEndCommand(
                () -> ledStrip.animateSingleFade(color, speed),
                ledStrip::clearAnimation,
                ledStrip
        );
    }

    public static Command getAnimateTwinkleCommand(Color color, double speed, TwinkleAnimation.TwinklePercent divider, LEDStrip ledStrip) {
        return new StartEndCommand(
                () -> ledStrip.animateTwinkle(color, speed, divider),
                ledStrip::clearAnimation,
                ledStrip
        );
    }

    public static Command getAnimateStrobeCommand(Color color, double speed, LEDStrip ledStrip) {
        return new StartEndCommand(
                () -> ledStrip.animateStrobe(color, speed),
                ledStrip::clearAnimation,
                ledStrip
        );
    }

    public static Command getClearAnimationCommand(LEDStrip ledStrip) {
        return new StartEndCommand(
                ledStrip::clearAnimation,
                ledStrip::clearAnimation,
                ledStrip
        );
    }
}
