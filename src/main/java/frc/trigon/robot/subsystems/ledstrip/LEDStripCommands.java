package frc.trigon.robot.subsystems.ledstrip;

import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

import java.util.Arrays;
import java.util.function.Consumer;

public class LEDStripCommands {
    public static Command getStaticColorCommand(Color color, LEDStrip... ledStrips) {
        return new StartEndCommand(
                runForEach((strip) -> strip.staticColor(color), ledStrips),
                () -> {
                },
                ledStrips
        );
    }

    public static Command getAnimateFireCommand(double brightness, double speed, double sparking, double cooling, boolean backwards, LEDStrip... ledStrips) {
        return new StartEndCommand(
                runForEach((strip) -> strip.animateFire(brightness, speed, sparking, cooling, backwards), ledStrips),
                runForEach(LEDStrip::clearAnimation, ledStrips),
                ledStrips
        );
    }

    public static Command getAnimateRainbowCommand(double brightness, double speed, boolean backwards, LEDStrip... ledStrips) {
        return new StartEndCommand(
                runForEach((strip) -> strip.animateRainbow(brightness, speed, backwards), ledStrips),
                runForEach(LEDStrip::clearAnimation, ledStrips),
                ledStrips
        );
    }

    public static Command getAnimateColorFlowCommand(Color color, double speed, ColorFlowAnimation.Direction direction, LEDStrip... ledStrips) {
        return new StartEndCommand(
                runForEach((strip) -> strip.animateColorFlow(color, speed, direction), ledStrips),
                runForEach(LEDStrip::clearAnimation, ledStrips),
                ledStrips
        );
    }

    public static Command getAnimateLarsonCommand(Color color, double speed, LarsonAnimation.BounceMode mode, int size, LEDStrip... ledStrips) {
        return new StartEndCommand(
                runForEach((strip) -> strip.animateLarson(color, speed, mode, size), ledStrips),
                runForEach(LEDStrip::clearAnimation, ledStrips),
                ledStrips
        );
    }

    public static Command getAnimateRGBFadeCommand(double brightness, double speed, LEDStrip... ledStrips) {
        return new StartEndCommand(
                runForEach((strip) -> strip.animateRGBFade(brightness, speed), ledStrips),
                runForEach(LEDStrip::clearAnimation, ledStrips),
                ledStrips
        );
    }

    public static Command getAnimateSingleFadeCommand(Color color, double speed, LEDStrip... ledStrips) {
        return new StartEndCommand(
                runForEach((strip) -> strip.animateSingleFade(color, speed), ledStrips),
                runForEach(LEDStrip::clearAnimation, ledStrips),
                ledStrips
        );
    }

    public static Command getAnimateTwinkleCommand(Color color, double speed, TwinkleAnimation.TwinklePercent divider, LEDStrip... ledStrips) {
        return new StartEndCommand(
                runForEach((strip) -> strip.animateTwinkle(color, speed, divider), ledStrips),
                runForEach(LEDStrip::clearAnimation, ledStrips),
                ledStrips
        );
    }

    public static Command getAnimateStrobeCommand(Color color, double speed, LEDStrip... ledStrips) {
        return new StartEndCommand(
                runForEach((strip) -> strip.animateStrobe(color, speed), ledStrips),
                runForEach(LEDStrip::clearAnimation, ledStrips),
                ledStrips
        );
    }

    public static Command getClearAnimationCommand(LEDStrip... ledStrips) {
        return new StartEndCommand(
                runForEach(LEDStrip::clearAnimation, ledStrips),
                () -> {},
                ledStrips
        );
    }

    private static Runnable runForEach(Consumer<LEDStrip> toRun, LEDStrip... ledStrips) {
        return () -> Arrays.stream(ledStrips).forEach(toRun);
    }
}
