package frc.trigon.robot.subsystems.ledstrip;

import com.ctre.phoenix.led.ColorFlowAnimation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class LEDStripCommands {
    public static Command getSetLEDs(Color colour, LEDStrip ledStrip) {
        return new FunctionalCommand(
                () -> clearAnimation(ledStrip),
                () -> ledStrip.setLEDs(colour),
                (interrupted) -> {
                },
                () -> false,
                ledStrip
        );
    }

    public static Command getAnimateFire(double brightness, double speed, double sparking, double cooling, boolean backwards, LEDStrip ledStrip) {
        return new FunctionalCommand(
                () -> clearAnimation(ledStrip),
                () -> ledStrip.animateFire(brightness, speed, sparking, cooling, backwards),
                (interrupted) -> {
                },
                () -> false,
                ledStrip
        );
    }

    public static Command getAnimateRainbow(double brightness, double speed, boolean backwards, LEDStrip ledStrip) {
        return new FunctionalCommand(
                () -> clearAnimation(ledStrip),
                () -> ledStrip.animateRainbow(brightness, speed, backwards),
                (interrupted) -> {
                },
                () -> false,
                ledStrip
        );
    }

    public static Command getAnimateColourFlow(Color colour, double speed, ColorFlowAnimation.Direction direction, LEDStrip ledStrip) {
        return new FunctionalCommand(
                () -> clearAnimation(ledStrip),
                () -> ledStrip.animateColourFlow(colour, speed, direction),
                (interrupted) -> {
                },
                () -> false,
                ledStrip
        );
    }

    public static Command clearAnimation(LEDStrip ledStrip) {
        return new RunCommand(
                ledStrip::clearAnimation,
                ledStrip
        );
    }
}
