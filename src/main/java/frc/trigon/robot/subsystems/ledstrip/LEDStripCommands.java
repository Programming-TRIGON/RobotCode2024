package frc.trigon.robot.subsystems.ledstrip;

import com.ctre.phoenix.led.ColorFlowAnimation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

public class LEDStripCommands {
    public static Command getSetLEDsCommand(Color color, LEDStrip ledStrip) {
        return new StartEndCommand(
                () -> getClearAnimationCommand(ledStrip),
                () -> ledStrip.setLEDs(color),
                ledStrip
        );
    }

    public static Command getAnimateFireCommand(double brightness, double speed, double sparking, double cooling, boolean backwards, LEDStrip ledStrip) {
        return new StartEndCommand(
                () -> ledStrip.animateFire(brightness, speed, sparking, cooling, backwards),
                () -> getClearAnimationCommand(ledStrip),
                ledStrip
        );
    }

    public static Command getAnimateRainbowCommand(double brightness, double speed, boolean backwards, LEDStrip ledStrip) {
        return new StartEndCommand(
                () -> ledStrip.animateRainbow(brightness, speed, backwards),
                () -> getClearAnimationCommand(ledStrip),
                ledStrip
        );
    }

    public static Command getAnimateColourFlowCommand(Color color, double speed, ColorFlowAnimation.Direction direction, LEDStrip ledStrip) {
        return new StartEndCommand(
                () -> ledStrip.animateColourFlow(color, speed, direction),
                () -> getClearAnimationCommand(ledStrip),
                ledStrip
        );
    }

    public static Command getClearAnimationCommand(LEDStrip ledStrip) {
        return new RunCommand(
                ledStrip::clearAnimation,
                ledStrip
        );
    }
}
