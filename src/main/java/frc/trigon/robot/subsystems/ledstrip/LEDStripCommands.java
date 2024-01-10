package frc.trigon.robot.subsystems.ledstrip;

import com.ctre.phoenix.led.ColorFlowAnimation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class LEDStripCommands {

    public static Command getSetLEDs(int r, int g, int b, LEDStrip ledStrip) {
        return new FunctionalCommand(
                () -> getSetRGBValues(r, g, b, ledStrip).alongWith(getCutTheLights(ledStrip)),
                () -> getTurnLEDOn(ledStrip),
                (interrupted) -> {
                },
                () -> false,
                ledStrip
        );
    }

    private static Command getSetRGBValues(int r, int g, int b, LEDStrip ledStrip) {
        return new RunCommand(
                () -> ledStrip.setRGB(r, g, b),
                ledStrip
        );
    }

    private static Command getTurnLEDOn(LEDStrip ledStrip) {
        return new RunCommand(
                ledStrip::setLEDOn,
                ledStrip
        );
    }

    public static Command getSetFireAnimation(double brightness, double speed, double sparking, double cooling, boolean backwards, LEDStrip ledStrip) {
        return new FunctionalCommand(
                () -> getCutTheLights(ledStrip),
                () -> ledStrip.setFireAnimation(brightness, speed, sparking, cooling, backwards),
                (interrupted) -> {
                },
                () -> false,
                ledStrip
        );
    }

    public static Command getSetRainbowAnimation(double brightness, double speed, boolean backwards, LEDStrip ledStrip) {
        return new FunctionalCommand(
                () -> getCutTheLights(ledStrip),
                () -> ledStrip.setRainbowAnimation(brightness, speed, backwards),
                (interrupted) -> {
                },
                () -> false,
                ledStrip
        );
    }

    public static Command getSetFlowAnimation(int r, int g, int b, double speed, ColorFlowAnimation.Direction direction, LEDStrip ledStrip) {
        return new FunctionalCommand(
                () -> getSetRGBValues(r, g, b, ledStrip).alongWith(getCutTheLights(ledStrip)),
                () -> ledStrip.setFlowAnimation(speed, direction),
                (interrupted) -> {
                },
                () -> false,
                ledStrip
        );
    }

    public static Command getCutTheLights(LEDStrip ledStrip) {
        return new RunCommand(
                ledStrip::cutTheLights,
                ledStrip
        );
    }
}
