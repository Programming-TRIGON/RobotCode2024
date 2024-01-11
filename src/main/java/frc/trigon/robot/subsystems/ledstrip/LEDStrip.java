package frc.trigon.robot.subsystems.ledstrip;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDStrip extends SubsystemBase {
    private int
            offset = 0,
            numLEDs = 0;
    private static final CANdle candle = new CANdle(0);

    public LEDStrip(int offset, int numLEDs) {
        this.offset = offset;
        this.numLEDs = numLEDs;
    }

    void setLEDs(Color colour) {
        candle.setLEDs((int) colour.red, (int) colour.green, (int) colour.blue, 0, offset, numLEDs);
    }

    void animateFire(double brightness, double speed, double sparking, double cooling, boolean backwards) {
        candle.animate(
                new FireAnimation(
                        brightness,
                        speed,
                        this.numLEDs,
                        sparking,
                        cooling,
                        backwards,
                        this.offset
                )
        );
    }

    void animateRainbow(double brightness, double speed, boolean backwards) {
        candle.animate(
                new RainbowAnimation(
                        brightness,
                        speed,
                        this.numLEDs,
                        backwards,
                        this.offset
                )
        );
    }

    void animateColourFlow(Color colour, double speed, ColorFlowAnimation.Direction direction) {
        candle.animate(new ColorFlowAnimation(
                (int) colour.red,
                (int) colour.green,
                (int) colour.blue,
                0,
                speed,
                this.numLEDs,
                direction,
                this.offset
        ));
    }

    void clearAnimation() {
        candle.clearAnimation(0);
    }
}
