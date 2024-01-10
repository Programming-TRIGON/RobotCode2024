package frc.trigon.robot.subsystems.ledstrip;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDStrip extends SubsystemBase {
    private final static LEDStrip INSTANCE = new LEDStrip(0, 0);

    private int
            offset = 0,
            numLEDs = 0,
            r = 0,
            g = 0,
            b = 0;
    private static final CANdle candle = null;

    public static LEDStrip getInstance() {
        return INSTANCE;
    }

    public LEDStrip(int offset, int numLEDs) {
        this.offset = offset;
        this.numLEDs = numLEDs;
    }

    void setRGB(int r, int g, int b) {
        this.r = r;
        this.g = g;
        this.b = b;
    }

    void setLEDOn() {
        candle.setLEDs(r, g, b, 0, offset, numLEDs);
    }

    void setFireAnimation(double brightness, double speed, double sparking, double cooling, boolean backwards) {
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

    void setRainbowAnimation(double brightness, double speed, boolean backwards) {
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

    void setFlowAnimation(double speed, ColorFlowAnimation.Direction direction) {
        candle.animate(new ColorFlowAnimation(
                r,
                g,
                b,
                0,
                speed,
                this.numLEDs,
                direction,
                this.offset
        ));
    }

    void cutTheLights() {
        candle.setLEDs(0, 0, 0, 0, offset, numLEDs);
        candle.animate(null);
    }
}
