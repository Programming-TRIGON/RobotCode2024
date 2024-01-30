package frc.trigon.robot.subsystems.ledstrip;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import frc.trigon.robot.constants.RobotConstants;

public class LEDStripConstants {
    private static final int CANDLE_ID = 0;
    private static final CANdle.LEDStripType STRIP_TYPE = CANdle.LEDStripType.RGB;
    private static final double BRIGHTNESS_SCALAR = 1;
    static final CANdle CANDLE = new CANdle(CANDLE_ID, RobotConstants.CANIVORE_NAME);

    public static final LEDStrip[] LED_STRIPS = {
            new LEDStrip(0, 60),
            new LEDStrip(60, 60),
    };

    static {
        final CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = STRIP_TYPE;
        config.brightnessScalar = BRIGHTNESS_SCALAR;
        config.enableOptimizations = true;
        CANDLE.configAllSettings(config);
    }
}
