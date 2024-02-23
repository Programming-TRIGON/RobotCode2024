package frc.trigon.robot.subsystems.ledstrip;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import frc.trigon.robot.constants.RobotConstants;

public class LEDStripConstants {
    private static final int CANDLE_ID = 0;
    private static final CANdle.LEDStripType STRIP_TYPE = CANdle.LEDStripType.RGB;
    private static final double BRIGHTNESS_SCALAR = 1;
    static final double
            MINIMUM_BATTERY_VOLTAGE = 10.5,
            LOW_BATTERY_FLASHING_SPEED = 1;
    static final CANdle CANDLE = new CANdle(CANDLE_ID, RobotConstants.CANIVORE_NAME);

    public static final LEDStrip
            LEFT_STRIP = new LEDStrip(8, 27, false),
            RIGHT_STRIP = new LEDStrip(27 + 8, 20, false),
            THE_REST = new LEDStrip(27 + 8 + 20, 100, false);
    public static final LEDStrip[] LED_STRIPS = {
            LEFT_STRIP,
            RIGHT_STRIP,
            THE_REST
    };

    static {
        final CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = STRIP_TYPE;
        config.brightnessScalar = BRIGHTNESS_SCALAR;
//        config.enableOptimizations = true;
        CANDLE.configAllSettings(config);
    }
}
