package frc.trigon.robot.subsystems.ledstrip;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;

public class LEDStripConstants {
    private static final int CANDLE_ID = 0;
    public static final CANdle CANDLE = new CANdle(CANDLE_ID);
    private static final CANdle.LEDStripType STRIP_TYPE = CANdle.LEDStripType.RGB;

    static {
        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = STRIP_TYPE;
        config.enableOptimizations = true;
        CANDLE.configAllSettings(config);
    }
}
