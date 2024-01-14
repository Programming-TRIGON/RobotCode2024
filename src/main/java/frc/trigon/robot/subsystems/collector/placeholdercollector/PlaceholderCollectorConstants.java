package frc.trigon.robot.subsystems.collector.placeholdercollector;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class PlaceholderCollectorConstants {
    static final boolean FOC_ENABLED = true;
    private static final int
            COLLECTING_MOTOR_ID = 1,
            ANGLE_MOTOR_ID = 2,
            ENCODER_ID = 1;
    private static final InvertedValue
            COLLECTING_MOTOR_INVERTED = InvertedValue.CounterClockwise_Positive,
            ANGLE_MOTOR_INVERTED = InvertedValue.CounterClockwise_Positive;
    private static final NeutralModeValue
            COLLECTING_MOTOR_NEUTRAL_MODE = NeutralModeValue.Coast,
            ANGLE_MOTOR_NEUTRAL_MODE = NeutralModeValue.Coast;
    private static final AbsoluteSensorRangeValue ENCODER_RANGE = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    private static final SensorDirectionValue ENCODER_DIRECTION = SensorDirectionValue.CounterClockwise_Positive;
    private static final double ENCODER_OFFSET = 0;
    private static final CANcoder ENCODER = new CANcoder(ENCODER_ID);
    static final TalonFX
            COLLECTING_MOTOR = new TalonFX(COLLECTING_MOTOR_ID),
            ANGLE_MOTOR = new TalonFX(ANGLE_MOTOR_ID);
    static final StatusSignal<Double>
            COLLECTOR_POSITION_SIGNAL = ENCODER.getPosition(),
            COLLECTOR_VELOCITY_SIGNAL = ENCODER.getVelocity();

    static {
        configureCollectingMotor();
        configureAngleMotor();
        configureEncoder();
    }

    private static void configureCollectingMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = COLLECTING_MOTOR_INVERTED;
        config.MotorOutput.NeutralMode = COLLECTING_MOTOR_NEUTRAL_MODE;
        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;
        COLLECTING_MOTOR.getConfigurator().apply(config);
    }

    private static void configureAngleMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = ANGLE_MOTOR_INVERTED;
        config.MotorOutput.NeutralMode = ANGLE_MOTOR_NEUTRAL_MODE;
        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;
        ANGLE_MOTOR.getConfigurator().apply(config);
    }

    private static void configureEncoder() {
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.AbsoluteSensorRange = ENCODER_RANGE;
        config.MagnetSensor.SensorDirection = ENCODER_DIRECTION;
        config.MagnetSensor.MagnetOffset = ENCODER_OFFSET;
        ENCODER.getConfigurator().apply(config);

        COLLECTOR_POSITION_SIGNAL.setUpdateFrequency(100);
        COLLECTOR_VELOCITY_SIGNAL.setUpdateFrequency(100);
        ENCODER.optimizeBusUtilization();
    }
}
