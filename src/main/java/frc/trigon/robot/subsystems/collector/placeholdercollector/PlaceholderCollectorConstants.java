package frc.trigon.robot.subsystems.collector.placeholdercollector;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;

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
    private static final FeedbackSensorSourceValue ENCODER_TYPE = FeedbackSensorSourceValue.RemoteCANcoder;
    private static final double
            MOTION_MAGIC_VELOCITY = 80,
            MOTION_MAGIC_ACCELERATION = 160,
            MOTION_MAGIC_JERK = 1600;
    private static final double
            ANGLE_P = 0.1,
            ANGLE_I = 0,
            ANGLE_D = 0,
            ANGLE_KA = 0,
            ANGLE_KS = 0,
            ANGLE_KV = 0;
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

        config.Slot0.kP = ANGLE_P;
        config.Slot0.kI = ANGLE_I;
        config.Slot0.kD = ANGLE_D;
        config.Slot0.kA = ANGLE_KA;
        config.Slot0.kS = ANGLE_KS;
        config.Slot0.kV = ANGLE_KV;

        config.Feedback.FeedbackRemoteSensorID = ENCODER_ID;
        config.Feedback.FeedbackRotorOffset = ENCODER_ID;
        config.Feedback.FeedbackSensorSource = ENCODER_TYPE;

        config.MotionMagic.MotionMagicCruiseVelocity = MOTION_MAGIC_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = MOTION_MAGIC_ACCELERATION;
        config.MotionMagic.MotionMagicJerk = MOTION_MAGIC_JERK;

        ANGLE_MOTOR.getConfigurator().apply(config);
        ANGLE_MOTOR.optimizeBusUtilization();
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
