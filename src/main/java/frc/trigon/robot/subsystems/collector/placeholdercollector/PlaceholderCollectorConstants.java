package frc.trigon.robot.subsystems.collector.placeholdercollector;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.collector.CollectorConstants;

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
            ANGLE_MOTOR_NEUTRAL_MODE = NeutralModeValue.Brake;
    private static final AbsoluteSensorRangeValue ENCODER_RANGE = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    private static final SensorDirectionValue ENCODER_DIRECTION = SensorDirectionValue.CounterClockwise_Positive;
    private static final double ENCODER_OFFSET = 0;
    private static final FeedbackSensorSourceValue ENCODER_TYPE = FeedbackSensorSourceValue.FusedCANcoder;
    private static final double
            MOTION_MAGIC_VELOCITY = 200,
            MOTION_MAGIC_ACCELERATION = 160,
            MOTION_MAGIC_JERK = 1600;
    private static final double
            ANGLE_P = 0.1,
            ANGLE_I = 0,
            ANGLE_D = 0,
            ANGLE_KA = 0,
            ANGLE_KG = 0,
            ANGLE_KS = 0,
            ANGLE_KV = 0;
    private static final CANcoder ENCODER = new CANcoder(ENCODER_ID, RobotConstants.CANIVORE_NAME);
    static final TalonFX
            COLLECTING_MOTOR = new TalonFX(COLLECTING_MOTOR_ID, RobotConstants.CANIVORE_NAME),
            ANGLE_MOTOR = new TalonFX(ANGLE_MOTOR_ID, RobotConstants.CANIVORE_NAME);

    static final StatusSignal<Double>
            ANGLE_POSITION_SIGNAL = ENCODER.getPosition(),
            ANGLE_VELOCITY_SIGNAL = ENCODER.getVelocity(),
            ANGLE_MOTOR_CURRENT_SIGNAL = ANGLE_MOTOR.getStatorCurrent(),
            ANGLE_MOTOR_VOLTAGE_SIGNAL = ANGLE_MOTOR.getMotorVoltage(),
            ANGLE_MOTOR_PROFILED_SETPOINT_SIGNAL = ANGLE_MOTOR.getClosedLoopReference(),
            COLLECTION_MOTOR_VELOCITY_SIGNAL = COLLECTING_MOTOR.getMotorVoltage(),
            COLLECTION_MOTOR_CURRENT_SIGNAL = COLLECTING_MOTOR.getStatorCurrent(),
            COLLECTION_MOTOR_VOLTAGE_SIGNAL = COLLECTING_MOTOR.getMotorVoltage();


    static {
        configureEncoder();
        configureCollectingMotor();
        configureAngleMotor();
    }

    private static void configureCollectingMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = COLLECTING_MOTOR_INVERTED;
        config.MotorOutput.NeutralMode = COLLECTING_MOTOR_NEUTRAL_MODE;
        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;
        config.Feedback.RotorToSensorRatio = CollectorConstants.COLLECTION_MOTOR_GEAR_RATIO;

        COLLECTION_MOTOR_VELOCITY_SIGNAL.setUpdateFrequency(100);
        COLLECTION_MOTOR_CURRENT_SIGNAL.setUpdateFrequency(100);
        COLLECTION_MOTOR_VOLTAGE_SIGNAL.setUpdateFrequency(100);

        COLLECTING_MOTOR.getConfigurator().apply(config);
        COLLECTING_MOTOR.optimizeBusUtilization();
    }

    private static void configureAngleMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = ANGLE_MOTOR_INVERTED;
        config.MotorOutput.NeutralMode = ANGLE_MOTOR_NEUTRAL_MODE;
        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.Slot0.kP = ANGLE_P;
        config.Slot0.kI = ANGLE_I;
        config.Slot0.kD = ANGLE_D;
        config.Slot0.kA = ANGLE_KA;
        config.Slot0.kG = ANGLE_KG;
        config.Slot0.kS = ANGLE_KS;
        config.Slot0.kV = ANGLE_KV;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        config.Feedback.FeedbackRemoteSensorID = ENCODER_ID;
        config.Feedback.FeedbackSensorSource = ENCODER_TYPE;
        config.Feedback.RotorToSensorRatio = CollectorConstants.ANGLE_MOTOR_GEAR_RATIO;

        config.MotionMagic.MotionMagicCruiseVelocity = MOTION_MAGIC_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = MOTION_MAGIC_ACCELERATION;
        config.MotionMagic.MotionMagicJerk = MOTION_MAGIC_JERK;

        ANGLE_MOTOR_CURRENT_SIGNAL.setUpdateFrequency(100);
        ANGLE_MOTOR_VOLTAGE_SIGNAL.setUpdateFrequency(100);
        ANGLE_MOTOR_PROFILED_SETPOINT_SIGNAL.setUpdateFrequency(100);

        ANGLE_MOTOR.getConfigurator().apply(config);
        ANGLE_MOTOR.optimizeBusUtilization();
    }

    private static void configureEncoder() {
        final CANcoderConfiguration config = new CANcoderConfiguration();

        config.MagnetSensor.AbsoluteSensorRange = ENCODER_RANGE;
        config.MagnetSensor.SensorDirection = ENCODER_DIRECTION;
        config.MagnetSensor.MagnetOffset = ENCODER_OFFSET;

        ENCODER.getConfigurator().apply(config);

        ANGLE_POSITION_SIGNAL.setUpdateFrequency(100);
        ANGLE_VELOCITY_SIGNAL.setUpdateFrequency(100);
        ENCODER.optimizeBusUtilization();
    }
}
