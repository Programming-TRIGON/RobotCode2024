package frc.trigon.robot.subsystems.elevator.placeholderelevator;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import frc.trigon.robot.constants.RobotConstants;

public class PLACEHOLDERElevatorConstants {
    static final boolean FOC_ENABLED = true;
    private static final int
            MASTER_MOTOR_ID = 0,
            FOLLOWER_MOTOR_ID = 1,
            ENCODER_ID = 0;
    private static final NeutralModeValue MOTOR_NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;
    private static final InvertedValue MOTOR_INVERTED_VALUE = InvertedValue.Clockwise_Positive;
    private static final double
            ENCODER_SENSOR_TO_MECHANISM_RATIO = 1,
            ENCODER_ROTOR_OFFSET = 0,
            P = 0,
            I = 0,
            D = 0,
            KS = 0,
            KV = 0,
            KG = 0,
            KA = 0,
            MOTION_MAGIC_CRUISE_VELOCITY = 10,
            MOTION_MAGIC_ACCELERATION = 10,
            MOTION_MAGIC_JERK = 10;
    private static final AbsoluteSensorRangeValue ENCODER_SENSOR_RANGE_VALUE = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    private static final SensorDirectionValue ENCODER_SENSOR_DIRECTION_VALUE = SensorDirectionValue.Clockwise_Positive;
    private static final double ENCODER_MAGNET_OFFSET = 0;
    static final TalonFX
            MASTER_MOTOR = new TalonFX(MASTER_MOTOR_ID, RobotConstants.CANIVORE_NAME),
            FOLLOWER_MOTOR = new TalonFX(FOLLOWER_MOTOR_ID, RobotConstants.CANIVORE_NAME);
    static final CANcoder ENCODER = new CANcoder(ENCODER_ID, RobotConstants.CANIVORE_NAME);

    static final StatusSignal<Double>
            ENCODER_POSITION_STATUS_SIGNAL = ENCODER.getPosition(),
            ENCODER_VELOCITY_STATUS_SIGNAL = ENCODER.getVelocity(),
            MASTER_MOTOR_VOLTAGE_STATUS_SIGNAL = MASTER_MOTOR.getMotorVoltage(),
            FOLLOWER_MOTOR_VOLTAGE_STATUS_SIGNAL = FOLLOWER_MOTOR.getMotorVoltage();

    static {
        configureMotors();
        configureEncoder();
    }

    private static void configureMotors() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = MOTOR_NEUTRAL_MODE_VALUE;
        config.MotorOutput.Inverted = MOTOR_INVERTED_VALUE;

        config.Feedback.FeedbackRemoteSensorID = ENCODER_ID;
        config.Feedback.SensorToMechanismRatio = ENCODER_SENSOR_TO_MECHANISM_RATIO;
        config.Feedback.FeedbackRotorOffset = ENCODER_ROTOR_OFFSET;

        config.Slot0.kP = P;
        config.Slot0.kI = I;
        config.Slot0.kD = D;
        config.Slot0.kS = KS;
        config.Slot0.kV = KV;
        config.Slot0.kG = KG;
        config.Slot0.kA = KA;

        config.MotionMagic.MotionMagicCruiseVelocity = MOTION_MAGIC_CRUISE_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = MOTION_MAGIC_ACCELERATION;
        config.MotionMagic.MotionMagicJerk = MOTION_MAGIC_JERK;

        applyMotorConfigs(config);
    }

    private static void configureEncoder() {
        final CANcoderConfiguration config = new CANcoderConfiguration();

        config.MagnetSensor.AbsoluteSensorRange = ENCODER_SENSOR_RANGE_VALUE;
        config.MagnetSensor.SensorDirection = ENCODER_SENSOR_DIRECTION_VALUE;
        config.MagnetSensor.MagnetOffset = ENCODER_MAGNET_OFFSET;

        ENCODER.getConfigurator().apply(config);
    }

    private static void applyMotorConfigs(TalonFXConfiguration config) {
        MASTER_MOTOR.getConfigurator().apply(config);
        FOLLOWER_MOTOR.getConfigurator().apply(config);
    }
}
