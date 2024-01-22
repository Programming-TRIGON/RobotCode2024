package frc.trigon.robot.subsystems.elevator.placeholderelevator;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.elevator.ElevatorConstants;

public class PLACEHOLDERElevatorConstants {
    static final boolean FOC_ENABLED = true;
    private static final int
            MASTER_MOTOR_ID = 0,
            FOLLOWER_MOTOR_ID = 1,
            ENCODER_ID = 0;
    private static final NeutralModeValue
            MASTER_MOTOR_NEUTRAL_MODE_VALUE = NeutralModeValue.Brake,
            FOLLOWER_MOTOR_NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;
    private static final InvertedValue
            MASTER_MOTOR_INVERTED_VALUE = InvertedValue.Clockwise_Positive,
            FOLLOWER_MOTOR_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;
    private static final double
            P = 0,
            I = 0,
            D = 0,
            KS = 0,
            KV = 0,
            KG = 0,
            KA = 0;
    private static final double
            MOTION_MAGIC_CRUISE_VELOCITY = 10,
            MOTION_MAGIC_ACCELERATION = 10,
            MOTION_MAGIC_JERK = 10;
    private static final boolean FOLLOWER_MOTOR_OPPOSITE_DIRECTION = false;
    private static final AbsoluteSensorRangeValue ENCODER_SENSOR_RANGE_VALUE = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    private static final SensorDirectionValue ENCODER_SENSOR_DIRECTION_VALUE = SensorDirectionValue.Clockwise_Positive;
    private static final FeedbackSensorSourceValue ENCODER_TYPE = FeedbackSensorSourceValue.FusedCANcoder;
    private static final double ENCODER_OFFSET = 0;
    static final TalonFX
            MASTER_MOTOR = new TalonFX(MASTER_MOTOR_ID, RobotConstants.CANIVORE_NAME),
            FOLLOWER_MOTOR = new TalonFX(FOLLOWER_MOTOR_ID, RobotConstants.CANIVORE_NAME);
    static final CANcoder ENCODER = new CANcoder(ENCODER_ID, RobotConstants.CANIVORE_NAME);

    static final StatusSignal<Double>
            ENCODER_POSITION_STATUS_SIGNAL = ENCODER.getPosition(),
            ENCODER_VELOCITY_STATUS_SIGNAL = ENCODER.getVelocity(),
            MOTOR_VOLTAGE_STATUS_SIGNAL = MASTER_MOTOR.getMotorVoltage();

    static {
        configureEncoder();
        configureMasterMotor();
        configureFollowerMotor();
    }

    private static void configureMasterMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = MASTER_MOTOR_NEUTRAL_MODE_VALUE;
        config.MotorOutput.Inverted = MASTER_MOTOR_INVERTED_VALUE;

        config.Feedback.FeedbackRemoteSensorID = ENCODER_ID;
        config.Feedback.FeedbackSensorSource = ENCODER_TYPE;
        config.Feedback.RotorToSensorRatio = ElevatorConstants.GEAR_RATIO;

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

        MASTER_MOTOR.getConfigurator().apply(config);

        MOTOR_VOLTAGE_STATUS_SIGNAL.setUpdateFrequency(100);

        MASTER_MOTOR.optimizeBusUtilization();
    }

    private static void configureFollowerMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = FOLLOWER_MOTOR_NEUTRAL_MODE_VALUE;
        config.MotorOutput.Inverted = FOLLOWER_MOTOR_INVERTED_VALUE;

        FOLLOWER_MOTOR.getConfigurator().apply(config);
        FOLLOWER_MOTOR.optimizeBusUtilization();

        FOLLOWER_MOTOR.setControl(new Follower(MASTER_MOTOR_ID, FOLLOWER_MOTOR_OPPOSITE_DIRECTION));
    }

    private static void configureEncoder() {
        final CANcoderConfiguration config = new CANcoderConfiguration();

        config.MagnetSensor.AbsoluteSensorRange = ENCODER_SENSOR_RANGE_VALUE;
        config.MagnetSensor.SensorDirection = ENCODER_SENSOR_DIRECTION_VALUE;
        config.MagnetSensor.MagnetOffset = ENCODER_OFFSET;

        ENCODER.getConfigurator().apply(config);

        ENCODER_POSITION_STATUS_SIGNAL.setUpdateFrequency(100);
        ENCODER_VELOCITY_STATUS_SIGNAL.setUpdateFrequency(100);

        ENCODER.optimizeBusUtilization();
    }
}
