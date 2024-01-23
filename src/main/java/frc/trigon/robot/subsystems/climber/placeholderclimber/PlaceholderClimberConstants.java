package frc.trigon.robot.subsystems.climber.placeholderclimber;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.climber.ClimberConstants;

public class PlaceholderClimberConstants {
    static final boolean ENABLE_FOC = true;
    private static final int
            MASTER_MOTOR_ID = 0,
            FOLLOWER_MOTOR_ID = 0,
            ENCODER_ID = 0;
    private static final InvertedValue
            MASTER_MOTOR_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive,
            FOLLOWER_MOTOR_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;
    private static final NeutralModeValue NEUTRAL_MODE_VALUE = NeutralModeValue.Coast;
    private static final SensorDirectionValue ENCODER_DIRECTION = SensorDirectionValue.CounterClockwise_Positive;
    private static final AbsoluteSensorRangeValue ENCODER_RANGE = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    private static final FeedbackSensorSourceValue ENCODER_SOURCE = FeedbackSensorSourceValue.FusedCANcoder;
    private static final double ENCODER_OFFSET_REVOLUTIONS = 0;
    private static final double
            MOTION_MAGIC_ACCELERATION = 200,
            MOTION_MAGIC_VELOCITY = 160,
            MOTION_MAGIC_JERK = 1600;
    private static final double
            P = 0,
            I = 0,
            D = 0,
            KA = 0,
            KG = 0,
            KS = 0,
            KV = 0;

    private static final CANcoder ENCODER = new CANcoder(ENCODER_ID, RobotConstants.CANIVORE_NAME);
    static final TalonFX
            MASTER_MOTOR = new TalonFX(MASTER_MOTOR_ID, RobotConstants.CANIVORE_NAME),
            FOLLOWER_MOTOR = new TalonFX(FOLLOWER_MOTOR_ID, RobotConstants.CANIVORE_NAME);

    static final StatusSignal<Double>
            ENCODER_POSITION_SIGNAL = ENCODER.getPosition(),
            ENCODER_VELOCITY_SIGNAL = ENCODER.getVelocity(),
            MOTOR_SETPOINT_SIGNAL = MASTER_MOTOR.getClosedLoopReference(),
            MOTOR_VOLTAGE_SIGNAL = MASTER_MOTOR.getMotorVoltage(),
            MOTOR_CURRENT_SIGNAL = MASTER_MOTOR.getStatorCurrent();

    static {
        configureEncoder();
        updateStatusSignals();
        configureClimbingMotor(MASTER_MOTOR, MASTER_MOTOR_INVERTED_VALUE);
        configureClimbingMotor(FOLLOWER_MOTOR, FOLLOWER_MOTOR_INVERTED_VALUE);
    }

    private static void configureClimbingMotor(TalonFX motor, InvertedValue invertedValue) {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = invertedValue;
        config.MotorOutput.NeutralMode = NEUTRAL_MODE_VALUE;
        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.Slot0.kP = P;
        config.Slot0.kI = I;
        config.Slot0.kD = D;
        config.Slot0.kA = KA;
        config.Slot0.kG = KG;
        config.Slot0.kS = KS;
        config.Slot0.kV = KV;

        config.MotionMagic.MotionMagicCruiseVelocity = MOTION_MAGIC_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = MOTION_MAGIC_ACCELERATION;
        config.MotionMagic.MotionMagicJerk = MOTION_MAGIC_JERK;

        config.Feedback.SensorToMechanismRatio = ClimberConstants.GEAR_RATIO;
        config.Feedback.FeedbackRemoteSensorID = ENCODER_ID;
        config.Feedback.FeedbackSensorSource = ENCODER_SOURCE;

        motor.getConfigurator().apply(config);

        motor.optimizeBusUtilization();
    }

    private static void configureEncoder() {
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.SensorDirection = ENCODER_DIRECTION;
        config.MagnetSensor.AbsoluteSensorRange = ENCODER_RANGE;
        config.MagnetSensor.MagnetOffset = ENCODER_OFFSET_REVOLUTIONS;
        ENCODER.getConfigurator().apply(config);

        ENCODER_POSITION_SIGNAL.setUpdateFrequency(100);
        ENCODER_VELOCITY_SIGNAL.setUpdateFrequency(100);

        ENCODER.optimizeBusUtilization();
    }

    private static void updateStatusSignals() {
        MOTOR_SETPOINT_SIGNAL.setUpdateFrequency(100);
        MOTOR_VOLTAGE_SIGNAL.setUpdateFrequency(100);
        MOTOR_CURRENT_SIGNAL.setUpdateFrequency(100);
    }
}
