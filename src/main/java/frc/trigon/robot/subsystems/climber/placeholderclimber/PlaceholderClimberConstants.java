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
            RIGHT_MOTOR_ID = 0,
            LEFT_MOTOR_ID = 0,
            ENCODER_ID = 0;
    private static final InvertedValue
            RIGHT_MOTOR_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive,
            LEFT_MOTOR_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;
    private static final NeutralModeValue NEUTRAL_MODE_VALUE = NeutralModeValue.Coast;
    private static final SensorDirectionValue ENCODER_DIRECTION = SensorDirectionValue.CounterClockwise_Positive;
    private static final AbsoluteSensorRangeValue ENCODER_RANGE = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    private static final FeedbackSensorSourceValue ENCODER_SOURCE = FeedbackSensorSourceValue.RemoteCANcoder;
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
            RIGHT_MOTOR = new TalonFX(RIGHT_MOTOR_ID, RobotConstants.CANIVORE_NAME),
            LEFT_MOTOR = new TalonFX(LEFT_MOTOR_ID, RobotConstants.CANIVORE_NAME);

    static final StatusSignal<Double>
            RIGHT_MOTOR_POSITION_SIGNAL = ENCODER.getPosition(),
            RIGHT_MOTOR_VELOCITY_SIGNAL = ENCODER.getVelocity(),
            RIGHT_MOTOR_SETPOINT_SIGNAL = RIGHT_MOTOR.getClosedLoopReference(),
            RIGHT_MOTOR_VOLTAGE_SIGNAL = RIGHT_MOTOR.getMotorVoltage(),
            RIGHT_MOTOR_CURRENT_SIGNAL = RIGHT_MOTOR.getStatorCurrent(),
            LEFT_MOTOR_POSITION_SIGNAL = ENCODER.getPosition(),
            LEFT_MOTOR_VELOCITY_SIGNAL = ENCODER.getVelocity(),
            LEFT_MOTOR_SETPOINT_SIGNAL = LEFT_MOTOR.getClosedLoopReference(),
            LEFT_MOTOR_VOLTAGE_SIGNAL = LEFT_MOTOR.getMotorVoltage(),
            LEFT_MOTOR_CURRENT_SIGNAL = LEFT_MOTOR.getStatorCurrent();

    static {
        configureEncoder();
        updateStatusSignals();
        configureClimbingMotor(RIGHT_MOTOR, RIGHT_MOTOR_INVERTED_VALUE);
        configureClimbingMotor(LEFT_MOTOR, LEFT_MOTOR_INVERTED_VALUE);
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
        ENCODER.getConfigurator().apply(config);

        RIGHT_MOTOR_POSITION_SIGNAL.setUpdateFrequency(100);
        RIGHT_MOTOR_VELOCITY_SIGNAL.setUpdateFrequency(100);
        LEFT_MOTOR_POSITION_SIGNAL.setUpdateFrequency(100);
        LEFT_MOTOR_VELOCITY_SIGNAL.setUpdateFrequency(100);

        ENCODER.optimizeBusUtilization();
    }

    private static void updateStatusSignals() {
        RIGHT_MOTOR_SETPOINT_SIGNAL.setUpdateFrequency(100);
        RIGHT_MOTOR_VOLTAGE_SIGNAL.setUpdateFrequency(100);
        RIGHT_MOTOR_CURRENT_SIGNAL.setUpdateFrequency(100);
        LEFT_MOTOR_SETPOINT_SIGNAL.setUpdateFrequency(100);
        LEFT_MOTOR_VOLTAGE_SIGNAL.setUpdateFrequency(100);
        LEFT_MOTOR_CURRENT_SIGNAL.setUpdateFrequency(100);
    }
}
