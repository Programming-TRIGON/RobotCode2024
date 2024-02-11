package frc.trigon.robot.subsystems.climber.triumphclimber;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.climber.ClimberConstants;

public class TriumphClimberConstants {
    static final boolean ENABLE_FOC = true;
    private static final int
            MASTER_MOTOR_ID = 0,
            FOLLOWER_MOTOR_ID = 0,
            ENCODER_ID = 0;
    private static final InvertedValue
            MASTER_MOTOR_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive,
            FOLLOWER_MOTOR_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;
    private static final boolean FOLLOWER_MOTOR_OPPOSITE_DIRECTION = false;
    private static final NeutralModeValue NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;
    private static final SensorDirectionValue ENCODER_DIRECTION = SensorDirectionValue.CounterClockwise_Positive;
    private static final AbsoluteSensorRangeValue ENCODER_RANGE = AbsoluteSensorRangeValue.Unsigned_0To1;
    private static final FeedbackSensorSourceValue ENCODER_SOURCE = FeedbackSensorSourceValue.FusedCANcoder;
    private static final double ENCODER_OFFSET_REVOLUTIONS = 0;
    static final int
            NON_CLIMBING_SLOT = 0,
            CLIMBING_SLOT = 1;
    private static final double
            NON_CLIMBING_P = 30,
            NON_CLIMBING_I = 0,
            NON_CLIMBING_D = 0,
            NON_CLIMBING_KS = 0.030713,
            NON_CLIMBING_KG = 0.009791,
            NON_CLIMBING_KV = 2.385,
            NON_CLIMBING_KA = 0.049261;
    private static final double
            CLIMBING_P = 30,
            CLIMBING_I = 0,
            CLIMBING_D = 0,
            CLIMBING_KS = 0.030713,
            CLIMBING_KG = 0.009791,
            CLIMBING_KV = 2.385,
            CLIMBING_KA = 0.049261;
    static final double
            MAX_NON_CLIMBING_VELOCITY = 18,
            MAX_NON_CLIMBING_ACCELERATION = 18,
            MAX_CLIMBING_VELOCITY = 18,
            MAX_CLIMBING_ACCELERATION = 18;
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
        configureMasterClimbingMotor();
        configureFollowerClimbingMotor();
    }

    private static void configureMasterClimbingMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = MASTER_MOTOR_INVERTED_VALUE;
        config.MotorOutput.NeutralMode = NEUTRAL_MODE_VALUE;
        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.Slot0.kP = NON_CLIMBING_P;
        config.Slot0.kI = NON_CLIMBING_I;
        config.Slot0.kD = NON_CLIMBING_D;
        config.Slot0.kS = NON_CLIMBING_KS;
        config.Slot0.kG = NON_CLIMBING_KG;
        config.Slot0.kV = NON_CLIMBING_KV;
        config.Slot0.kA = NON_CLIMBING_KA;

        config.Slot1.kP = CLIMBING_P;
        config.Slot1.kI = CLIMBING_I;
        config.Slot1.kD = CLIMBING_D;
        config.Slot1.kS = CLIMBING_KS;
        config.Slot1.kG = CLIMBING_KG;
        config.Slot1.kV = CLIMBING_KV;
        config.Slot1.kA = CLIMBING_KA;

        config.Feedback.RotorToSensorRatio = ClimberConstants.GEAR_RATIO;
        config.Feedback.FeedbackRemoteSensorID = ENCODER_ID;
        config.Feedback.FeedbackSensorSource = ENCODER_SOURCE;

        MASTER_MOTOR.getConfigurator().apply(config);

        MOTOR_SETPOINT_SIGNAL.setUpdateFrequency(100);
        MOTOR_VOLTAGE_SIGNAL.setUpdateFrequency(100);
        MOTOR_CURRENT_SIGNAL.setUpdateFrequency(100);
        MASTER_MOTOR.optimizeBusUtilization();
    }

    private static void configureFollowerClimbingMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = FOLLOWER_MOTOR_INVERTED_VALUE;
        config.MotorOutput.NeutralMode = NEUTRAL_MODE_VALUE;
        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        FOLLOWER_MOTOR.getConfigurator().apply(config);

        FOLLOWER_MOTOR.optimizeBusUtilization();

        final Follower followerRequest = new Follower(MASTER_MOTOR_ID, FOLLOWER_MOTOR_OPPOSITE_DIRECTION);
        FOLLOWER_MOTOR.setControl(followerRequest);
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

}
