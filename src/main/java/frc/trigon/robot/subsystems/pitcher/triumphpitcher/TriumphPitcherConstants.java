package frc.trigon.robot.subsystems.pitcher.triumphpitcher;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.pitcher.PitcherConstants;

public class TriumphPitcherConstants {
    static final boolean FOC_ENABLED = true;
    private static final int
            MOTOR_ID = 11,
            ENCODER_ID = 11;
    private static final NeutralModeValue NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;
    private static final InvertedValue INVERTED_VALUE = InvertedValue.Clockwise_Positive;
    private static final SensorDirectionValue SENSOR_DIRECTION_VALUE = SensorDirectionValue.CounterClockwise_Positive;
    private static final AbsoluteSensorRangeValue ABSOLUTE_SENSOR_RANGE_VALUE = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    private static final FeedbackSensorSourceValue ENCODER_TYPE = FeedbackSensorSourceValue.FusedCANcoder;
    private static final double OFFSET = -0.639892578 + 0.5;
    private static final double
            P = 0,
            I = 0,
            D = 0,
            KG = 0.86752,
            KV = 4.0035,
            KA = 3.9045,
            KS = 1.0412;
    private static final double
            MAXIMUM_VELOCITY = 0.5,
            MAXIMUM_ACCELERATION = 0.5;
    private static final CANcoder ENCODER = new CANcoder(ENCODER_ID, RobotConstants.CANIVORE_NAME);
    static final TalonFX MOTOR = new TalonFX(MOTOR_ID, RobotConstants.CANIVORE_NAME);

    static final StatusSignal<Double>
            POSITION_SIGNAL = ENCODER.getPosition().clone(),
            VELOCITY_SIGNAL = ENCODER.getVelocity().clone(),
            VOLTAGE_SIGNAL = MOTOR.getMotorVoltage().clone(),
            PROFILED_SETPOINT_SIGNAL = MOTOR.getClosedLoopReference().clone();

    static {
        configureEncoder();
        configureMotor();
    }

    private static void configureEncoder() {
        final CANcoderConfiguration config = new CANcoderConfiguration();

        config.MagnetSensor.MagnetOffset = OFFSET;
        config.MagnetSensor.SensorDirection = SENSOR_DIRECTION_VALUE;
        config.MagnetSensor.AbsoluteSensorRange = ABSOLUTE_SENSOR_RANGE_VALUE;

        ENCODER.getConfigurator().apply(config);

        POSITION_SIGNAL.setUpdateFrequency(100);
        VELOCITY_SIGNAL.setUpdateFrequency(100);
        ENCODER.optimizeBusUtilization();
    }

    private static void configureMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnConfig = false;
        config.Audio.BeepOnBoot = false;

        config.Slot0.kP = P;
        config.Slot0.kI = I;
        config.Slot0.kD = D;
        config.Slot0.kG = KG;
        config.Slot0.kV = KV;
        config.Slot0.kA = KA;
        config.Slot0.kS = KS;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.ClosedLoopGeneral.ContinuousWrap = true;

        config.MotorOutput.Inverted = INVERTED_VALUE;
        config.MotorOutput.NeutralMode = NEUTRAL_MODE_VALUE;

        config.Feedback.RotorToSensorRatio = PitcherConstants.GEAR_RATIO;
        config.Feedback.FeedbackRemoteSensorID = ENCODER_ID;
        config.Feedback.FeedbackSensorSource = ENCODER_TYPE;

        config.MotionMagic.MotionMagicAcceleration = MAXIMUM_ACCELERATION;
        config.MotionMagic.MotionMagicCruiseVelocity = MAXIMUM_VELOCITY;

        MOTOR.getConfigurator().apply(config);

        VOLTAGE_SIGNAL.setUpdateFrequency(100);
        PROFILED_SETPOINT_SIGNAL.setUpdateFrequency(100);
        MOTOR.optimizeBusUtilization();
    }
}
