package frc.trigon.robot.subsystems.climber.placeholderclimber;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.DifferentialMechanism;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.swerve.Swerve;

public class PlaceholderClimberConstants {
    static final boolean ENABLE_FOC = true;
    private static final int
            RIGHT_MOTOR_ID = 0,
            LEFT_MOTOR_ID = 0;
    private static final InvertedValue
            RIGHT_MOTOR_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive,
            LEFT_MOTOR_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;
    private static final NeutralModeValue NEUTRAL_MODE_VALUE = NeutralModeValue.Coast;
    private static final boolean ARE_MOTORS_ALIGNED = true;
    private static final double
            MOTION_MAGIC_ACCELERATION = 200,
            MOTION_MAGIC_VELOCITY = 160,
            MOTION_MAGIC_JERK = 1600;
    private static final double
            AVERAGE_P = 0,
            AVERAGE_I = 0,
            AVERAGE_D = 0,
            AVERAGE_KA = 0,
            AVERAGE_KG = 0,
            AVERAGE_KS = 0,
            AVERAGE_KV = 0,
            DIFFERENTIAL_P = 0,
            DIFFERENTIAL_I = 0,
            DIFFERENTIAL_D = 0,
            DIFFERENTIAL_KA = 0,
            DIFFERENTIAL_KG = 0,
            DIFFERENTIAL_KS = 0,
            DIFFERENTIAL_KV = 0;

    static final TalonFX
            RIGHT_MOTOR = new TalonFX(RIGHT_MOTOR_ID, RobotConstants.CANIVORE_NAME),
            LEFT_MOTOR = new TalonFX(LEFT_MOTOR_ID, RobotConstants.CANIVORE_NAME);
    static final Pigeon2 GYRO = Swerve.getInstance().getConstants().getPigeon().get();
    static final DifferentialMechanism DIFFERENTIAL_MECHANISM = new DifferentialMechanism(RIGHT_MOTOR, LEFT_MOTOR, ARE_MOTORS_ALIGNED, GYRO, DifferentialMechanism.DifferentialPigeon2Source.Roll);
    static final StatusSignal<Double>
            RIGHT_MOTOR_POSITION_SIGNAL = RIGHT_MOTOR.getPosition(),
            RIGHT_MOTOR_VELOCITY_SIGNAL = RIGHT_MOTOR.getVelocity(),
            RIGHT_MOTOR_SETPOINT_SIGNAL = RIGHT_MOTOR.getClosedLoopReference(),
            RIGHT_MOTOR_VOLTAGE_SIGNAL = RIGHT_MOTOR.getMotorVoltage(),
            RIGHT_MOTOR_CURRENT_SIGNAL = RIGHT_MOTOR.getStatorCurrent(),
            LEFT_MOTOR_POSITION_SIGNAL = LEFT_MOTOR.getPosition(),
            LEFT_MOTOR_VELOCITY_SIGNAL = LEFT_MOTOR.getVelocity(),
            LEFT_MOTOR_SETPOINT_SIGNAL = LEFT_MOTOR.getClosedLoopReference(),
            LEFT_MOTOR_VOLTAGE_SIGNAL = LEFT_MOTOR.getMotorVoltage(),
            LEFT_MOTOR_CURRENT_SIGNAL = LEFT_MOTOR.getStatorCurrent();

    static {
        configureRightMotor();
        configureLeftMotor();
    }

    private static void configureRightMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = RIGHT_MOTOR_INVERTED_VALUE;
        config.MotorOutput.NeutralMode = NEUTRAL_MODE_VALUE;
        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.Slot0.kP = AVERAGE_P;
        config.Slot0.kI = AVERAGE_I;
        config.Slot0.kD = AVERAGE_D;
        config.Slot0.kA = AVERAGE_KA;
        config.Slot0.kG = AVERAGE_KG;
        config.Slot0.kS = AVERAGE_KS;
        config.Slot0.kV = AVERAGE_KV;

        config.Slot1.kP = DIFFERENTIAL_P;
        config.Slot1.kI = DIFFERENTIAL_I;
        config.Slot1.kD = DIFFERENTIAL_D;
        config.Slot1.kA = DIFFERENTIAL_KA;
        config.Slot1.kG = DIFFERENTIAL_KG;
        config.Slot1.kS = DIFFERENTIAL_KS;
        config.Slot1.kV = DIFFERENTIAL_KV;

        config.MotionMagic.MotionMagicCruiseVelocity = MOTION_MAGIC_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = MOTION_MAGIC_ACCELERATION;
        config.MotionMagic.MotionMagicJerk = MOTION_MAGIC_JERK;

        RIGHT_MOTOR.getConfigurator().apply(config);

        RIGHT_MOTOR_POSITION_SIGNAL.setUpdateFrequency(100);
        RIGHT_MOTOR_VELOCITY_SIGNAL.setUpdateFrequency(100);
        RIGHT_MOTOR_SETPOINT_SIGNAL.setUpdateFrequency(100);
        RIGHT_MOTOR_VOLTAGE_SIGNAL.setUpdateFrequency(100);
        RIGHT_MOTOR_CURRENT_SIGNAL.setUpdateFrequency(100);

        RIGHT_MOTOR.optimizeBusUtilization();
    }

    private static void configureLeftMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = LEFT_MOTOR_INVERTED_VALUE;
        config.MotorOutput.NeutralMode = NEUTRAL_MODE_VALUE;
        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.Slot0.kP = AVERAGE_P;
        config.Slot0.kI = AVERAGE_I;
        config.Slot0.kD = AVERAGE_D;
        config.Slot0.kA = AVERAGE_KA;
        config.Slot0.kG = AVERAGE_KG;
        config.Slot0.kS = AVERAGE_KS;
        config.Slot0.kV = AVERAGE_KV;

        config.Slot1.kP = DIFFERENTIAL_P;
        config.Slot1.kI = DIFFERENTIAL_I;
        config.Slot1.kD = DIFFERENTIAL_D;
        config.Slot1.kA = DIFFERENTIAL_KA;
        config.Slot1.kG = DIFFERENTIAL_KG;
        config.Slot1.kS = DIFFERENTIAL_KS;
        config.Slot1.kV = DIFFERENTIAL_KV;

        config.MotionMagic.MotionMagicCruiseVelocity = MOTION_MAGIC_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = MOTION_MAGIC_ACCELERATION;
        config.MotionMagic.MotionMagicJerk = MOTION_MAGIC_JERK;

        LEFT_MOTOR.getConfigurator().apply(config);

        LEFT_MOTOR_POSITION_SIGNAL.setUpdateFrequency(100);
        LEFT_MOTOR_VELOCITY_SIGNAL.setUpdateFrequency(100);
        LEFT_MOTOR_SETPOINT_SIGNAL.setUpdateFrequency(100);
        LEFT_MOTOR_VOLTAGE_SIGNAL.setUpdateFrequency(100);
        LEFT_MOTOR_CURRENT_SIGNAL.setUpdateFrequency(100);

        RIGHT_MOTOR.optimizeBusUtilization();
    }
}
