package frc.trigon.robot.subsystems.shooter.placeholdershooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.trigon.robot.constants.RobotConstants;

public class PLACEHOLDERShooterConstants {
    private static final int
            SHOOTER_MOTOR_ID = -1,
            FEEDING_MOTOR_ID = -1;
    private static final NeutralModeValue
            SHOOTER_MOTOR_NEUTRAL_MODE_VALUE = NeutralModeValue.Coast,
            FEEDING_MOTOR_NEUTRAL_MODE_VALUE = NeutralModeValue.Coast;
    private static final InvertedValue
            SHOOTER_MOTOR_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive,
            FEEDING_MOTOR_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;
    private static final double SHOOTING_MOTOR_GEAR_RATIO = 1;
    private static final double
            SHOOTING_MOTOR_P = 0,
            SHOOTING_MOTOR_I = 0,
            SHOOTING_MOTOR_D = 0;
    static final TalonFX
            SHOOTING_MOTOR = new TalonFX(SHOOTER_MOTOR_ID, RobotConstants.CANIVORE_NAME),
            FEEDING_MOTOR = new TalonFX(FEEDING_MOTOR_ID, RobotConstants.CANIVORE_NAME);

    static final StatusSignal<Double>
            SHOOTING_MOTOR_VELOCITY_SIGNAL = SHOOTING_MOTOR.getVelocity(),
            FEEDING_MOTOR_VOLTAGE_SIGNAL = FEEDING_MOTOR.getMotorVoltage();

    static {
        configureShootingMotor();
        configureFeedingMotor();
    }

    private static void configureShootingMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnConfig = false;
        config.Audio.BeepOnBoot = false;

        config.Slot0.kP = SHOOTING_MOTOR_P;
        config.Slot0.kI = SHOOTING_MOTOR_I;
        config.Slot0.kD = SHOOTING_MOTOR_D;

        config.MotorOutput.Inverted = SHOOTER_MOTOR_INVERTED_VALUE;
        config.MotorOutput.NeutralMode = SHOOTER_MOTOR_NEUTRAL_MODE_VALUE;
        config.Feedback.SensorToMechanismRatio = SHOOTING_MOTOR_GEAR_RATIO;

        SHOOTING_MOTOR.getConfigurator().apply(config);

        SHOOTING_MOTOR.getVelocity().setUpdateFrequency(100);
        SHOOTING_MOTOR.optimizeBusUtilization();
    }

    private static void configureFeedingMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnConfig = false;
        config.Audio.BeepOnBoot = false;

        config.MotorOutput.Inverted = FEEDING_MOTOR_INVERTED_VALUE;
        config.MotorOutput.NeutralMode = FEEDING_MOTOR_NEUTRAL_MODE_VALUE;

        FEEDING_MOTOR.getConfigurator().apply(config);

        FEEDING_MOTOR.getMotorVoltage().setUpdateFrequency(100);
        FEEDING_MOTOR.optimizeBusUtilization();
    }
}
