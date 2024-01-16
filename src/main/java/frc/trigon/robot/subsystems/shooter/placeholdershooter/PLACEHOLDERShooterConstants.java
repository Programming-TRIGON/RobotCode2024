package frc.trigon.robot.subsystems.shooter.placeholdershooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.trigon.robot.subsystems.shooter.ShooterConstants;

public class PLACEHOLDERShooterConstants {
    private static final int
            TOP_SHOOTING_MOTOR_ID = 1,
            BOTTOM_SHOOTING_MOTOR_ID = 0;
    private static final NeutralModeValue NEUTRAL_MODE_VALUE = NeutralModeValue.Coast;
    private static final InvertedValue
            TOP_MOTOR_INVERTED_VALUE = InvertedValue.Clockwise_Positive,
            BOTTOM_MOTOR_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;
    private static final double
            TOP_MOTOR_P = 6,
            TOP_MOTOR_I = 0,
            TOP_MOTOR_D = 0,
            BOTTOM_MOTOR_P = 6,
            BOTTOM_MOTOR_I = 0,
            BOTTOM_MOTOR_D = 0;
    static final TalonFX
            TOP_SHOOTING_MOTOR = new TalonFX(TOP_SHOOTING_MOTOR_ID),
            BOTTOM_SHOOTING_MOTOR = new TalonFX(BOTTOM_SHOOTING_MOTOR_ID);

    static final StatusSignal<Double>
            TOP_MOTOR_VELOCITY_SIGNAL = TOP_SHOOTING_MOTOR.getVelocity(),
            BOTTOM_MOTOR_VELOCITY_SIGNAL = BOTTOM_SHOOTING_MOTOR.getVelocity();

    static {
        configureTopShootingMotor();
        configureBottomShootingMotor();
    }

    private static void configureTopShootingMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnConfig = false;
        config.Audio.BeepOnBoot = false;

        config.Slot0.kP = TOP_MOTOR_P;
        config.Slot0.kI = TOP_MOTOR_I;
        config.Slot0.kD = TOP_MOTOR_D;

        config.MotorOutput.Inverted = TOP_MOTOR_INVERTED_VALUE;
        config.MotorOutput.NeutralMode = NEUTRAL_MODE_VALUE;
        config.Feedback.SensorToMechanismRatio = ShooterConstants.SHOOTER_GEAR_RATIO;

        TOP_SHOOTING_MOTOR.getConfigurator().apply(config);

        TOP_MOTOR_VELOCITY_SIGNAL.setUpdateFrequency(100);
        TOP_SHOOTING_MOTOR.optimizeBusUtilization();
    }

    private static void configureBottomShootingMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnConfig = false;
        config.Audio.BeepOnBoot = false;

        config.Slot0.kP = BOTTOM_MOTOR_P;
        config.Slot0.kI = BOTTOM_MOTOR_I;
        config.Slot0.kD = BOTTOM_MOTOR_D;

        config.MotorOutput.Inverted = BOTTOM_MOTOR_INVERTED_VALUE;
        config.MotorOutput.NeutralMode = NEUTRAL_MODE_VALUE;
        config.Feedback.SensorToMechanismRatio = ShooterConstants.SHOOTER_GEAR_RATIO;

        BOTTOM_SHOOTING_MOTOR.getConfigurator().apply(config);

        BOTTOM_MOTOR_VELOCITY_SIGNAL.setUpdateFrequency(100);
        BOTTOM_SHOOTING_MOTOR.optimizeBusUtilization();
    }
}
