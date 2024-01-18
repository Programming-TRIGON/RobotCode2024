package frc.trigon.robot.subsystems.shooter.placeholdershooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.trigon.robot.subsystems.shooter.ShooterConstants;

public class PLACEHOLDERShooterConstants {
    static final boolean FOC_ENABLED = true;
    private static final int
            TOP_SHOOTING_MOTOR_ID = 1,
            BOTTOM_SHOOTING_MOTOR_ID = 0;
    private static final NeutralModeValue NEUTRAL_MODE_VALUE = NeutralModeValue.Coast;
    private static final InvertedValue
            TOP_MOTOR_INVERTED_VALUE = InvertedValue.Clockwise_Positive,
            BOTTOM_MOTOR_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;
    static final TalonFX
            TOP_MOTOR = new TalonFX(TOP_SHOOTING_MOTOR_ID),
            BOTTOM_MOTOR = new TalonFX(BOTTOM_SHOOTING_MOTOR_ID);

    static final StatusSignal<Double>
            TOP_MOTOR_VELOCITY_SIGNAL = TOP_MOTOR.getVelocity(),
            TOP_MOTOR_POSITION_SIGNAL = TOP_MOTOR.getPosition(),
            TOP_MOTOR_VOLTAGE_SIGNAL = TOP_MOTOR.getMotorVoltage(),
            BOTTOM_MOTOR_VELOCITY_SIGNAL = BOTTOM_MOTOR.getVelocity(),
            BOTTOM_MOTOR_POSITION_SIGNAL = BOTTOM_MOTOR.getPosition(),
            BOTTOM_MOTOR_VOLTAGE_SIGNAL = BOTTOM_MOTOR.getMotorVoltage();

    static {
        configureShootingMotor(TOP_MOTOR, TOP_MOTOR_INVERTED_VALUE);
        configureShootingMotor(BOTTOM_MOTOR, BOTTOM_MOTOR_INVERTED_VALUE);
        configureStatusSignals();
    }

    private static void configureShootingMotor(TalonFX motor, InvertedValue invertedValue) {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnConfig = false;
        config.Audio.BeepOnBoot = false;

        config.MotorOutput.Inverted = invertedValue;
        config.MotorOutput.NeutralMode = NEUTRAL_MODE_VALUE;
        config.Feedback.SensorToMechanismRatio = ShooterConstants.GEAR_RATIO;

        motor.getConfigurator().apply(config);
        motor.optimizeBusUtilization();
    }

    private static void configureStatusSignals() {
        TOP_MOTOR_VELOCITY_SIGNAL.setUpdateFrequency(100);
        TOP_MOTOR_POSITION_SIGNAL.setUpdateFrequency(100);
        TOP_MOTOR_VOLTAGE_SIGNAL.setUpdateFrequency(100);
        BOTTOM_MOTOR_VELOCITY_SIGNAL.setUpdateFrequency(100);
        BOTTOM_MOTOR_POSITION_SIGNAL.setUpdateFrequency(100);
        BOTTOM_MOTOR_VOLTAGE_SIGNAL.setUpdateFrequency(100);

        TOP_MOTOR.optimizeBusUtilization();
        BOTTOM_MOTOR.optimizeBusUtilization();
    }
}
