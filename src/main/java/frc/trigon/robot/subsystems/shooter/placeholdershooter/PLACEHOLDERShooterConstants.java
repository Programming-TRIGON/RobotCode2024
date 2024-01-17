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
    private static final double
            TOP_MOTOR_KS = 0.24958,
            TOP_MOTOR_KV = 0.12401,
            TOP_MOTOR_KA = 0.0092721,
            BOTTOM_MOTOR_KS = 0.33493,
            BOTTOM_MOTOR_KV = 0.12104,
            BOTTOM_MOTOR_KA = 0.037;
    static final TalonFX
            TOP_SHOOTING_MOTOR = new TalonFX(TOP_SHOOTING_MOTOR_ID),
            BOTTOM_SHOOTING_MOTOR = new TalonFX(BOTTOM_SHOOTING_MOTOR_ID);

    static final StatusSignal<Double>
            TOP_MOTOR_VELOCITY_SIGNAL = TOP_SHOOTING_MOTOR.getVelocity(),
            TOP_MOTOR_POSITION_SIGNAL = TOP_SHOOTING_MOTOR.getPosition(),
            TOP_MOTOR_VOLTAGE_SIGNAL = TOP_SHOOTING_MOTOR.getMotorVoltage(),
            BOTTOM_MOTOR_VELOCITY_SIGNAL = BOTTOM_SHOOTING_MOTOR.getVelocity(),
            BOTTOM_MOTOR_POSITION_SIGNAL = BOTTOM_SHOOTING_MOTOR.getPosition(),
            BOTTOM_MOTOR_VOLTAGE_SIGNAL = BOTTOM_SHOOTING_MOTOR.getMotorVoltage();

    static {
        configureTopShootingMotor();
        configureBottomShootingMotor();
    }

    private static void configureTopShootingMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnConfig = false;
        config.Audio.BeepOnBoot = false;

        config.Slot0.kS = TOP_MOTOR_KS;
        config.Slot0.kV = TOP_MOTOR_KV;
        config.Slot0.kA = TOP_MOTOR_KA;

        config.MotorOutput.Inverted = TOP_MOTOR_INVERTED_VALUE;
        config.MotorOutput.NeutralMode = NEUTRAL_MODE_VALUE;
        config.Feedback.SensorToMechanismRatio = ShooterConstants.SHOOTER_GEAR_RATIO;

        TOP_SHOOTING_MOTOR.getConfigurator().apply(config);

        TOP_MOTOR_VELOCITY_SIGNAL.setUpdateFrequency(100);
        TOP_MOTOR_POSITION_SIGNAL.setUpdateFrequency(100);
        TOP_MOTOR_VOLTAGE_SIGNAL.setUpdateFrequency(100);
        TOP_SHOOTING_MOTOR.optimizeBusUtilization();
    }

    private static void configureBottomShootingMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnConfig = false;
        config.Audio.BeepOnBoot = false;

        config.Slot0.kS = BOTTOM_MOTOR_KS;
        config.Slot0.kV = BOTTOM_MOTOR_KV;
        config.Slot0.kA = BOTTOM_MOTOR_KA;

        config.MotorOutput.Inverted = BOTTOM_MOTOR_INVERTED_VALUE;
        config.MotorOutput.NeutralMode = NEUTRAL_MODE_VALUE;
        config.Feedback.SensorToMechanismRatio = ShooterConstants.SHOOTER_GEAR_RATIO;

        BOTTOM_SHOOTING_MOTOR.getConfigurator().apply(config);

        BOTTOM_MOTOR_VELOCITY_SIGNAL.setUpdateFrequency(100);
        BOTTOM_MOTOR_POSITION_SIGNAL.setUpdateFrequency(100);
        BOTTOM_MOTOR_VOLTAGE_SIGNAL.setUpdateFrequency(100);
        BOTTOM_SHOOTING_MOTOR.optimizeBusUtilization();
    }
}
