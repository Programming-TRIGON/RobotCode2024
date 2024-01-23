package frc.trigon.robot.subsystems.shooter.placeholdershooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.trigon.robot.subsystems.shooter.ShooterConstants;

public class PLACEHOLDERShooterConstants {
    static final boolean FOC_ENABLED = true;
    private static final int
            MASTER_SHOOTING_MOTOR_ID = 1,
            FOLLOWER_SHOOTING_MOTOR_ID = 0;
    private static final NeutralModeValue NEUTRAL_MODE_VALUE = NeutralModeValue.Coast;
    private static final InvertedValue
            MASTER_MOTOR_INVERTED_VALUE = InvertedValue.Clockwise_Positive,
            FOLLOWER_MOTOR_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;
    private static final boolean FOLLOWER_OPPOSES_MASTER = false;
    static final TalonFX
            MASTER_MOTOR = new TalonFX(MASTER_SHOOTING_MOTOR_ID),
            FOLLOWER_MOTOR = new TalonFX(FOLLOWER_SHOOTING_MOTOR_ID);

    static final StatusSignal<Double>
            MASTER_MOTOR_VELOCITY_SIGNAL = MASTER_MOTOR.getVelocity(),
            MASTER_MOTOR_POSITION_SIGNAL = MASTER_MOTOR.getPosition(),
            MASTER_MOTOR_VOLTAGE_SIGNAL = MASTER_MOTOR.getMotorVoltage();

    static {
        configureMasterMotor();
        configureFollowerMotor();
    }

    private static void configureMasterMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnConfig = false;
        config.Audio.BeepOnBoot = false;

        config.MotorOutput.Inverted = MASTER_MOTOR_INVERTED_VALUE;
        config.MotorOutput.NeutralMode = NEUTRAL_MODE_VALUE;
        config.Feedback.SensorToMechanismRatio = ShooterConstants.GEAR_RATIO;

        MASTER_MOTOR.getConfigurator().apply(config);

        MASTER_MOTOR_VELOCITY_SIGNAL.setUpdateFrequency(100);
        MASTER_MOTOR_POSITION_SIGNAL.setUpdateFrequency(100);
        MASTER_MOTOR_VOLTAGE_SIGNAL.setUpdateFrequency(100);
        MASTER_MOTOR.optimizeBusUtilization();
    }

    private static void configureFollowerMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnConfig = false;
        config.Audio.BeepOnBoot = false;

        config.MotorOutput.Inverted = FOLLOWER_MOTOR_INVERTED_VALUE;
        config.MotorOutput.NeutralMode = NEUTRAL_MODE_VALUE;
        config.Feedback.SensorToMechanismRatio = ShooterConstants.GEAR_RATIO;

        FOLLOWER_MOTOR.getConfigurator().apply(config);
        FOLLOWER_MOTOR.optimizeBusUtilization();
        FOLLOWER_MOTOR.setControl(new Follower(MASTER_SHOOTING_MOTOR_ID, FOLLOWER_OPPOSES_MASTER));
    }
}
