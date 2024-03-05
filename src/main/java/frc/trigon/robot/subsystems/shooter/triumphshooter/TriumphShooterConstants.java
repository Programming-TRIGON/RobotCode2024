package frc.trigon.robot.subsystems.shooter.triumphshooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.shooter.ShooterConstants;

public class TriumphShooterConstants {
    static final double CURRENT_LIMIT_TIME_THRESHOLD = 0.02;
    static final double HOLDING_CURRENT = 30;
    static final double CURRENT_LIMIT_THRESHOLD = 30;
    static final boolean FOC_ENABLED = true;
    private static final int
            MASTER_SHOOTING_MOTOR_ID = 9,
            FOLLOWER_SHOOTING_MOTOR_ID = 10;
    private static final NeutralModeValue NEUTRAL_MODE_VALUE = NeutralModeValue.Coast;
    private static final InvertedValue
            MASTER_MOTOR_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive,
            FOLLOWER_MOTOR_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;
    private static final double
            P = 0.02,
            I = 0,
            D = 0,
            KV = 0.13,
            KA = 0.027497,
            KS = 0.37303;
    private static final boolean FOLLOWER_OPPOSES_MASTER = false;
    static final TalonFX
            MASTER_MOTOR = new TalonFX(MASTER_SHOOTING_MOTOR_ID, RobotConstants.CANIVORE_NAME),
            FOLLOWER_MOTOR = new TalonFX(FOLLOWER_SHOOTING_MOTOR_ID, RobotConstants.CANIVORE_NAME);

    static final StatusSignal<Double>
            MASTER_MOTOR_VELOCITY_SIGNAL = MASTER_MOTOR.getVelocity().clone(),
            MASTER_MOTOR_POSITION_SIGNAL = MASTER_MOTOR.getPosition().clone(),
            MASTER_MOTOR_VOLTAGE_SIGNAL = MASTER_MOTOR.getMotorVoltage().clone(),
            MASTER_MOTOR_CURRENT_SIGNAL = MASTER_MOTOR.getStatorCurrent().clone(),
            MASTER_MOTOR_ACCELERATION = MASTER_MOTOR.getAcceleration().clone();

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

        config.Slot0.kP = P;
        config.Slot0.kI = I;
        config.Slot0.kD = D;
        config.Slot0.kV = KV;
        config.Slot0.kA = KA;
        config.Slot0.kS = KS;

        MASTER_MOTOR.getConfigurator().apply(config);

        MASTER_MOTOR_VELOCITY_SIGNAL.setUpdateFrequency(100);
        MASTER_MOTOR_POSITION_SIGNAL.setUpdateFrequency(100);
        MASTER_MOTOR_VOLTAGE_SIGNAL.setUpdateFrequency(100);
        MASTER_MOTOR_CURRENT_SIGNAL.setUpdateFrequency(100);
        MASTER_MOTOR_ACCELERATION.setUpdateFrequency(100);
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
