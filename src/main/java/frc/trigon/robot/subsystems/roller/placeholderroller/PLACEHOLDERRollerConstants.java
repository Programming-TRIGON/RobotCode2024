package frc.trigon.robot.subsystems.roller.placeholderroller;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;

public class PLACEHOLDERRollerConstants {
    static final boolean FOC_ENABLED = true;
    private static final int
            MOTOR_ID = 0,
            INFRARED_SENSOR_CHANNEL = 0;
    private static final NeutralModeValue MOTOR_NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;
    private static final InvertedValue MOTOR_INVERTED_VALUE = InvertedValue.Clockwise_Positive;
    static final TalonFX MOTOR = new TalonFX(MOTOR_ID);
    static final DigitalInput INFRARED_SENSOR = new DigitalInput(INFRARED_SENSOR_CHANNEL);

    private static final double
            P = 0,
            I = 0,
            D = 0,
            KS = 0,
            KV = 0,
            KG = 0,
            KA = 0;
    private static double
            MOTION_MAGIC_JERK = 0,
            MOTION_MAGIC_ACCELERATION = 0,
            MOTION_MAGIC_CRUISE_VELOCITY = 0;

    static {
        configureMotor();
    }

    private static void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Audio.BeepOnBoot = false;
        config.MotorOutput.Inverted = MOTOR_INVERTED_VALUE;
        config.MotorOutput.NeutralMode = MOTOR_NEUTRAL_MODE_VALUE;

        config.Slot0.kP = P;
        config.Slot0.kI = I;
        config.Slot0.kD = D;

        config.Slot0.kS = KS;
        config.Slot0.kV = KV;
        config.Slot0.kG = KG;
        config.Slot0.kA = KA;

        config.MotionMagic.MotionMagicJerk = MOTION_MAGIC_JERK;
        config.MotionMagic.MotionMagicAcceleration = MOTION_MAGIC_ACCELERATION;
        config.MotionMagic.MotionMagicCruiseVelocity = MOTION_MAGIC_CRUISE_VELOCITY;

        MOTOR.getConfigurator().apply(config);
        MOTOR.optimizeBusUtilization();
    }
}