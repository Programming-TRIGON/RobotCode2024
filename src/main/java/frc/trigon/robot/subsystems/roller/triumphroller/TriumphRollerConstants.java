package frc.trigon.robot.subsystems.roller.triumphroller;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.roller.RollerConstants;

public class TriumphRollerConstants {
    private static final int
            MOTOR_ID = 0,
            INFRARED_SENSOR_CHANNEL = 0;
    private static final NeutralModeValue MOTOR_NEUTRAL_MODE_VALUE = NeutralModeValue.Coast;
    private static final InvertedValue MOTOR_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;
    private static final double
            P = 0,
            I = 0,
            D = 0;
    static final TalonFX MOTOR = new TalonFX(MOTOR_ID, RobotConstants.CANIVORE_NAME);
    static final DigitalInput INFRARED_SENSOR = new DigitalInput(INFRARED_SENSOR_CHANNEL);

    static final StatusSignal<Double>
            VOLTAGE_STATUS_SIGNAL = MOTOR.getMotorVoltage(),
            CURRENT_STATUS_SIGNAL = MOTOR.getTorqueCurrent(),
            VELOCITY_STATUS_SIGNAL = MOTOR.getVelocity();

    static {
        configureMotor();
    }

    private static void configureMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;
        config.MotorOutput.Inverted = MOTOR_INVERTED_VALUE;
        config.MotorOutput.NeutralMode = MOTOR_NEUTRAL_MODE_VALUE;
        config.Feedback.SensorToMechanismRatio = RollerConstants.GEAR_RATIO;

        config.Slot0.kP = P;
        config.Slot0.kI = I;
        config.Slot0.kD = D;

        MOTOR.getConfigurator().apply(config);

        VOLTAGE_STATUS_SIGNAL.setUpdateFrequency(100);
        CURRENT_STATUS_SIGNAL.setUpdateFrequency(100);
        VELOCITY_STATUS_SIGNAL.setUpdateFrequency(100);

        MOTOR.optimizeBusUtilization();
    }
}