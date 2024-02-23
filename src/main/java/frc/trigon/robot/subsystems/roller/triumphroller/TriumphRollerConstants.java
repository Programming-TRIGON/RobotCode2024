package frc.trigon.robot.subsystems.roller.triumphroller;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.roller.RollerConstants;

public class TriumphRollerConstants {
    private static final int MOTOR_ID = 16;
    private static final NeutralModeValue MOTOR_NEUTRAL_MODE_VALUE = NeutralModeValue.Coast;
    private static final InvertedValue MOTOR_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;
    static final TalonFX MOTOR = new TalonFX(MOTOR_ID, RobotConstants.CANIVORE_NAME);

    static final StatusSignal<Double>
            VOLTAGE_SIGNAL = MOTOR.getMotorVoltage().clone(),
            CURRENT_SIGNAL = MOTOR.getStatorCurrent().clone(),
            VELOCITY_SIGNAL = MOTOR.getVelocity().clone();

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

        MOTOR.getConfigurator().apply(config);

        VOLTAGE_SIGNAL.setUpdateFrequency(100);
        CURRENT_SIGNAL.setUpdateFrequency(100);
        VELOCITY_SIGNAL.setUpdateFrequency(100);

        MOTOR.optimizeBusUtilization();
    }
}