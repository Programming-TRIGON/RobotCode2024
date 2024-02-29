package frc.trigon.robot.subsystems.intake.triumphintake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.intake.IntakeConstants;

public class TriumphIntakeConstants {
    static final boolean FOC_ENABLED = true;
    private static final int MOTOR_ID = 18;
    private static final InvertedValue INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;
    private static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;
    static final TalonFX MOTOR = new TalonFX(MOTOR_ID, RobotConstants.CANIVORE_NAME);

    static final StatusSignal<Double>
            COLLECTION_MOTOR_VELOCITY_SIGNAL = MOTOR.getVelocity().clone(),
            COLLECTION_MOTOR_CURRENT_SIGNAL = MOTOR.getStatorCurrent().clone(),
            COLLECTION_MOTOR_VOLTAGE_SIGNAL = MOTOR.getMotorVoltage().clone();

    static {
        configureCollectingMotor();
    }

    private static void configureCollectingMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = INVERTED_VALUE;
        config.MotorOutput.NeutralMode = NEUTRAL_MODE;
        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;
        config.Feedback.SensorToMechanismRatio = IntakeConstants.GEAR_RATIO;

        MOTOR.getConfigurator().apply(config);

        COLLECTION_MOTOR_VELOCITY_SIGNAL.setUpdateFrequency(100);
        COLLECTION_MOTOR_CURRENT_SIGNAL.setUpdateFrequency(100);
        COLLECTION_MOTOR_VOLTAGE_SIGNAL.setUpdateFrequency(100);
        MOTOR.optimizeBusUtilization();
    }
}
