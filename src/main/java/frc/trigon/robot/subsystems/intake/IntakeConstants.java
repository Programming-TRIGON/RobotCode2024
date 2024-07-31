package frc.trigon.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.robot.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.robot.hardware.simulation.SimpleMotorSimulation;
import frc.trigon.robot.utilities.mechanisms.SpeedMechanism2d;

public class IntakeConstants {
    private static final int MOTOR_ID = 17;
    private static final String MOTOR_NAME = "IntakeMotor";
    static final TalonFXMotor MOTOR = new TalonFXMotor(
            MOTOR_ID,
            MOTOR_NAME,
            RobotConstants.CANIVORE_NAME
    );

    private static final NeutralModeValue NEUTRAL_MODE_VALUE = NeutralModeValue.Coast;
    private static final InvertedValue INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;
    private static final double GEAR_RATIO = 1.66666666667;
    static final boolean FOC_ENABLED = true;

    private static final int MOTOR_AMOUNT = 1;
    private static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(MOTOR_AMOUNT);
    private static final double MOMENT_OF_INERTIA = 1;
    private static final SimpleMotorSimulation SIMULATION = new SimpleMotorSimulation(
            GEARBOX,
            GEAR_RATIO,
            MOMENT_OF_INERTIA
    );

    private static final double MAX_DISPLAYABLE_VELOCITY = 12;
    static final SpeedMechanism2d MECHANISM = new SpeedMechanism2d(
            "IntakeMechanism", MAX_DISPLAYABLE_VELOCITY
    );

    static final double
            NOTE_COLLECTION_CURRENT = 34,
            NOTE_COLLECTION_TIME_THRESHOLD_SECONDS = 0.15;

    static {
        configureMotor();
    }

    private static void configureMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = NEUTRAL_MODE_VALUE;
        config.MotorOutput.Inverted = INVERTED_VALUE;
        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

        MOTOR.applyConfiguration(config);
        MOTOR.setPhysicsSimulation(SIMULATION);

        MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
        MOTOR.registerSignal(TalonFXSignal.SUPPLY_CURRENT, 100);
        MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
    }

    public enum IntakeState {
        STOPPED(0),
        COLLECTING(10),
        EJECTING(-4);

        final double voltage;

        IntakeState(double voltage) {
            this.voltage = voltage;
        }
    }
}
