package frc.trigon.robot.subsystems.transporter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.hardware.misc.objectdetectioncamera.SimulationObjectDetectionCameraIO;
import frc.trigon.robot.hardware.misc.simplesensor.SimpleSensor;
import frc.trigon.robot.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.robot.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.robot.hardware.simulation.FlywheelSimulation;
import frc.trigon.robot.utilities.mechanisms.SpeedMechanism2d;

import java.util.function.DoubleSupplier;

public class TransporterConstants {
    private static final int
            MOTOR_ID = 16,
            BEAM_BREAK_DIO_PORT = 1;
    private static final String
            MOTOR_NAME = "TransporterMotor",
            BEAM_BREAK_NAME = "BeamBreak";
    static final TalonFXMotor MOTOR = new TalonFXMotor(MOTOR_ID, MOTOR_NAME, RobotConstants.CANIVORE_NAME);
    static final SimpleSensor BEAM_BREAK = SimpleSensor.createDigitalSensor(BEAM_BREAK_DIO_PORT, BEAM_BREAK_NAME);

    private static final NeutralModeValue MOTOR_NEUTRAL_MODE_VALUE = NeutralModeValue.Coast;
    private static final InvertedValue MOTOR_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;
    static final boolean FOC_ENABLED = true;
    static final double GEAR_RATIO = 1.33333333333;
    private static final int MOTOR_AMOUNT = 1;
    private static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(MOTOR_AMOUNT);
    private static final double MOMENT_OF_INERTIA = 0.003;
    private static final FlywheelSimulation SIMULATION = new FlywheelSimulation(GEARBOX, GEAR_RATIO, MOMENT_OF_INERTIA);
    private static final DoubleSupplier BEAM_BREAK_SIMULATION_VALUE_SUPPLIER = () -> SimulationObjectDetectionCameraIO.HAS_OBJECTS ? 1 : 0;


    private static final double MAXIMUM_DISPLAYABLE_VELOCITY = 12;
    static final SpeedMechanism2d MECHANISM = new SpeedMechanism2d("Transporter", MAXIMUM_DISPLAYABLE_VELOCITY);

    static final double NOTE_COLLECTION_THRESHOLD_SECONDS = 0.09;
    static final double
            NOTE_COLLECTION_RUMBLE_DURATION_SECONDS = 0.6,
            NOTE_COLLECTION_RUMBLE_POWER = 1;

    static {
        configureMotor();
        configureBeamBreak();
    }

    private static void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.Inverted = MOTOR_INVERTED_VALUE;
        config.MotorOutput.NeutralMode = MOTOR_NEUTRAL_MODE_VALUE;
        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

        MOTOR.applyConfiguration(config);
        MOTOR.setPhysicsSimulation(SIMULATION);

        MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);

    }

    private static void configureBeamBreak() {
        BEAM_BREAK.setSimulationSupplier(BEAM_BREAK_SIMULATION_VALUE_SUPPLIER);
    }

    public enum TransporterState {
        STOPPED(0),
        COLLECTING(4),
        AUTONOMOUS_FEEDING(12),
        FEEDING(12),
        SCORE_AMP(-12),
        ALIGNING_FOR_AMP(-3),
        ALIGNING_FOR_AMP_BACKWARDS(3),
        ALIGNING_FOR_TRAP(3),
        SCORE_TRAP(-5),
        EJECTING(-3);

        final double voltage;

        TransporterState(double voltage) {
            this.voltage = voltage;
        }
    }
}
