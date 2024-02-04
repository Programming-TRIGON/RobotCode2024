package frc.trigon.robot.subsystems.climber.simulationclimber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.trigon.robot.simulation.ElevatorSimulation;
import frc.trigon.robot.subsystems.climber.ClimberConstants;

public class SimulationClimberConstants {
    private static final int MOTOR_AMOUNT = 2;
    private static final double MASS_KILOGRAMS = 2;
    private static final double MAXIMUM_HEIGHT_METERS = 0.7188;
    private static final boolean SIMULATE_GRAVITY = true;
    private static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(MOTOR_AMOUNT);
    static final int
            NON_CLIMBING_SLOT = 0,
            CLIMBING_SLOT = 1;
    private static final double
            NON_CLIMBING_P = 30,
            NON_CLIMBING_I = 0,
            NON_CLIMBING_D = 0,
            NON_CLIMBING_KS = 0.030713,
            NON_CLIMBING_KG = 0.009791,
            NON_CLIMBING_KV = 2.385,
            NON_CLIMBING_KA = 0.049261;
    private static final double
            CLIMBING_P = 30,
            CLIMBING_I = 0,
            CLIMBING_D = 0,
            CLIMBING_KS = 0.030713,
            CLIMBING_KG = 0.009791,
            CLIMBING_KV = 2.385,
            CLIMBING_KA = 0.049261;
    static final double
            MAX_NON_CLIMBING_VELOCITY = 18,
            MAX_NON_CLIMBING_ACCELERATION = 18,
            MAX_CLIMBING_VELOCITY = 18,
            MAX_CLIMBING_ACCELERATION = 18;
    static final ElevatorSimulation MOTOR = new ElevatorSimulation(
            GEARBOX,
            ClimberConstants.GEAR_RATIO,
            MASS_KILOGRAMS,
            ClimberConstants.DRUM_RADIUS_METERS,
            ClimberConstants.RETRACTED_CLIMBER_LENGTH_METERS,
            MAXIMUM_HEIGHT_METERS,
            SIMULATE_GRAVITY
    );

    static {
        configureMotor();
    }

    private static void configureMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = NON_CLIMBING_P;
        config.Slot0.kI = NON_CLIMBING_I;
        config.Slot0.kD = NON_CLIMBING_D;
        config.Slot0.kS = NON_CLIMBING_KS;
        config.Slot0.kG = NON_CLIMBING_KG;
        config.Slot0.kV = NON_CLIMBING_KV;
        config.Slot0.kA = NON_CLIMBING_KA;

        config.Slot1.kP = CLIMBING_P;
        config.Slot1.kI = CLIMBING_I;
        config.Slot1.kD = CLIMBING_D;
        config.Slot1.kS = CLIMBING_KS;
        config.Slot1.kG = CLIMBING_KG;
        config.Slot1.kV = CLIMBING_KV;
        config.Slot1.kA = CLIMBING_KA;

        MOTOR.applyConfiguration(config);
    }
}
