package frc.trigon.robot.subsystems.climber.simulationclimber;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.trigon.robot.simulation.ElevatorSimulation;
import frc.trigon.robot.simulation.MotorSimulationConfiguration;
import frc.trigon.robot.subsystems.climber.ClimberConstants;

public class SimulationClimberConstants {
    private static final int MOTOR_AMOUNT = 2;
    private static final double MASS_KILOGRAMS = 2;
    private static final double MAXIMUM_HEIGHT_METERS = 0.7188;
    private static final boolean SIMULATE_GRAVITY = true;
    private static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(MOTOR_AMOUNT);
    private static final double VOLTAGE_COMPENSATION_SATURATION = 12;
    private static final double CONVERSIONS_FACTOR = 1;
    private static final double
            P = 0,
            I = 5,
            D = 0,
            KS = 0.04231,
            KG = 0.0090024,
            KV = 18.793,
            KA = 0.39411;
    private static final double
            MAX_VELOCITY = 5,
            MAX_ACCELERATION = 3;
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
        final MotorSimulationConfiguration config = new MotorSimulationConfiguration();

        config.pidConfigs.kP = P;
        config.pidConfigs.kI = I;
        config.pidConfigs.kD = D;

        config.feedforwardConfigs.kS = KS;
        config.feedforwardConfigs.kG = KG;
        config.feedforwardConfigs.kV = KV;
        config.feedforwardConfigs.kA = KA;

        config.motionMagicConfigs.maximumVelocity = MAX_VELOCITY;
        config.motionMagicConfigs.maximumAcceleration = MAX_ACCELERATION;

        config.voltageCompensationSaturation = VOLTAGE_COMPENSATION_SATURATION;
        config.conversionsFactor = CONVERSIONS_FACTOR;

        MOTOR.applyConfiguration(config);
    }
}
