package frc.trigon.robot.subsystems.climber.simulationclimber;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.trigon.robot.simulation.ElevatorSimulation;
import frc.trigon.robot.simulation.MotorSimulationConfiguration;
import frc.trigon.robot.subsystems.climber.ClimberConstants;

public class SimulationClimberConstants {
    private static final int MOTOR_AMOUNT = 1;
    private static final double MASS_KILOGRAMS = 6;
    private static final double MAXIMUM_HEIGHT_METERS = 0.7;
    private static final boolean SIMULATE_GRAVITY = true;
    private static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(MOTOR_AMOUNT);
    private static final double VOLTAGE_COMPENSATION_SATURATION = 12;
    private static final double CONVERSIONS_FACTOR = 1;
    private static final double
            P = 100,
            I = 5,
            D = 0,
            KS = 0,
            KG = 0,
            KV = 0,
            KA = 0;
    private static final double
            MAX_VELOCITY = 5,
            MAX_ACCELERATION = 3;
    static final ElevatorSimulation
            MASTER_MOTOR = new ElevatorSimulation(
            GEARBOX,
            ClimberConstants.GEAR_RATIO,
            MASS_KILOGRAMS,
            ClimberConstants.DRUM_RADIUS_METERS,
            ClimberConstants.RETRACTED_CLIMBER_LENGTH_METERS,
            MAXIMUM_HEIGHT_METERS,
            SIMULATE_GRAVITY
    ),
            FOLLOWER_MOTOR = new ElevatorSimulation(
                    GEARBOX,
                    ClimberConstants.GEAR_RATIO,
                    MASS_KILOGRAMS,
                    ClimberConstants.DRUM_RADIUS_METERS,
                    ClimberConstants.RETRACTED_CLIMBER_LENGTH_METERS,
                    MAXIMUM_HEIGHT_METERS,
                    SIMULATE_GRAVITY
            );

    static {
        configureMotor(MASTER_MOTOR);
        configureMotor(FOLLOWER_MOTOR);
    }

    private static void configureMotor(ElevatorSimulation motor) {
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

        motor.applyConfiguration(config);
    }
}
