package frc.trigon.robot.subsystems.climber.simulationclimber;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.trigon.robot.simulation.ElevatorSimulation;
import frc.trigon.robot.simulation.MotorSimulationConfiguration;
import frc.trigon.robot.subsystems.climber.ClimberConstants;

public class SimulationClimberConstants {
    private static final int MOTOR_AMOUNT = 1;
    private static final double
            RIGHT_MOTOR_GEAR_RATIO = 1,
            LEFT_MOTOR_GEAR_RATIO = 1;
    private static final double
            RIGHT_MOTOR_MOMENT_OF_INERTIA = 0.0001,
            LEFT_MOTOR_MOMENT_OF_INERTIA = 0.0001;
    private static final double
            RETRACTED_ARM_LENGTH_METERS = 0.1,
            MAXIMUM_HEIGHT_METERS = 0.5;
    private static final boolean SIMULATE_GRAVITY = true;
    private static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(MOTOR_AMOUNT);
    private static final double VOLTAGE_COMPENSATION_SATURATION = 12;
    private static final double CONVERSIONS_FACTOR = 1;
    private static final double
            P = 0,
            I = 0,
            D = 0,
            KS = 0,
            KG = 0,
            KV = 0,
            KA = 0;
    private static final double
            MAX_VELOCITY = 100,
            MAX_ACCELERATION = 60;
    static final ElevatorSimulation
            RIGHT_MOTOR = new ElevatorSimulation(
            GEARBOX,
            RIGHT_MOTOR_GEAR_RATIO,
            RIGHT_MOTOR_MOMENT_OF_INERTIA,
            ClimberConstants.DRUM_RADIUS,
            RETRACTED_ARM_LENGTH_METERS,
            MAXIMUM_HEIGHT_METERS,
            SIMULATE_GRAVITY
    ),
            LEFT_MOTOR = new ElevatorSimulation(
                    GEARBOX,
                    LEFT_MOTOR_GEAR_RATIO,
                    LEFT_MOTOR_MOMENT_OF_INERTIA,
                    ClimberConstants.DRUM_RADIUS,
                    RETRACTED_ARM_LENGTH_METERS,
                    MAXIMUM_HEIGHT_METERS,
                    SIMULATE_GRAVITY
            );

    static {
        configureMotor(RIGHT_MOTOR);
        configureMotor(LEFT_MOTOR);
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
