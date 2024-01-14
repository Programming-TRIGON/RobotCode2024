package frc.trigon.robot.subsystems.collector.simulationcollector;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.trigon.robot.motorsimulation.MotorSimulationConfiguration;
import frc.trigon.robot.motorsimulation.SimpleMotorSimulation;

public class SimulationCollectorConstants {
    static final boolean
            FOC_ENABLED = true;
    private static final int
            ANGLE_MOTOR_AMOUNT = 1,
            COLLECTION_MOTOR_AMOUNT = 1;
    private static final double
            ANGLE_MOTOR_GEAR_RATION = 1,
            COLLECTION_MOTOR_GEAR_RATIO = 1;
    private static final double
            ANGLE_MOTOR_MOMENT_OF_INERTIA = 0.0001,
            COLLECTION_MOTOR_MOMENT_OF_INERTIA = 0.0001;
    private static final DCMotor
            ANGLE_GEARBOX = DCMotor.getKrakenX60Foc(ANGLE_MOTOR_AMOUNT),
            COLLECTION_GEARBOX = DCMotor.getKrakenX60Foc(COLLECTION_MOTOR_AMOUNT);
    private static final double
            ANGLE_VOLTAGE_COMPENSATION_SATURATION = 12,
            COLLECTION_VOLTAGE_COMPENSATION_SATURATION = 12;
    private static final double
            ANGLE_MOTOR_CONVERSION_FACTOR = 360,
            COLLECTION_MOTOR_CONVERSION_FACTOR = 1;
    static final SimpleMotorSimulation
            ANGLE_MOTOR = new SimpleMotorSimulation(
            ANGLE_GEARBOX,
            ANGLE_MOTOR_GEAR_RATION,
            ANGLE_MOTOR_MOMENT_OF_INERTIA
    ),
            COLLECTION_MOTOR = new SimpleMotorSimulation(
                    COLLECTION_GEARBOX,
                    COLLECTION_MOTOR_GEAR_RATIO,
                    COLLECTION_MOTOR_MOMENT_OF_INERTIA
            );
    private static final double
            ANGLE_P = 0,
            ANGLE_I = 0,
            ANGLE_D = 0,
            ANGLE_KS = 0,
            ANGLE_KV = 0,
            ANGLE_KA = 0;
    private static final double
            ANGLE_MAX_VELOCITY = 0,
            ANGLE_MAX_ACCELERATION = 0;

    static {
        configureAngleMotor();
        configureCollectionMotor();
    }

    private static void configureAngleMotor() {
        MotorSimulationConfiguration config = new MotorSimulationConfiguration();

        config.pidConfigs.kP = ANGLE_P;
        config.pidConfigs.kI = ANGLE_I;
        config.pidConfigs.kD = ANGLE_D;

        config.feedforwardConfigs.kS = ANGLE_KS;
        config.feedforwardConfigs.kV = ANGLE_KV;
        config.feedforwardConfigs.kA = ANGLE_KA;

        config.motionMagicConfigs.maximumVelocity = ANGLE_MAX_VELOCITY;
        config.motionMagicConfigs.maximumAcceleration = ANGLE_MAX_ACCELERATION;

        config.voltageCompensationSaturation = ANGLE_VOLTAGE_COMPENSATION_SATURATION;
        config.conversionFactor = ANGLE_MOTOR_CONVERSION_FACTOR;
        ANGLE_MOTOR.applyConfiguration(config);
    }

    private static void configureCollectionMotor() {
        MotorSimulationConfiguration config = new MotorSimulationConfiguration();
        config.voltageCompensationSaturation = COLLECTION_VOLTAGE_COMPENSATION_SATURATION;
        config.conversionFactor = COLLECTION_MOTOR_CONVERSION_FACTOR;
        COLLECTION_MOTOR.applyConfiguration(config);
    }
}
