package frc.trigon.robot.subsystems.collector.simulationcollector;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.trigon.robot.simulation.MotorSimulationConfiguration;
import frc.trigon.robot.simulation.SimpleMotorSimulation;
import frc.trigon.robot.simulation.SingleJointedArmSimulation;

public class SimulationCollectorConstants {
    private static final int
            ANGLE_MOTOR_AMOUNT = 1,
            COLLECTION_MOTOR_AMOUNT = 1;
    private static final double
            ANGLE_MOTOR_GEAR_RATIO = 50,
            COLLECTION_MOTOR_GEAR_RATIO = 20;
    private static final double ANGLE_ARM_LENGTH_METERS = 0.7;
    private static final double ANGLE_ARM_MASS = 6;
    private static final Rotation2d
            MINIMUM_ANGLE = Rotation2d.fromDegrees(-90),
            MAXIMUM_ANGLE = Rotation2d.fromDegrees(90);
    private static final boolean SIMULATE_GRAVITY = true;
    private static final double COLLECTION_MOTOR_MOMENT_OF_INERTIA = 1;
    private static final DCMotor
            ANGLE_GEARBOX = DCMotor.getKrakenX60Foc(ANGLE_MOTOR_AMOUNT),
            COLLECTION_GEARBOX = DCMotor.getKrakenX60Foc(COLLECTION_MOTOR_AMOUNT);
    private static final double VOLTAGE_COMPENSATION_SATURATION = 12;
    private static final double CONVERSION_FACTOR = 1;
    private static final double
            ANGLE_P = 500,
            ANGLE_I = 0,
            ANGLE_D = 0,
            ANGLE_KS = 0,
            ANGLE_KG = 0,
            ANGLE_KV = 0,
            ANGLE_KA = 0;
    private static final double
            ANGLE_MAX_VELOCITY = 8,
            ANGLE_MAX_ACCELERATION = 4;
    static final SingleJointedArmSimulation ANGLE_MOTOR = new SingleJointedArmSimulation(
            ANGLE_GEARBOX,
            ANGLE_MOTOR_GEAR_RATIO,
            ANGLE_ARM_LENGTH_METERS,
            ANGLE_ARM_MASS,
            MINIMUM_ANGLE,
            MAXIMUM_ANGLE,
            SIMULATE_GRAVITY
    );
    static final SimpleMotorSimulation COLLECTION_MOTOR = new SimpleMotorSimulation(
            COLLECTION_GEARBOX,
            COLLECTION_MOTOR_GEAR_RATIO,
            COLLECTION_MOTOR_MOMENT_OF_INERTIA
    );

    static {
        configureAngleMotor();
        configureCollectionMotor();
    }

    private static void configureAngleMotor() {
        final MotorSimulationConfiguration config = new MotorSimulationConfiguration();

        config.pidConfigs.kP = ANGLE_P;
        config.pidConfigs.kI = ANGLE_I;
        config.pidConfigs.kD = ANGLE_D;

        config.feedforwardConfigs.kS = ANGLE_KS;
        config.feedforwardConfigs.kG = ANGLE_KG;
        config.feedforwardConfigs.kV = ANGLE_KV;
        config.feedforwardConfigs.kA = ANGLE_KA;

        config.motionMagicConfigs.maximumVelocity = ANGLE_MAX_VELOCITY;
        config.motionMagicConfigs.maximumAcceleration = ANGLE_MAX_ACCELERATION;

        config.voltageCompensationSaturation = VOLTAGE_COMPENSATION_SATURATION;
        config.conversionsFactor = CONVERSION_FACTOR;

        ANGLE_MOTOR.applyConfiguration(config);
    }

    private static void configureCollectionMotor() {
        final MotorSimulationConfiguration config = new MotorSimulationConfiguration();

        config.voltageCompensationSaturation = VOLTAGE_COMPENSATION_SATURATION;
        config.conversionsFactor = CONVERSION_FACTOR;

        COLLECTION_MOTOR.applyConfiguration(config);
    }
}
