package frc.trigon.robot.subsystems.pitcher.simulationpitcher;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.trigon.robot.motorsimulation.MotorSimulationConfiguration;
import frc.trigon.robot.motorsimulation.SingleJointedArmSimulation;
import frc.trigon.robot.subsystems.pitcher.PitcherConstants;

public class SimulationPitcherConstants {
    private static final double CONVERSIONS_FACTOR = 1;
    private static final double VOLTAGE_COMPENSATION_SATURATION = 12;
    private static final int MOTOR_AMOUNT = 1;
    private static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(MOTOR_AMOUNT);
    private static final double GEAR_RATIO = 1;
    private static final double MASS = 12;
    private static final Rotation2d
            MINIMUM_ANGLE = Rotation2d.fromDegrees(0),
            MAXIMUM_ANGLE = Rotation2d.fromDegrees(90);
    private static final boolean SIMULATE_GRAVITY = true;
    private static final double
            P = 0,
            I = 0,
            D = 0,
            KG = 0,
            KV = 0,
            KA = 0,
            KS = 0;
    private static final double
            MAXIMUM_VELOCITY = 0,
            MAXIMUM_ACCELERATION = 0;
    static final SingleJointedArmSimulation MOTOR = new SingleJointedArmSimulation(
            GEARBOX,
            GEAR_RATIO,
            PitcherConstants.PITCHER_LENGTH_METERS,
            MASS,
            MINIMUM_ANGLE,
            MAXIMUM_ANGLE,
            SIMULATE_GRAVITY
    );

    static {
        final MotorSimulationConfiguration config = new MotorSimulationConfiguration();

        config.pidConfigs.kP = P;
        config.pidConfigs.kI = I;
        config.pidConfigs.kD = D;
        config.pidConfigs.enableContinuousInput = true;

        config.feedforwardConfigs.kG = KG;
        config.feedforwardConfigs.kV = KV;
        config.feedforwardConfigs.kA = KA;
        config.feedforwardConfigs.kS = KS;

        config.motionMagicConfigs.maximumVelocity = MAXIMUM_VELOCITY;
        config.motionMagicConfigs.maximumAcceleration = MAXIMUM_ACCELERATION;

        config.voltageCompensationSaturation = VOLTAGE_COMPENSATION_SATURATION;
        config.conversionsFactor = CONVERSIONS_FACTOR;

        MOTOR.applyConfiguration(config);
    }
}
