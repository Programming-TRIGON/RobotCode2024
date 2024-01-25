package frc.trigon.robot.subsystems.pitcher.simulationpitcher;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.trigon.robot.simulation.MotorSimulationConfiguration;
import frc.trigon.robot.simulation.SingleJointedArmSimulation;
import frc.trigon.robot.subsystems.pitcher.PitcherConstants;

public class SimulationPitcherConstants {
    private static final double CONVERSIONS_FACTOR = 1;
    private static final double VOLTAGE_COMPENSATION_SATURATION = 12;
    private static final int MOTOR_AMOUNT = 1;
    private static final DCMotor GEARBOX = DCMotor.getFalcon500Foc(MOTOR_AMOUNT);
    private static final double MASS_KILOGRAMS = 5.5;
    private static final Rotation2d
            MINIMUM_ANGLE = Rotation2d.fromDegrees(13),
            MAXIMUM_ANGLE = Rotation2d.fromDegrees(60);
    private static final boolean SIMULATE_GRAVITY = true;
    private static final double
            P = 3.8346 * 360,
            I = 0,
            D = 0,
            KG = 0.048,
            KV = 0.10527 * 360,
            KA = 0,
            KS = 0.082358;
    private static final double
            MAXIMUM_VELOCITY = 2,
            MAXIMUM_ACCELERATION = 2;
    static final SingleJointedArmSimulation MOTOR = new SingleJointedArmSimulation(
            GEARBOX,
            PitcherConstants.GEAR_RATIO,
            PitcherConstants.PITCHER_LENGTH_METERS,
            MASS_KILOGRAMS,
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
