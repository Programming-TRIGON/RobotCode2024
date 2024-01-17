package frc.trigon.robot.subsystems.shooter.simulationshooter;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.trigon.robot.simulation.MotorSimulationConfiguration;
import frc.trigon.robot.simulation.SimpleMotorSimulation;
import frc.trigon.robot.subsystems.shooter.ShooterConstants;

public class SimulationShooterConstants {
    private static final double CONVERSIONS_FACTOR = 1;
    private static final double VOLTAGE_COMPENSATION_SATURATION = 12;
    private static final int
            TOP_MOTOR_AMOUNT = 1,
            BOTTOM_MOTOR_AMOUNT = 1;
    private static final DCMotor
            TOP_GEARBOX = DCMotor.getKrakenX60Foc(TOP_MOTOR_AMOUNT),
            BOTTOM_GEARBOX = DCMotor.getKrakenX60Foc(BOTTOM_MOTOR_AMOUNT);
    private static final double
            TOP_MOMENT_OF_INERTIA = 0.0001,
            BOTTOM_MOMENT_OF_INERTIA = 0.0001;
    private static final double
            TOP_MOTOR_KS = 0.24958,
            TOP_MOTOR_KV = 0.12401,
            TOP_MOTOR_KA = 0.0092721,
            BOTTOM_MOTOR_KS = 0.33493,
            BOTTOM_MOTOR_KV = 0.12104,
            BOTTOM_MOTOR_KA = 0.037;
    static final SimpleMotorSimulation
            TOP_MOTOR = new SimpleMotorSimulation(TOP_GEARBOX, ShooterConstants.SHOOTER_GEAR_RATIO, TOP_MOMENT_OF_INERTIA),
            BOTTOM_MOTOR = new SimpleMotorSimulation(BOTTOM_GEARBOX, ShooterConstants.SHOOTER_GEAR_RATIO, BOTTOM_MOMENT_OF_INERTIA);

    static {
        configureShootingMotor();
        configureBottomMotor();
    }

    private static void configureShootingMotor() {
        final MotorSimulationConfiguration config = new MotorSimulationConfiguration();

        config.feedforwardConfigs.kS = TOP_MOTOR_KS;
        config.feedforwardConfigs.kV = TOP_MOTOR_KV;
        config.feedforwardConfigs.kA = TOP_MOTOR_KA;

        config.voltageCompensationSaturation = VOLTAGE_COMPENSATION_SATURATION;
        config.conversionsFactor = CONVERSIONS_FACTOR;

        TOP_MOTOR.applyConfiguration(config);
    }

    private static void configureBottomMotor() {
        final MotorSimulationConfiguration config = new MotorSimulationConfiguration();

        config.feedforwardConfigs.kS = BOTTOM_MOTOR_KS;
        config.feedforwardConfigs.kV = BOTTOM_MOTOR_KV;
        config.feedforwardConfigs.kA = BOTTOM_MOTOR_KA;

        config.voltageCompensationSaturation = VOLTAGE_COMPENSATION_SATURATION;
        config.conversionsFactor = CONVERSIONS_FACTOR;

        BOTTOM_MOTOR.applyConfiguration(config);
    }
}
