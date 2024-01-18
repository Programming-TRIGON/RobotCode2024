package frc.trigon.robot.subsystems.shooter.simulationshooter;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.trigon.robot.simulation.FlywheelSimulation;
import frc.trigon.robot.simulation.MotorSimulationConfiguration;
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
    static final FlywheelSimulation
            TOP_MOTOR = new FlywheelSimulation(TOP_GEARBOX, ShooterConstants.GEAR_RATIO, ShooterConstants.TOP_MOMENT_OF_INERTIA),
            BOTTOM_MOTOR = new FlywheelSimulation(BOTTOM_GEARBOX, ShooterConstants.GEAR_RATIO, ShooterConstants.BOTTOM_MOMENT_OF_INERTIA);

    static {
        configureShootingMotor();
        configureBottomMotor();
    }

    private static void configureShootingMotor() {
        final MotorSimulationConfiguration config = new MotorSimulationConfiguration();

        config.voltageCompensationSaturation = VOLTAGE_COMPENSATION_SATURATION;
        config.conversionsFactor = CONVERSIONS_FACTOR;

        TOP_MOTOR.applyConfiguration(config);
    }

    private static void configureBottomMotor() {
        final MotorSimulationConfiguration config = new MotorSimulationConfiguration();

        config.voltageCompensationSaturation = VOLTAGE_COMPENSATION_SATURATION;
        config.conversionsFactor = CONVERSIONS_FACTOR;

        BOTTOM_MOTOR.applyConfiguration(config);
    }
}
