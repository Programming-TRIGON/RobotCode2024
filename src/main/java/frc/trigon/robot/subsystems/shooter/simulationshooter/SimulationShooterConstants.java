package frc.trigon.robot.subsystems.shooter.simulationshooter;

import frc.trigon.robot.simulation.FlywheelSimulation;
import frc.trigon.robot.simulation.MotorSimulationConfiguration;
import frc.trigon.robot.subsystems.shooter.ShooterConstants;

public class SimulationShooterConstants {
    private static final double CONVERSIONS_FACTOR = 1;
    private static final double VOLTAGE_COMPENSATION_SATURATION = 12;
    static final FlywheelSimulation MOTOR = new FlywheelSimulation(ShooterConstants.GEARBOX, ShooterConstants.GEAR_RATIO, ShooterConstants.MOMENT_OF_INERTIA);

    static {
        configureMotor();
    }

    private static void configureMotor() {
        final MotorSimulationConfiguration config = new MotorSimulationConfiguration();

        config.voltageCompensationSaturation = VOLTAGE_COMPENSATION_SATURATION;
        config.conversionsFactor = CONVERSIONS_FACTOR;

        MOTOR.applyConfiguration(config);
    }
}
