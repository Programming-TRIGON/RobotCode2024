package frc.trigon.robot.subsystems.shooter.simulationshooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc.trigon.robot.simulation.FlywheelSimulation;
import frc.trigon.robot.subsystems.shooter.ShooterConstants;

public class SimulationShooterConstants {
    static final FlywheelSimulation MOTOR = new FlywheelSimulation(ShooterConstants.GEARBOX, ShooterConstants.GEAR_RATIO, ShooterConstants.MOMENT_OF_INERTIA);

    static {
        configureMotor();
    }

    private static void configureMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();
        MOTOR.applyConfiguration(config);
    }
}
