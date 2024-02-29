package frc.trigon.robot.subsystems.intake.simulationintake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.trigon.robot.simulation.FlywheelSimulation;
import frc.trigon.robot.subsystems.intake.IntakeConstants;

public class SimulationIntakeConstants {
    private static final int MOTOR_AMOUNT = 1;
    private static final double MOMENT_OF_INERTIA = 1;
    private static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(MOTOR_AMOUNT);
    static final FlywheelSimulation MOTOR = new FlywheelSimulation(
            GEARBOX,
            IntakeConstants.GEAR_RATIO,
            MOMENT_OF_INERTIA
    );

    static {
        configureCollectionMotor();
    }

    private static void configureCollectionMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();
        MOTOR.applyConfiguration(config);
    }
}
