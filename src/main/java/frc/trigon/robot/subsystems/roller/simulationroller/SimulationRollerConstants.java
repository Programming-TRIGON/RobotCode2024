package frc.trigon.robot.subsystems.roller.simulationroller;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.trigon.robot.simulation.FlywheelSimulation;
import frc.trigon.robot.subsystems.roller.RollerConstants;

public class SimulationRollerConstants {
    private static final int NUMBER_OF_MOTORS = 1;
    private static final double MOMENT_OF_INERTIA = 0.003;
    private static final DCMotor MOTOR_GEARBOX = DCMotor.getKrakenX60Foc(NUMBER_OF_MOTORS);
    private static final double
            KS = 0,
            KV = 0.123,
            KA = 0;

    static final FlywheelSimulation MOTOR = new FlywheelSimulation(
            MOTOR_GEARBOX,
            RollerConstants.GEAR_RATIO,
            MOMENT_OF_INERTIA
    );

    static {
        final TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kS = KS;
        config.Slot0.kV = KV;
        config.Slot0.kA = KA;
        MOTOR.applyConfiguration(config);
    }
}
