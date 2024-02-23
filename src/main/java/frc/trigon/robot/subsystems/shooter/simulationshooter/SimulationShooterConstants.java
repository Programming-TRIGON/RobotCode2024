package frc.trigon.robot.subsystems.shooter.simulationshooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.trigon.robot.simulation.FlywheelSimulation;
import frc.trigon.robot.subsystems.shooter.ShooterConstants;

public class SimulationShooterConstants {
    private static final int MOTOR_AMOUNT = 2;
    private static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(MOTOR_AMOUNT);
    private static final double MOMENT_OF_INERTIA = 0.003;
    private static final double
            P = 0,
            I = 0,
            D = 0,
            KV = 0.14142,
            KA = 0.038638,
            KS = 0.59843;
    static final FlywheelSimulation MOTOR = new FlywheelSimulation(GEARBOX, ShooterConstants.GEAR_RATIO, MOMENT_OF_INERTIA);

    static {
        configureMotor();
    }

    private static void configureMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = P;
        config.Slot0.kI = I;
        config.Slot0.kD = D;
        config.Slot0.kV = KV;
        config.Slot0.kA = KA;
        config.Slot0.kS = KS;

        MOTOR.applyConfiguration(config);
    }
}
