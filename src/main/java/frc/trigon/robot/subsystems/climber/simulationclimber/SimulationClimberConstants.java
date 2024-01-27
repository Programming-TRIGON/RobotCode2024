package frc.trigon.robot.subsystems.climber.simulationclimber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.trigon.robot.simulation.ElevatorSimulation;
import frc.trigon.robot.subsystems.climber.ClimberConstants;

public class SimulationClimberConstants {
    private static final int MOTOR_AMOUNT = 2;
    private static final double MASS_KILOGRAMS = 2;
    private static final double MAXIMUM_HEIGHT_METERS = 0.7188;
    private static final boolean SIMULATE_GRAVITY = true;
    private static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(MOTOR_AMOUNT);
    private static final double
            P = 30,
            I = 0,
            D = 0,
            KS = 0.030713,
            KG = 0.009791,
            KV = 2.385,
            KA = 0.049261;
    private static final double
            MAX_VELOCITY = 18,
            MAX_ACCELERATION = 18;
    static final ElevatorSimulation MOTOR = new ElevatorSimulation(
            GEARBOX,
            ClimberConstants.GEAR_RATIO,
            MASS_KILOGRAMS,
            ClimberConstants.DRUM_RADIUS_METERS,
            ClimberConstants.RETRACTED_CLIMBER_LENGTH_METERS,
            MAXIMUM_HEIGHT_METERS,
            SIMULATE_GRAVITY
    );

    static {
        configureMotor();
    }

    private static void configureMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = P;
        config.Slot0.kI = I;
        config.Slot0.kD = D;
        config.Slot0.kS = KS;
        config.Slot0.kG = KG;
        config.Slot0.kV = KV;
        config.Slot0.kA = KA;

        config.MotionMagic.MotionMagicCruiseVelocity = MAX_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = MAX_ACCELERATION;

        MOTOR.applyConfiguration(config);
    }
}
