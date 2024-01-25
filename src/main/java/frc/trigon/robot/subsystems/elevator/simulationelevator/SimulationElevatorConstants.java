package frc.trigon.robot.subsystems.elevator.simulationelevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.trigon.robot.simulation.ElevatorSimulation;
import frc.trigon.robot.subsystems.elevator.ElevatorConstants;

public class SimulationElevatorConstants {
    private static final int NUMBER_OF_MOTORS = 2;
    private static final double MASS_KILOGRAMS = 5.5;
    private static final double MAXIMUM_HEIGHT_METERS = 1.111;
    private static final double
            P = 0,
            I = 0,
            D = 0,
            KS = 0.13021,
            KV = 4.5821,
            KG = 0.11799,
            KA = 0.23603;
    private static final double
            MAXIMUM_ACCELERATION = 100,
            MAXIMUM_VELOCITY = 100;
    private static final boolean SIMULATE_GRAVITY = true;
    private static final DCMotor MOTOR_GEARBOX = DCMotor.getKrakenX60Foc(NUMBER_OF_MOTORS);

    static final ElevatorSimulation MOTOR = new ElevatorSimulation(
            MOTOR_GEARBOX,
            ElevatorConstants.GEAR_RATIO,
            MASS_KILOGRAMS,
            ElevatorConstants.DRUM_RADIUS_METERS,
            ElevatorConstants.RETRACTED_ELEVATOR_LENGTH_METERS,
            MAXIMUM_HEIGHT_METERS,
            SIMULATE_GRAVITY
    );

    static {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = P;
        config.Slot0.kI = I;
        config.Slot0.kD = D;
        config.Slot0.kS = KS;
        config.Slot0.kV = KV;
        config.Slot0.kG = KG;
        config.Slot0.kA = KA;

        config.MotionMagic.MotionMagicAcceleration = MAXIMUM_ACCELERATION;
        config.MotionMagic.MotionMagicCruiseVelocity = MAXIMUM_VELOCITY;

        MOTOR.applyConfiguration(config);
    }
}
