package frc.trigon.robot.subsystems.elevator.simulationelevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.trigon.robot.simulation.ElevatorSimulation;
import frc.trigon.robot.subsystems.elevator.ElevatorConstants;

public class SimulationElevatorConstants {
    private static final int NUMBER_OF_MOTORS = 2;
    private static final double MASS_KILOGRAMS = 5.5;
    private static final double MAXIMUM_HEIGHT_METERS = 1.111;
    private static final GravityTypeValue GRAVITY_TYPE = GravityTypeValue.Elevator_Static;
    private static final double
            P = 52,
            I = 0,
            D = 0,
            KS = 0.019539,
            KV = 0.987,
            KG = 0.11151,
            KA = 0.017514;
    private static final double
            MAXIMUM_ACCELERATION = 9,
            MAXIMUM_VELOCITY = 9;
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
        config.Slot0.GravityType = GRAVITY_TYPE;

        config.MotionMagic.MotionMagicAcceleration = MAXIMUM_ACCELERATION;
        config.MotionMagic.MotionMagicCruiseVelocity = MAXIMUM_VELOCITY;

        MOTOR.applyConfiguration(config);
    }
}
