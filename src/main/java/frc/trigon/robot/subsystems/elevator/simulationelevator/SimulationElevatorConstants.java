package frc.trigon.robot.subsystems.elevator.simulationelevator;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.trigon.robot.simulation.ElevatorSimulation;
import frc.trigon.robot.simulation.MotorSimulationConfiguration;
import frc.trigon.robot.subsystems.elevator.ElevatorConstants;

public class SimulationElevatorConstants {
    static final double VOLTAGE_COMPENSATION_SATURATION = 12;
    private static final int NUMBER_OF_MOTORS = 2;
    private static final double MASS_KILOGRAMS = 10;
    private static final double MAXIMUM_HEIGHT_METERS = 1;
    private static final double CONVERSIONS_FACTOR = 1;
    private static final double
            P = 0,
            I = 0,
            D = 0,
            KS = 0,
            KV = 0,
            KG = 0,
            KA = 0;
    private static final double
            MAXIMUM_ACCELERATION = 0,
            MAXIMUM_VELOCITY = 0;
    private static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(MAXIMUM_VELOCITY, MAXIMUM_ACCELERATION);
    static final ProfiledPIDController PROFILED_PID_CONTROLLER = new ProfiledPIDController(P, I, D, CONSTRAINTS);
    static final SimpleMotorFeedforward FEEDFORWARD = new SimpleMotorFeedforward(KS, KV, KA);
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
        final MotorSimulationConfiguration config = new MotorSimulationConfiguration();

        config.voltageCompensationSaturation = VOLTAGE_COMPENSATION_SATURATION;
        config.conversionsFactor = CONVERSIONS_FACTOR;

        config.pidConfigs.kP = P;
        config.pidConfigs.kI = I;
        config.pidConfigs.kD = D;

        config.feedforwardConfigs.kS = KS;
        config.feedforwardConfigs.kV = KV;
        config.feedforwardConfigs.kG = KG;
        config.feedforwardConfigs.kA = KA;

        config.motionMagicConfigs.maximumAcceleration = MAXIMUM_ACCELERATION;
        config.motionMagicConfigs.maximumVelocity = MAXIMUM_VELOCITY;

        MOTOR.applyConfiguration(config);
    }
}
