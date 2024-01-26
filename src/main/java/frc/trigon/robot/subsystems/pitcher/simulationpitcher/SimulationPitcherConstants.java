package frc.trigon.robot.subsystems.pitcher.simulationpitcher;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.trigon.robot.simulation.SingleJointedArmSimulation;
import frc.trigon.robot.subsystems.pitcher.PitcherConstants;

public class SimulationPitcherConstants {
    private static final int MOTOR_AMOUNT = 1;
    private static final DCMotor GEARBOX = DCMotor.getFalcon500Foc(MOTOR_AMOUNT);
    private static final double MASS_KILOGRAMS = 5.5;
    private static final Rotation2d
            MINIMUM_ANGLE = Rotation2d.fromDegrees(13),
            MAXIMUM_ANGLE = Rotation2d.fromDegrees(60);
    private static final boolean SIMULATE_GRAVITY = true;
    private static final double
            P = 100,
            I = 0,
            D = 0,
            KG = 0.04366,
            KV = 39,
            KA = 0.85062,
            KS = 0.053988;
    private static final double
            MAXIMUM_VELOCITY = 2,
            MAXIMUM_ACCELERATION = 2;
    static final SingleJointedArmSimulation MOTOR = new SingleJointedArmSimulation(
            GEARBOX,
            PitcherConstants.GEAR_RATIO,
            PitcherConstants.PITCHER_LENGTH_METERS,
            MASS_KILOGRAMS,
            MINIMUM_ANGLE,
            MAXIMUM_ANGLE,
            SIMULATE_GRAVITY
    );

    static {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = P;
        config.Slot0.kI = I;
        config.Slot0.kD = D;
        config.Slot0.kG = KG;
        config.Slot0.kV = KV;
        config.Slot0.kA = KA;
        config.Slot0.kS = KS;
        config.ClosedLoopGeneral.ContinuousWrap = true;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        config.MotionMagic.MotionMagicCruiseVelocity = MAXIMUM_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = MAXIMUM_ACCELERATION;

        MOTOR.applyConfiguration(config);
    }
}
