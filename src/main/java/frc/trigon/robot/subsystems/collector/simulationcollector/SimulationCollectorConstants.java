package frc.trigon.robot.subsystems.collector.simulationcollector;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.trigon.robot.simulation.FlywheelSimulation;
import frc.trigon.robot.simulation.SingleJointedArmSimulation;
import frc.trigon.robot.subsystems.collector.CollectorConstants;

public class SimulationCollectorConstants {
    private static final int
            ANGLE_MOTOR_AMOUNT = 1,
            COLLECTION_MOTOR_AMOUNT = 1;
    private static final double ANGLE_ARM_LENGTH_METERS = 0.26096919434;
    private static final double ANGLE_ARM_MASS = 4;
    private static final Rotation2d
            MINIMUM_ANGLE = Rotation2d.fromDegrees(0),
            MAXIMUM_ANGLE = Rotation2d.fromDegrees(110);
    private static final boolean SIMULATE_GRAVITY = true;
    private static final double COLLECTION_MOTOR_MOMENT_OF_INERTIA = 1;
    private static final DCMotor
            ANGLE_GEARBOX = DCMotor.getFalcon500Foc(ANGLE_MOTOR_AMOUNT),
            COLLECTION_GEARBOX = DCMotor.getKrakenX60Foc(COLLECTION_MOTOR_AMOUNT);
    private static final double
            ANGLE_P = 270,
            ANGLE_I = 0,
            ANGLE_D = 0,
            ANGLE_KS = 0.061104,
            ANGLE_KG = 0,
            ANGLE_KV = 7.5081,
            ANGLE_KA = 0.17635;
    private static final GravityTypeValue ANGLE_GRAVITY_TYPE = GravityTypeValue.Arm_Cosine;
    private static final double
            ANGLE_MAX_VELOCITY = 8,
            ANGLE_MAX_ACCELERATION = 8;
    static final SingleJointedArmSimulation ANGLE_MOTOR = new SingleJointedArmSimulation(
            ANGLE_GEARBOX,
            CollectorConstants.ANGLE_MOTOR_GEAR_RATIO,
            ANGLE_ARM_LENGTH_METERS,
            ANGLE_ARM_MASS,
            MINIMUM_ANGLE,
            MAXIMUM_ANGLE,
            SIMULATE_GRAVITY
    );
    static final FlywheelSimulation COLLECTION_MOTOR = new FlywheelSimulation(
            COLLECTION_GEARBOX,
            CollectorConstants.COLLECTION_MOTOR_GEAR_RATIO,
            COLLECTION_MOTOR_MOMENT_OF_INERTIA
    );

    static {
        configureAngleMotor();
        configureCollectionMotor();
    }

    private static void configureAngleMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = ANGLE_P;
        config.Slot0.kI = ANGLE_I;
        config.Slot0.kD = ANGLE_D;
        config.Slot0.kS = ANGLE_KS;
        config.Slot0.kG = ANGLE_KG;
        config.Slot0.kV = ANGLE_KV;
        config.Slot0.kA = ANGLE_KA;
        config.Slot0.GravityType = ANGLE_GRAVITY_TYPE;

        config.MotionMagic.MotionMagicCruiseVelocity = ANGLE_MAX_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = ANGLE_MAX_ACCELERATION;

        ANGLE_MOTOR.applyConfiguration(config);
    }

    private static void configureCollectionMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();
        COLLECTION_MOTOR.applyConfiguration(config);
    }
}
