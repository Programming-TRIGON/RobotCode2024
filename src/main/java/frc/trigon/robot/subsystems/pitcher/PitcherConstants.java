package frc.trigon.robot.subsystems.pitcher;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.constants.RobotConstants;
import org.trigon.hardware.RobotHardwareStats;
import org.trigon.hardware.phoenix6.cancoder.CANcoderEncoder;
import org.trigon.hardware.phoenix6.cancoder.CANcoderSignal;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;
import org.trigon.hardware.simulation.SingleJointedArmSimulation;
import org.trigon.utilities.Conversions;
import org.trigon.utilities.mechanisms.SingleJointedArmMechanism2d;

public class PitcherConstants {
    private static final int
            MOTOR_ID = 11,
            ENCODER_ID = MOTOR_ID;
    private static final String
            MOTOR_NAME = "PitcherMotor",
            ENCODER_NAME = "PitcherEncoder";
    static final TalonFXMotor MOTOR = new TalonFXMotor(
            MOTOR_ID,
            MOTOR_NAME,
            RobotConstants.CANIVORE_NAME
    );
    static final CANcoderEncoder ENCODER = new CANcoderEncoder(
            ENCODER_ID,
            ENCODER_NAME,
            RobotConstants.CANIVORE_NAME
    );

    private static final NeutralModeValue NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;
    private static final InvertedValue INVERTED_VALUE = InvertedValue.Clockwise_Positive;
    private static final SensorDirectionValue SENSOR_DIRECTION_VALUE = SensorDirectionValue.CounterClockwise_Positive;
    private static final AbsoluteSensorRangeValue ABSOLUTE_SENSOR_RANGE_VALUE = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    private static final FeedbackSensorSourceValue ENCODER_TYPE = FeedbackSensorSourceValue.RemoteCANcoder;
    public static final double GRAVITY_POSITION_TO_REAL_POSITION = Conversions.degreesToRotations(-118.8756);
    private static final double OFFSET = -0.0487158583333333;
    private static final double
            MOTION_MAGIC_P = RobotHardwareStats.isSimulation() ? 300 : 180,
            MOTION_MAGIC_I = 0,
            MOTION_MAGIC_D = RobotHardwareStats.isSimulation() ? 0 : 0,
            KS = RobotHardwareStats.isSimulation() ? 1.0346 : 0.20177,
            KV = RobotHardwareStats.isSimulation() ? 41 : 40.163,
            KA = RobotHardwareStats.isSimulation() ? 0.85062 : 0,
            KG = RobotHardwareStats.isSimulation() ? 0.04366 : 0.087458;
    private static final double
            MOTION_MAGIC_CRUISE_VELOCITY = 0.28,
            MOTION_MAGIC_ACCELERATION = 4,
            MOTION_MAGIC_JERK = 40;
    static final boolean FOC_ENABLED = true;
    static final double GEAR_RATIO = 355.5;

    private static final int MOTOR_AMOUNT = 1;
    private static final DCMotor GEARBOX = DCMotor.getFalcon500Foc(MOTOR_AMOUNT);
    private static final double MASS_KILOGRAMS = 5.5;
    private static final Rotation2d
            MINIMUM_ANGLE = Rotation2d.fromDegrees(13),
            MAXIMUM_ANGLE = Rotation2d.fromDegrees(90);
    private static final boolean SIMULATE_GRAVITY = true;
    private static final double PITCHER_LENGTH_METERS = 0.25;
    static final SingleJointedArmSimulation SIMULATION = new SingleJointedArmSimulation(
            GEARBOX,
            GEAR_RATIO,
            PITCHER_LENGTH_METERS,
            MASS_KILOGRAMS,
            MINIMUM_ANGLE,
            MAXIMUM_ANGLE,
            SIMULATE_GRAVITY
    );

    static final SysIdRoutine.Config SYS_ID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(1).per(Units.Second),
            Units.Volts.of(4),
            Units.Second.of(1000)
    );

    static final Pose3d PITCHER_ORIGIN_POINT = new Pose3d(-0.025, 0, 0.2563, new Rotation3d());
    static final SingleJointedArmMechanism2d MECHANISM = new SingleJointedArmMechanism2d(
            "PitcherMechanism", PITCHER_LENGTH_METERS, new Color8Bit(Color.kGreen)
    );

    public static final Rotation2d DEFAULT_PITCH = Rotation2d.fromDegrees(30);
    static final double PITCH_TOLERANCE_DEGREES = 0.4;

    static {
        configuredEncoder();
        configureMotor();
    }

    private static void configureMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnConfig = false;
        config.Audio.BeepOnBoot = false;

        config.Slot0.kP = MOTION_MAGIC_P;
        config.Slot0.kI = MOTION_MAGIC_I;
        config.Slot0.kD = MOTION_MAGIC_D;
        config.Slot0.kG = KG;
        config.Slot0.kV = KV;
        config.Slot0.kA = KA;
        config.Slot0.kS = KS;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

        config.MotorOutput.Inverted = INVERTED_VALUE;
        config.MotorOutput.NeutralMode = NEUTRAL_MODE_VALUE;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Conversions.degreesToRotations(29) + PitcherConstants.GRAVITY_POSITION_TO_REAL_POSITION;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Conversions.degreesToRotations(90) + PitcherConstants.GRAVITY_POSITION_TO_REAL_POSITION;

        config.Feedback.RotorToSensorRatio = GEAR_RATIO;
        config.Feedback.FeedbackRemoteSensorID = ENCODER_ID;
        config.Feedback.FeedbackSensorSource = ENCODER_TYPE;

        config.MotionMagic.MotionMagicCruiseVelocity = MOTION_MAGIC_CRUISE_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = MOTION_MAGIC_ACCELERATION;
        config.MotionMagic.MotionMagicJerk = MOTION_MAGIC_JERK;

        MOTOR.applyConfiguration(config);
        MOTOR.setPhysicsSimulation(SIMULATION);

        MOTOR.registerSignal(TalonFXSignal.POSITION, 100);
        MOTOR.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 100);
        MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
        MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        MOTOR.registerSignal(TalonFXSignal.ROTOR_VELOCITY, 100);
        MOTOR.registerSignal(TalonFXSignal.ROTOR_POSITION, 100);
        MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
    }

    private static void configuredEncoder() {
        final CANcoderConfiguration config = new CANcoderConfiguration();

        config.MagnetSensor.MagnetOffset = OFFSET;
        config.MagnetSensor.SensorDirection = SENSOR_DIRECTION_VALUE;
        config.MagnetSensor.AbsoluteSensorRange = ABSOLUTE_SENSOR_RANGE_VALUE;

        ENCODER.applyConfiguration(config);
        ENCODER.setSimulationInputsFromTalonFX(MOTOR);

        ENCODER.registerSignal(CANcoderSignal.POSITION, 100);
        ENCODER.registerSignal(CANcoderSignal.VELOCITY, 100);
    }
}
