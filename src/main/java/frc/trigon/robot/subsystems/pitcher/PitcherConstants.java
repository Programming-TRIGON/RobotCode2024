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
import frc.trigon.robot.hardware.phoenix6.cancoder.CANcoderEncoder;
import frc.trigon.robot.hardware.phoenix6.cancoder.CANcoderSignal;
import frc.trigon.robot.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.robot.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.robot.hardware.simulation.SingleJointedArmSimulation;
import frc.trigon.robot.utilities.Conversions;
import frc.trigon.robot.utilities.mechanisms.SingleJointedArmMechanism2d;

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
    private static final FeedbackSensorSourceValue ENCODER_TYPE = FeedbackSensorSourceValue.FusedCANcoder;
    private static final double OFFSET = Conversions.degreesToRotations(11.337891 + 90);
    private static final double
            MOTION_MAGIC_P = RobotConstants.IS_SIMULATION ? 100 : 90,
            MOTION_MAGIC_I = 0,
            MOTION_MAGIC_D = RobotConstants.IS_SIMULATION ? 0 : 45,
            KS = RobotConstants.IS_SIMULATION ? 0.053988 : 1.4,
            KV = RobotConstants.IS_SIMULATION ? 39 : 0,
            KA = RobotConstants.IS_SIMULATION ? 0.85062 : 9.2523,
            KG = RobotConstants.IS_SIMULATION ? 0.04366 : 1.2,
            EXPO_KV = RobotConstants.IS_SIMULATION ? KV : 38.757,
            EXPO_KA = RobotConstants.IS_SIMULATION ? KA : 0.6;
    static final boolean FOC_ENABLED = true;
    private static final double GEAR_RATIO = 352.8;

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
            Units.Volts.of(0.5).per(Units.Second),
            Units.Volts.of(5),
            Units.Second.of(1000)
    );

    static final Pose3d PITCHER_ORIGIN_POINT = new Pose3d(-0.025, 0, 0.2563, new Rotation3d());
    static final SingleJointedArmMechanism2d MECHANISM = new SingleJointedArmMechanism2d(
            "PitcherMechanism", PITCHER_LENGTH_METERS, new Color8Bit(Color.kGreen)
    );

    public static final Rotation2d DEFAULT_PITCH = Rotation2d.fromDegrees(30);
    static final double PITCH_TOLERANCE_DEGREES = 0.6;

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
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        config.MotorOutput.Inverted = INVERTED_VALUE;
        config.MotorOutput.NeutralMode = NEUTRAL_MODE_VALUE;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Conversions.degreesToRotations(24);
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Conversions.degreesToRotations(90);

        config.Feedback.RotorToSensorRatio = PitcherConstants.GEAR_RATIO;
        config.Feedback.FeedbackRemoteSensorID = PitcherConstants.ENCODER_ID;
        config.Feedback.FeedbackSensorSource = ENCODER_TYPE;

        config.MotionMagic.MotionMagicExpo_kA = EXPO_KA;
        config.MotionMagic.MotionMagicExpo_kV = EXPO_KV;

        MOTOR.applyConfiguration(config);
        MOTOR.setPhysicsSimulation(SIMULATION);

        MOTOR.registerSignal(TalonFXSignal.POSITION, 100);
        MOTOR.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 100);
        MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
        MOTOR.registerSignal(TalonFXSignal.TORQUE_CURRENT, 100);
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
