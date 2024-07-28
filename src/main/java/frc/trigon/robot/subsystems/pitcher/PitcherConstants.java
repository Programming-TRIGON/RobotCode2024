package frc.trigon.robot.subsystems.pitcher;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.hardware.phoenix6.cancoder.CANcoderEncoder;
import frc.trigon.robot.hardware.phoenix6.cancoder.CANcoderSignal;
import frc.trigon.robot.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.robot.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.robot.utilities.Conversions;
import frc.trigon.robot.utilities.mechanisms.SingleJointedArmMechanism2d;

public class PitcherConstants {
    private static final int
            MOTOR_ID = 11,
            ENCODER_ID = MOTOR_ID;
    static final TalonFXMotor MOTOR = new TalonFXMotor(
            MOTOR_ID,
            "PitcherMotor",
            RobotConstants.CANIVORE_NAME
    );
    static final CANcoderEncoder ENCODER = new CANcoderEncoder(
            ENCODER_ID,
            "PitcherEncoder",
            RobotConstants.CANIVORE_NAME
    );

    private static final NeutralModeValue NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;
    private static final InvertedValue INVERTED_VALUE = InvertedValue.Clockwise_Positive;
    private static final SensorDirectionValue SENSOR_DIRECTION_VALUE = SensorDirectionValue.CounterClockwise_Positive;
    private static final AbsoluteSensorRangeValue ABSOLUTE_SENSOR_RANGE_VALUE = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    private static final FeedbackSensorSourceValue ENCODER_TYPE = FeedbackSensorSourceValue.FusedCANcoder;
    private static final double OFFSET = Conversions.degreesToRevolutions(11.337891 + 90);
    private static final double
            MOTION_MAGIC_P = RobotConstants.IS_SIMULATION ? 100 : 90,
            MOTION_MAGIC_I = 0,
            MOTION_MAGIC_D = RobotConstants.IS_SIMULATION ? 0 : 45,
            POSITION_P = 0,
            POSITION_I = 0,
            POSITION_D = 0,
            KS = RobotConstants.IS_SIMULATION ? 0.053988 : 1.4,
            KV = RobotConstants.IS_SIMULATION ? 39 : 0,
            KA = RobotConstants.IS_SIMULATION ? 0.85062 : 9.2523,
            KG = RobotConstants.IS_SIMULATION ? 0.04366 : 1.2,
            EXPO_KV = RobotConstants.IS_SIMULATION ? KV : 38.757,
            EXPO_KA = RobotConstants.IS_SIMULATION ? KA : 0.6;

    static final int
            MOTION_MAGIC_SLOT = 0,
            POSITION_SLOT = 1;
    static final double PITCHER_LENGTH_METERS = 0.25;
    static final double GEAR_RATIO = 352.8;
    public static final Rotation2d DEFAULT_PITCH = Rotation2d.fromDegrees(30);

    static final double PITCH_TOLERANCE_DEGREES = 0.6;
    static final double SWITCHING_TO_POSITION_CONTROL_TOLERANCE_DEGREES = 2;
    static final SysIdRoutine.Config SYS_ID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(0.5).per(Units.Second),
            Units.Volts.of(5),
            Units.Second.of(1000)
    );

    public static final Pose3d PITCHER_ORIGIN_POINT = new Pose3d(-0.025, 0, 0.2563, new Rotation3d());
    static final SingleJointedArmMechanism2d MECHANISM = new SingleJointedArmMechanism2d(
            "Pitcher", PITCHER_LENGTH_METERS, new Color8Bit(Color.kGreen)
    );

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

        config.Slot1.kP = POSITION_P;
        config.Slot1.kI = POSITION_I;
        config.Slot1.kD = POSITION_D;
        config.Slot1.kG = KG;
        config.Slot1.kS = KS;
        config.Slot1.GravityType = GravityTypeValue.Arm_Cosine;
        config.Slot1.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

        config.MotorOutput.Inverted = INVERTED_VALUE;
        config.MotorOutput.NeutralMode = NEUTRAL_MODE_VALUE;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Conversions.degreesToRevolutions(24);
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Conversions.degreesToRevolutions(90);

        config.Feedback.RotorToSensorRatio = PitcherConstants.GEAR_RATIO;
        config.Feedback.FeedbackRemoteSensorID = PitcherConstants.ENCODER_ID;
        config.Feedback.FeedbackSensorSource = ENCODER_TYPE;

        config.MotionMagic.MotionMagicExpo_kA = EXPO_KA;
        config.MotionMagic.MotionMagicExpo_kV = EXPO_KV;

        MOTOR.applyConfiguration(config);

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

        ENCODER.registerSignal(CANcoderSignal.POSITION, 100);
        ENCODER.registerSignal(CANcoderSignal.VELOCITY, 100);
    }
}
