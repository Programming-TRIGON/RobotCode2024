package frc.trigon.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.geometry.Pose3d;
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
import org.trigon.hardware.simulation.ElevatorSimulation;
import org.trigon.utilities.mechanisms.ElevatorMechanism2d;

public class ElevatorConstants {
    private static final int
            MASTER_MOTOR_ID = 14,
            FOLLOWER_MOTOR_ID = 15,
            ENCODER_ID = 14;
    private static final String
            MASTER_MOTOR_NAME = "ElevatorMasterMotor",
            FOLLOWER_MOTOR_NAME = "ElevatorFollowerMotor",
            ENCODER_NAME = "ElevatorEncoder";
    static final TalonFXMotor
            MASTER_MOTOR = new TalonFXMotor(MASTER_MOTOR_ID, MASTER_MOTOR_NAME, RobotConstants.CANIVORE_NAME),
            FOLLOWER_MOTOR = new TalonFXMotor(FOLLOWER_MOTOR_ID, FOLLOWER_MOTOR_NAME, RobotConstants.CANIVORE_NAME);
    static final CANcoderEncoder ENCODER = new CANcoderEncoder(ENCODER_ID, ENCODER_NAME, RobotConstants.CANIVORE_NAME);

    private static final NeutralModeValue NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;
    private static final InvertedValue
            MASTER_MOTOR_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive,
            FOLLOWER_MOTOR_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;
    private static final boolean FOLLOWER_MOTOR_OPPOSITE_DIRECTION = true;
    private static final AbsoluteSensorRangeValue ENCODER_SENSOR_RANGE_VALUE = AbsoluteSensorRangeValue.Unsigned_0To1;
    private static final SensorDirectionValue ENCODER_SENSOR_DIRECTION_VALUE = SensorDirectionValue.CounterClockwise_Positive;
    private static final double ENCODER_MAGNET_OFFSET_VALUE = -0.711181640625;
    private static final FeedbackSensorSourceValue ENCODER_TYPE = FeedbackSensorSourceValue.RemoteCANcoder;
    private static final double
            P = RobotHardwareStats.isSimulation() ? 52 : 1.5,
            I = 0,
            D = 0,
            KS = RobotHardwareStats.isSimulation() ? 0.019539 : 0.02,
            KV = RobotHardwareStats.isSimulation() ? 0.987 : 0.415,
            KG = RobotHardwareStats.isSimulation() ? 0.12551 : 0.37,
            KA = RobotHardwareStats.isSimulation() ? 0.017514 : 0.01;
    static final double
            MOTION_MAGIC_CRUISE_VELOCITY = 23,
            MOTION_MAGIC_ACCELERATION = 30,
            MOTION_MAGIC_JERK = MOTION_MAGIC_ACCELERATION * 7;
    static final double GEAR_RATIO = 3.2;
    static final boolean FOC_ENABLED = true;

    private static final double
            MASS_KILOGRAMS = 5.5,
            DRUM_RADIUS_METERS = 0.0222997564,
            MAXIMUM_HEIGHT_METERS = 1.111,
            RETRACTED_ELEVATOR_LENGTH_METERS = 0.63;
    private static final int MOTOR_AMOUNT = 2;
    private static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(MOTOR_AMOUNT);
    private static final ElevatorSimulation SIMULATION = new ElevatorSimulation(GEARBOX, GEAR_RATIO, MASS_KILOGRAMS, DRUM_RADIUS_METERS, RETRACTED_ELEVATOR_LENGTH_METERS, MAXIMUM_HEIGHT_METERS, true);

    static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(0.25).per(Units.Second.of(1)),
            Units.Volts.of(1),
            Units.Second.of(1000)
    );

    static final Pose3d
            ELEVATOR_ORIGIN_POINT = new Pose3d(0.10018, 0, 0.04, new Rotation3d(0, edu.wpi.first.math.util.Units.degreesToRadians(10), 0)),
            TRANSPORTER_ORIGIN_POINT = new Pose3d(0.10018, 0, 0.06, new Rotation3d(0, edu.wpi.first.math.util.Units.degreesToRadians(10), 0));
    static final ElevatorMechanism2d MECHANISM = new ElevatorMechanism2d(
            "ElevatorMechanism",
            MAXIMUM_HEIGHT_METERS,
            RETRACTED_ELEVATOR_LENGTH_METERS,
            new Color8Bit(Color.kYellow)
    );

    static final double
            TOLERANCE_METERS = 0.035,
            DRUM_DIAMETER_METERS = DRUM_RADIUS_METERS * 2,
            CAMERA_PLATE_HEIGHT_METERS = 0.190193;

    static {
        configureEncoder();
        configureMasterMotor();
        configureFollowerMotor();
    }

    private static void configureEncoder() {
        final CANcoderConfiguration config = new CANcoderConfiguration();

        config.MagnetSensor.AbsoluteSensorRange = ENCODER_SENSOR_RANGE_VALUE;
        config.MagnetSensor.SensorDirection = ENCODER_SENSOR_DIRECTION_VALUE;
        config.MagnetSensor.MagnetOffset = ENCODER_MAGNET_OFFSET_VALUE;

        ENCODER.applyConfiguration(config);
        ENCODER.setSimulationInputsFromTalonFX(MASTER_MOTOR);

        ENCODER.registerSignal(CANcoderSignal.POSITION, 100);
        ENCODER.registerSignal(CANcoderSignal.VELOCITY, 100);
    }

    private static void configureMasterMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = NEUTRAL_MODE_VALUE;
        config.MotorOutput.Inverted = MASTER_MOTOR_INVERTED_VALUE;

        config.Feedback.FeedbackRemoteSensorID = ENCODER_ID;
        config.Feedback.FeedbackSensorSource = ENCODER_TYPE;
        config.Feedback.RotorToSensorRatio = GEAR_RATIO;

        config.Slot0.kP = P;
        config.Slot0.kI = I;
        config.Slot0.kD = D;
        config.Slot0.kS = KS;
        config.Slot0.kV = KV;
        config.Slot0.kG = KG;
        config.Slot0.kA = KA;
        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.01;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 4.440674;

        config.MotionMagic.MotionMagicCruiseVelocity = MOTION_MAGIC_CRUISE_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = MOTION_MAGIC_ACCELERATION;
        config.MotionMagic.MotionMagicJerk = MOTION_MAGIC_JERK;

        MASTER_MOTOR.applyConfiguration(config);
        MASTER_MOTOR.setPhysicsSimulation(SIMULATION);

        MASTER_MOTOR.registerSignal(TalonFXSignal.POSITION, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.ROTOR_POSITION, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.ROTOR_VELOCITY, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 100);
    }

    private static void configureFollowerMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = NEUTRAL_MODE_VALUE;
        config.MotorOutput.Inverted = FOLLOWER_MOTOR_INVERTED_VALUE;

        FOLLOWER_MOTOR.applyConfiguration(config);

        FOLLOWER_MOTOR.setControl(new Follower(MASTER_MOTOR_ID, FOLLOWER_MOTOR_OPPOSITE_DIRECTION));
    }

    public enum ElevatorState {
        RESTING(-0.005, 100),
        FEEDING_FOR_CLOSE_SHOT(0, 100),
        SCORE_AMP(0.45, 100),
        SCORE_TRAP(0.5, 10),
        SCORE_TRAP_LOWERED(0.15, 10),
        FINISH_TRAP(0, 10);

        final double positionMeters;
        final double speedPercentage;

        ElevatorState(double positionMeters, double speedPercentage) {
            this.positionMeters = positionMeters;
            this.speedPercentage = speedPercentage;
        }
    }
}