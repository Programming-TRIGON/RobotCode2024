package frc.trigon.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.hardware.misc.digitalsensor.SimpleSensorInputs;
import frc.trigon.robot.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.robot.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.robot.utilities.mechanisms.ElevatorMechanism2d;

import java.util.function.BooleanSupplier;

public class ClimberConstants {
    public static final double
            DRUM_RADIUS_METERS = 0.02,
            DRUM_DIAMETER_METERS = DRUM_RADIUS_METERS * 2;
    static final double LIMIT_SWITCH_PRESSED_THRESHOLD_SECONDS = 0.2;
    static final double RETRACTED_CLIMBER_LENGTH_METERS = 0.185;
    private static final double MAXIMUM_HEIGHT_METERS = 0.7188;
    static final boolean ENABLE_FOC = true;
    static final double GEAR_RATIO = 19.64;
    static final double TOLERANCE_METERS = 0.01;
    static final double READY_FOR_ELEVATOR_OPENING_MAXIMUM_POSITION_METERS = 0.2;
    static final double
            MAX_NON_CLIMBING_VELOCITY = 20,
            MAX_NON_CLIMBING_ACCELERATION = 20,
            MAX_CLIMBING_VELOCITY = 1,
            MAX_CLIMBING_ACCELERATION = 1;

    private static final int
            MASTER_MOTOR_ID = 12,
            FOLLOWER_MOTOR_ID = 13;
    private static final String
            MASTER_MOTOR_NAME = "ClimberMasterMotor",
            FOLLOWER_MOTOR_NAME = "ClimberFollowerMotor";
    static final TalonFXMotor
            MASTER_MOTOR = new TalonFXMotor(
            MASTER_MOTOR_ID,
            MASTER_MOTOR_NAME,
            RobotConstants.CANIVORE_NAME
    ),
            FOLLOWER_MOTOR = new TalonFXMotor(
                    FOLLOWER_MOTOR_ID,
                    FOLLOWER_MOTOR_NAME,
                    RobotConstants.CANIVORE_NAME
            );
    private static final int LIMIT_SWITCH_CHANNEL = 0;
    private static final String LIMIT_SWITCH_NAME = "ClimberLimitSwitch";
    private static final BooleanSupplier IS_TRIGGERED_IN_SIMULATION_SUPPLIER = () -> false;
    static final SimpleSensorInputs LIMIT_SWITCH = new SimpleSensorInputs(LIMIT_SWITCH_NAME, LIMIT_SWITCH_CHANNEL);
    private static final InvertedValue
            MASTER_MOTOR_INVERTED_VALUE = InvertedValue.Clockwise_Positive,
            FOLLOWER_MOTOR_INVERTED_VALUE = InvertedValue.Clockwise_Positive;
    private static final boolean FOLLOWER_MOTOR_OPPOSITE_DIRECTION = false;
    private static final NeutralModeValue NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;
    private static final double
            NON_CLIMBING_P = RobotConstants.IS_SIMULATION ? 30 : 0,
            NON_CLIMBING_I = 0,
            NON_CLIMBING_D = 0,
            NON_CLIMBING_KS = RobotConstants.IS_SIMULATION ? 0.030713 : 0.3081,
            NON_CLIMBING_KG = RobotConstants.IS_SIMULATION ? 0.009791 : -0.094675,
            NON_CLIMBING_KV = RobotConstants.IS_SIMULATION ? 2.385 : 2.3525,
            NON_CLIMBING_KA = RobotConstants.IS_SIMULATION ? 0.049261 : 0.045652;
    private static final double
            CLIMBING_P = RobotConstants.IS_SIMULATION ? 30 : 15,
            CLIMBING_I = 0,
            CLIMBING_D = 0,
            CLIMBING_KS = RobotConstants.IS_SIMULATION ? 0.030713 : 0.30095,
            CLIMBING_KG = RobotConstants.IS_SIMULATION ? 0.009791 : -0.58087,
            CLIMBING_KV = RobotConstants.IS_SIMULATION ? 2.385 : 2.2353,
            CLIMBING_KA = RobotConstants.IS_SIMULATION ? 0.049261 : 0.061477;
    static final int
            NON_CLIMBING_SLOT = 0,
            CLIMBING_SLOT = 1;
    static final Pose3d CLIMBER_ORIGIN_POINT = new Pose3d(0.16636, 0, 0.118, new Rotation3d(0, edu.wpi.first.math.util.Units.degreesToRadians(-15), 0));
    static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(1.5).per(Units.Second.of(1)),
            Units.Volts.of(8),
            null,
            null
    );
    static final ElevatorMechanism2d MECHANISM = new ElevatorMechanism2d(
            "ClimberMechanism", MAXIMUM_HEIGHT_METERS, RETRACTED_CLIMBER_LENGTH_METERS, new Color8Bit(Color.kRed)
    );

    static {
        configureMasterClimbingMotor();
        configureFollowerClimbingMotor();
        configureLimitSwitch();
    }

    private static void configureMasterClimbingMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = MASTER_MOTOR_INVERTED_VALUE;
        config.MotorOutput.NeutralMode = NEUTRAL_MODE_VALUE;
        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;
//
//        config.Slot0.kP = NON_CLIMBING_P;
//        config.Slot0.kI = NON_CLIMBING_I;
//        config.Slot0.kD = NON_CLIMBING_D;
//        config.Slot0.kS = NON_CLIMBING_KS;
//        config.Slot0.kG = NON_CLIMBING_KG;
//        config.Slot0.kV = NON_CLIMBING_KV;
//        config.Slot0.kA = NON_CLIMBING_KA;

        config.Slot1.kP = CLIMBING_P;
        config.Slot1.kI = CLIMBING_I;
        config.Slot1.kD = CLIMBING_D;
        config.Slot1.kS = CLIMBING_KS;
        config.Slot1.kG = CLIMBING_KG;
        config.Slot1.kV = CLIMBING_KV;
        config.Slot1.kA = CLIMBING_KA;
//        config.Slot1.kP = NON_CLIMBING_P;
//        config.Slot1.kI = NON_CLIMBING_I;
//        config.Slot1.kD = NON_CLIMBING_D;
//        config.Slot1.kS = NON_CLIMBING_KS;
//        config.Slot1.kG = NON_CLIMBING_KG;
//        config.Slot1.kV = NON_CLIMBING_KV;
//        config.Slot1.kA = NON_CLIMBING_KA;

        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

        MASTER_MOTOR.applyConfiguration(config);

        MASTER_MOTOR.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.TORQUE_CURRENT, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.POSITION, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
    }

    private static void configureLimitSwitch() {
        LIMIT_SWITCH.setSimulationSupplier(IS_TRIGGERED_IN_SIMULATION_SUPPLIER);
    }

    private static void configureFollowerClimbingMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = FOLLOWER_MOTOR_INVERTED_VALUE;
        config.MotorOutput.NeutralMode = NEUTRAL_MODE_VALUE;
        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        FOLLOWER_MOTOR.applyConfiguration(config);

        final Follower followerRequest = new Follower(MASTER_MOTOR_ID, FOLLOWER_MOTOR_OPPOSITE_DIRECTION);
        FOLLOWER_MOTOR.setControl(followerRequest);
    }

    public enum ClimberState {
        RESTING(0, false),
        CLIMB(-0.03, true),
        CLIMB_MIDDLE(0.23, true),
        CLIMB_FINISH(0.05, true),
        CLIMBING_PREPARATION(0.620495, false);

        final double positionMeters;
        final boolean affectedByWeight;

        ClimberState(double positionMeters, boolean affectedByWeight) {
            this.positionMeters = positionMeters;
            this.affectedByWeight = affectedByWeight;
        }
    }
}
