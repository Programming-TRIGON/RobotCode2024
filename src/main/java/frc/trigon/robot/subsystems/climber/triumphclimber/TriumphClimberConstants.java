package frc.trigon.robot.subsystems.climber.triumphclimber;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.climber.ClimberConstants;

public class TriumphClimberConstants {
    static final boolean ENABLE_FOC = true;
    private static final int
            MASTER_MOTOR_ID = 12,
            FOLLOWER_MOTOR_ID = 13;
    private static final InvertedValue
            MASTER_MOTOR_INVERTED_VALUE = InvertedValue.Clockwise_Positive,
            FOLLOWER_MOTOR_INVERTED_VALUE = InvertedValue.Clockwise_Positive;
    private static final boolean FOLLOWER_MOTOR_OPPOSITE_DIRECTION = false;
    private static final NeutralModeValue NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;
    static final int
            NON_CLIMBING_SLOT = 0,
            CLIMBING_SLOT = 1;
    private static final double
            NON_CLIMBING_P = 15,
            NON_CLIMBING_I = 0,
            NON_CLIMBING_D = 0,
            NON_CLIMBING_KS = 0.3081,
            NON_CLIMBING_KG = -0.094675,
            NON_CLIMBING_KV = 2.3525,
            NON_CLIMBING_KA = 0.045652;
    private static final double
            CLIMBING_P = 0,
            CLIMBING_I = 0,
            CLIMBING_D = 0,
            CLIMBING_KS = 0,
            CLIMBING_KG = 0,
            CLIMBING_KV = 0,
            CLIMBING_KA = 0;
    static final double
            MAX_NON_CLIMBING_VELOCITY = 20,
            MAX_NON_CLIMBING_ACCELERATION = 20,
            MAX_CLIMBING_VELOCITY = 1,
            MAX_CLIMBING_ACCELERATION = 1;
    static final TalonFX
            MASTER_MOTOR = new TalonFX(MASTER_MOTOR_ID, RobotConstants.CANIVORE_NAME),
            FOLLOWER_MOTOR = new TalonFX(FOLLOWER_MOTOR_ID, RobotConstants.CANIVORE_NAME);

    private static final int LIMIT_SWITCH_CHANNEL = 3;
    static final DigitalInput LIMIT_SWITCH = new DigitalInput(LIMIT_SWITCH_CHANNEL);

    static final StatusSignal<Double>
            POSITION_SIGNAL = MASTER_MOTOR.getPosition().clone(),
            VELOCITY_SIGNAL = MASTER_MOTOR.getVelocity().clone(),
            MOTOR_SETPOINT_SIGNAL = MASTER_MOTOR.getClosedLoopReference().clone(),
            MOTOR_VOLTAGE_SIGNAL = MASTER_MOTOR.getMotorVoltage().clone(),
            MOTOR_CURRENT_SIGNAL = MASTER_MOTOR.getSupplyCurrent().clone();

    static {
        configureMasterClimbingMotor();
        configureFollowerClimbingMotor();
    }

    private static void configureMasterClimbingMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = MASTER_MOTOR_INVERTED_VALUE;
        config.MotorOutput.NeutralMode = NEUTRAL_MODE_VALUE;
        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.Slot0.kP = NON_CLIMBING_P;
        config.Slot0.kI = NON_CLIMBING_I;
        config.Slot0.kD = NON_CLIMBING_D;
        config.Slot0.kS = NON_CLIMBING_KS;
        config.Slot0.kG = NON_CLIMBING_KG;
        config.Slot0.kV = NON_CLIMBING_KV;
        config.Slot0.kA = NON_CLIMBING_KA;

//        config.Slot1.kP = CLIMBING_P;
//        config.Slot1.kI = CLIMBING_I;
//        config.Slot1.kD = CLIMBING_D;
//        config.Slot1.kS = CLIMBING_KS;
//        config.Slot1.kG = CLIMBING_KG;
//        config.Slot1.kV = CLIMBING_KV;
//        config.Slot1.kA = CLIMBING_KA;
        config.Slot1.kP = NON_CLIMBING_P;
        config.Slot1.kI = NON_CLIMBING_I;
        config.Slot1.kD = NON_CLIMBING_D;
        config.Slot1.kS = NON_CLIMBING_KS;
        config.Slot1.kG = NON_CLIMBING_KG;
        config.Slot1.kV = NON_CLIMBING_KV;
        config.Slot1.kA = NON_CLIMBING_KA;

        config.Feedback.SensorToMechanismRatio = ClimberConstants.GEAR_RATIO;

        MASTER_MOTOR.getConfigurator().apply(config);

        MOTOR_SETPOINT_SIGNAL.setUpdateFrequency(100);
        MOTOR_VOLTAGE_SIGNAL.setUpdateFrequency(100);
        MOTOR_CURRENT_SIGNAL.setUpdateFrequency(100);
        POSITION_SIGNAL.setUpdateFrequency(100);
        VELOCITY_SIGNAL.setUpdateFrequency(100);
        MASTER_MOTOR.optimizeBusUtilization();
    }

    private static void configureFollowerClimbingMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = FOLLOWER_MOTOR_INVERTED_VALUE;
        config.MotorOutput.NeutralMode = NEUTRAL_MODE_VALUE;
        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        FOLLOWER_MOTOR.getConfigurator().apply(config);

        FOLLOWER_MOTOR.optimizeBusUtilization();

        final Follower followerRequest = new Follower(MASTER_MOTOR_ID, FOLLOWER_MOTOR_OPPOSITE_DIRECTION);
        FOLLOWER_MOTOR.setControl(followerRequest);
    }
}
