package frc.trigon.robot.subsystems.swerve.placeholderswere;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.utilities.Conversions;

public class PLACEHOLDERSwerveModuleConstants {
    static final double WHEEL_DIAMETER_METERS = 0.1016;
    static final double MAX_SPEED_REVOLUTIONS_PER_SECOND = Conversions.distanceToRevolutions(PLACEHOLDERSwerveConstants.MAX_SPEED_METERS_PER_SECOND, WHEEL_DIAMETER_METERS);
    static final double VOLTAGE_COMPENSATION_SATURATION = 12;
    static final boolean ENABLE_FOC = true;

    static final double
            DRIVE_GEAR_RATIO = 6.75,
            STEER_GEAR_RATIO = 12.8,
            COUPLING_RATIO = 0;

    static final int
            FRONT_LEFT_ID = 0,
            FRONT_RIGHT_ID = 1,
            REAR_LEFT_ID = 2,
            REAR_RIGHT_ID = 3;
    private static final int
            FRONT_LEFT_DRIVE_MOTOR_ID = FRONT_LEFT_ID + 1,
            FRONT_RIGHT_DRIVE_MOTOR_ID = FRONT_RIGHT_ID + 1,
            REAR_LEFT_DRIVE_MOTOR_ID = REAR_LEFT_ID + 1,
            REAR_RIGHT_DRIVE_MOTOR_ID = REAR_RIGHT_ID + 1;
    private static final int
            FRONT_LEFT_STEER_MOTOR_ID = FRONT_LEFT_ID + 5,
            FRONT_RIGHT_STEER_MOTOR_ID = FRONT_RIGHT_ID + 5,
            REAR_LEFT_STEER_MOTOR_ID = REAR_LEFT_ID + 5,
            REAR_RIGHT_STEER_MOTOR_ID = REAR_RIGHT_ID + 5;

    private static final InvertedValue
            DRIVE_MOTOR_INVERTED_VALUE = InvertedValue.Clockwise_Positive,
            STEER_MOTOR_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;
    private static final SensorDirectionValue STEER_ENCODER_DIRECTION = SensorDirectionValue.Clockwise_Positive;
    private static final AbsoluteSensorRangeValue STEER_ENCODER_RANGE = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    private static final NeutralModeValue
            DRIVE_MOTOR_NEUTRAL_MODE_VALUE = NeutralModeValue.Brake,
            STEER_MOTOR_NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;
    private static final double
            DRIVE_SLIP_CURRENT = 100,
            STEER_CURRENT_LIMIT = 50;

    //TODO: check gains
    private static final double
            STEER_MOTOR_P = 75,
            STEER_MOTOR_I = 0,
            STEER_MOTOR_D = 0;
    private static final double
            DRIVE_MOTOR_P = 40,
            DRIVE_MOTOR_I = 0,
            DRIVE_MOTOR_D = 0;

    private static final double
            FRONT_LEFT_STEER_ENCODER_OFFSET = Conversions.degreesToRevolutions(311.064148),
            FRONT_RIGHT_STEER_ENCODER_OFFSET = Conversions.degreesToRevolutions(299.171448),
            REAR_LEFT_STEER_ENCODER_OFFSET = Conversions.degreesToRevolutions(504.691315),
            REAR_RIGHT_STEER_ENCODER_OFFSET = Conversions.degreesToRevolutions(-31.997681);

    private static final TalonFX
            FRONT_LEFT_DRIVE_MOTOR = new TalonFX(FRONT_LEFT_DRIVE_MOTOR_ID, RobotConstants.CANIVORE_NAME),
            FRONT_RIGHT_DRIVE_MOTOR = new TalonFX(FRONT_RIGHT_DRIVE_MOTOR_ID, RobotConstants.CANIVORE_NAME),
            REAR_LEFT_DRIVE_MOTOR = new TalonFX(REAR_LEFT_DRIVE_MOTOR_ID, RobotConstants.CANIVORE_NAME),
            REAR_RIGHT_DRIVE_MOTOR = new TalonFX(REAR_RIGHT_DRIVE_MOTOR_ID, RobotConstants.CANIVORE_NAME);
    private static final TalonFX
            FRONT_LEFT_STEER_MOTOR = new TalonFX(FRONT_LEFT_STEER_MOTOR_ID, RobotConstants.CANIVORE_NAME),
            FRONT_RIGHT_STEER_MOTOR = new TalonFX(FRONT_RIGHT_STEER_MOTOR_ID, RobotConstants.CANIVORE_NAME),
            REAR_LEFT_STEER_MOTOR = new TalonFX(REAR_LEFT_STEER_MOTOR_ID, RobotConstants.CANIVORE_NAME),
            REAR_RIGHT_STEER_MOTOR = new TalonFX(REAR_RIGHT_STEER_MOTOR_ID, RobotConstants.CANIVORE_NAME);
    private static final CANcoder
            FRONT_LEFT_STEER_ENCODER = new CANcoder(FRONT_LEFT_ID, RobotConstants.CANIVORE_NAME),
            FRONT_RIGHT_STEER_ENCODER = new CANcoder(FRONT_LEFT_ID, RobotConstants.CANIVORE_NAME),
            REAR_LEFT_STEER_ENCODER = new CANcoder(FRONT_LEFT_ID, RobotConstants.CANIVORE_NAME),
            REAR_RIGHT_STEER_ENCODER = new CANcoder(FRONT_LEFT_ID, RobotConstants.CANIVORE_NAME);

    static final PLACEHOLDERSwerveModuleConstants
            FRONT_LEFT_SWERVE_MODULE_CONSTANTS = new PLACEHOLDERSwerveModuleConstants(
            FRONT_LEFT_DRIVE_MOTOR,
            FRONT_LEFT_STEER_MOTOR,
            FRONT_LEFT_STEER_ENCODER,
            FRONT_LEFT_STEER_ENCODER_OFFSET
    ),
            FRONT_RIGHT_SWERVE_MODULE_CONSTANTS = new PLACEHOLDERSwerveModuleConstants(
                    FRONT_RIGHT_DRIVE_MOTOR,
                    FRONT_RIGHT_STEER_MOTOR,
                    FRONT_RIGHT_STEER_ENCODER,
                    FRONT_RIGHT_STEER_ENCODER_OFFSET
            ),
            REAR_LEFT_SWERVE_MODULE_CONSTANTS = new PLACEHOLDERSwerveModuleConstants(
                    REAR_LEFT_DRIVE_MOTOR,
                    REAR_LEFT_STEER_MOTOR,
                    REAR_LEFT_STEER_ENCODER,
                    REAR_LEFT_STEER_ENCODER_OFFSET
            ),
            REAR_RIGHT_SWERVE_MODULE_CONSTANTS = new PLACEHOLDERSwerveModuleConstants(
                    REAR_RIGHT_DRIVE_MOTOR,
                    REAR_RIGHT_STEER_MOTOR,
                    REAR_RIGHT_STEER_ENCODER,
                    REAR_RIGHT_STEER_ENCODER_OFFSET
            );

    final TalonFX driveMotor, steerMotor;
    final CANcoder steerEncoder;
    final double encoderOffset;
    StatusSignal<Double> steerPositionSignal, steerVelocitySignal, driveStatorCurrentSignal, drivePositionSignal, driveVelocitySignal;

    private PLACEHOLDERSwerveModuleConstants(TalonFX driveMotor, TalonFX steerMotor, CANcoder steerEncoder, double encoderOffset) {
        this.driveMotor = driveMotor;
        this.steerMotor = steerMotor;
        this.steerEncoder = steerEncoder;
        this.encoderOffset = encoderOffset;

        if (!RobotConstants.IS_REPLAY) {
            configureSteerEncoder();
            configureDriveMotor();
            configureSteerMotor();
        }
    }

    private void configureSteerEncoder() {
        final CANcoderConfiguration config = new CANcoderConfiguration();

        config.MagnetSensor.MagnetOffset = encoderOffset;
        config.MagnetSensor.SensorDirection = STEER_ENCODER_DIRECTION;
        config.MagnetSensor.AbsoluteSensorRange = STEER_ENCODER_RANGE;

        steerEncoder.getConfigurator().apply(config);

        steerPositionSignal = steerMotor.getPosition().clone();
        steerVelocitySignal = steerMotor.getVelocity().clone();
        steerPositionSignal.setUpdateFrequency(250);
        steerVelocitySignal.setUpdateFrequency(250);
        steerEncoder.optimizeBusUtilization();
    }

    private void configureDriveMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.Inverted = DRIVE_MOTOR_INVERTED_VALUE;
        config.MotorOutput.NeutralMode = DRIVE_MOTOR_NEUTRAL_MODE_VALUE;
        config.Feedback.SensorToMechanismRatio = DRIVE_GEAR_RATIO;

        config.TorqueCurrent.PeakForwardTorqueCurrent = DRIVE_SLIP_CURRENT;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -DRIVE_SLIP_CURRENT;
        config.CurrentLimits.StatorCurrentLimit = DRIVE_SLIP_CURRENT;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.Slot0.kP = DRIVE_MOTOR_P;
        config.Slot0.kI = DRIVE_MOTOR_I;
        config.Slot0.kD = DRIVE_MOTOR_D;

        driveMotor.getConfigurator().apply(config);

        drivePositionSignal = driveMotor.getPosition().clone();
        driveVelocitySignal = driveMotor.getVelocity().clone();
        driveStatorCurrentSignal = driveMotor.getStatorCurrent().clone();
        drivePositionSignal.setUpdateFrequency(250);
        driveVelocitySignal.setUpdateFrequency(250);
        driveStatorCurrentSignal.setUpdateFrequency(20);
        driveMotor.optimizeBusUtilization();
    }

    private void configureSteerMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.Inverted = STEER_MOTOR_INVERTED_VALUE;
        config.MotorOutput.NeutralMode = STEER_MOTOR_NEUTRAL_MODE_VALUE;
        config.CurrentLimits.StatorCurrentLimit = STEER_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.Feedback.RotorToSensorRatio = STEER_GEAR_RATIO;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        config.Feedback.FeedbackRemoteSensorID = steerEncoder.getDeviceID();

        config.Slot0.kP = STEER_MOTOR_P;
        config.Slot0.kI = STEER_MOTOR_I;
        config.Slot0.kD = STEER_MOTOR_D;
        config.ClosedLoopGeneral.ContinuousWrap = true;

        steerMotor.getConfigurator().apply(config);
        steerMotor.optimizeBusUtilization();
    }
}
