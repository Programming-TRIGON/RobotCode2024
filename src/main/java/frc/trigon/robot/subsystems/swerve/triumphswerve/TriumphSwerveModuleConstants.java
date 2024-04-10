package frc.trigon.robot.subsystems.swerve.triumphswerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.poseestimation.poseestimator.PoseEstimatorConstants;
import frc.trigon.robot.utilities.Conversions;

public class TriumphSwerveModuleConstants {
    static final double WHEEL_DIAMETER_METERS = 0.049149 * 2;
    static final double MAX_SPEED_REVOLUTIONS_PER_SECOND = Conversions.distanceToRevolutions(TriumphSwerveConstants.MAX_SPEED_METERS_PER_SECOND, WHEEL_DIAMETER_METERS);
    static final double VOLTAGE_COMPENSATION_SATURATION = 12;
    static final boolean ENABLE_FOC = true;

    static final double
            DRIVE_GEAR_RATIO = 6.12,
            STEER_GEAR_RATIO = 12.8,
            COUPLING_RATIO = 0;

    static final double
            DRIVE_OPEN_LOOP_RAMP_RATE = 0.1,
            DRIVE_CLOSED_LOOP_RAMP_RATE = 0.1;

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
            DRIVE_MOTOR_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive,
            STEER_MOTOR_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;
    private static final SensorDirectionValue STEER_ENCODER_DIRECTION = SensorDirectionValue.CounterClockwise_Positive;
    private static final AbsoluteSensorRangeValue STEER_ENCODER_RANGE = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    private static final NeutralModeValue
            DRIVE_MOTOR_NEUTRAL_MODE_VALUE = NeutralModeValue.Brake,
            STEER_MOTOR_NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;
    private static final double
            DRIVE_SLIP_CURRENT = 100,
            STEER_CURRENT_LIMIT = 50;

    private static final double
            STEER_MOTOR_P = 75,
            STEER_MOTOR_I = 0,
            STEER_MOTOR_D = 0;
    private static final double
            DRIVE_MOTOR_P = 50,
            DRIVE_MOTOR_I = 0,
            DRIVE_MOTOR_D = 0;

    private static final double
            FRONT_LEFT_STEER_ENCODER_OFFSET = -Conversions.degreesToRevolutions(225.263672 - 360),
            FRONT_RIGHT_STEER_ENCODER_OFFSET = -Conversions.degreesToRevolutions(-256.904297 + 360),
            REAR_LEFT_STEER_ENCODER_OFFSET = -Conversions.degreesToRevolutions(108.369141),
            REAR_RIGHT_STEER_ENCODER_OFFSET = -Conversions.degreesToRevolutions(-36.035156);

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
            FRONT_LEFT_STEER_ENCODER = new CANcoder(FRONT_LEFT_DRIVE_MOTOR_ID, RobotConstants.CANIVORE_NAME),
            FRONT_RIGHT_STEER_ENCODER = new CANcoder(FRONT_RIGHT_DRIVE_MOTOR_ID, RobotConstants.CANIVORE_NAME),
            REAR_LEFT_STEER_ENCODER = new CANcoder(REAR_LEFT_DRIVE_MOTOR_ID, RobotConstants.CANIVORE_NAME),
            REAR_RIGHT_STEER_ENCODER = new CANcoder(REAR_RIGHT_DRIVE_MOTOR_ID, RobotConstants.CANIVORE_NAME);

    static final TriumphSwerveModuleConstants
            FRONT_LEFT_SWERVE_MODULE_CONSTANTS = new TriumphSwerveModuleConstants(
            FRONT_LEFT_DRIVE_MOTOR,
            FRONT_LEFT_STEER_MOTOR,
            FRONT_LEFT_STEER_ENCODER,
            FRONT_LEFT_STEER_ENCODER_OFFSET
    ),
            FRONT_RIGHT_SWERVE_MODULE_CONSTANTS = new TriumphSwerveModuleConstants(
                    FRONT_RIGHT_DRIVE_MOTOR,
                    FRONT_RIGHT_STEER_MOTOR,
                    FRONT_RIGHT_STEER_ENCODER,
                    FRONT_RIGHT_STEER_ENCODER_OFFSET
            ),
            REAR_LEFT_SWERVE_MODULE_CONSTANTS = new TriumphSwerveModuleConstants(
                    REAR_LEFT_DRIVE_MOTOR,
                    REAR_LEFT_STEER_MOTOR,
                    REAR_LEFT_STEER_ENCODER,
                    REAR_LEFT_STEER_ENCODER_OFFSET
            ),
            REAR_RIGHT_SWERVE_MODULE_CONSTANTS = new TriumphSwerveModuleConstants(
                    REAR_RIGHT_DRIVE_MOTOR,
                    REAR_RIGHT_STEER_MOTOR,
                    REAR_RIGHT_STEER_ENCODER,
                    REAR_RIGHT_STEER_ENCODER_OFFSET
            );

    final TalonFX driveMotor, steerMotor;
    final CANcoder steerEncoder;
    final double encoderOffset;
    StatusSignal<Double> steerPositionSignal, steerVelocitySignal, steerVoltageSignal, driveStatorCurrentSignal, drivePositionSignal, driveVelocitySignal, driveVoltageSignal;

    private TriumphSwerveModuleConstants(TalonFX driveMotor, TalonFX steerMotor, CANcoder steerEncoder, double encoderOffset) {
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

        int counter = 10;
        StatusCode statusCode = null;
        while (statusCode != StatusCode.OK && counter > 0) {
            statusCode = steerEncoder.getConfigurator().apply(config);
            counter--;
        }

        steerPositionSignal = steerEncoder.getPosition().clone();
        steerVelocitySignal = steerEncoder.getVelocity().clone();
        steerPositionSignal.setUpdateFrequency(PoseEstimatorConstants.ODOMETRY_FREQUENCY_HERTZ);
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
        config.ClosedLoopRamps.TorqueClosedLoopRampPeriod = DRIVE_CLOSED_LOOP_RAMP_RATE;
        config.OpenLoopRamps.VoltageOpenLoopRampPeriod = DRIVE_OPEN_LOOP_RAMP_RATE;

        config.Slot0.kP = DRIVE_MOTOR_P;
        config.Slot0.kI = DRIVE_MOTOR_I;
        config.Slot0.kD = DRIVE_MOTOR_D;

        int counter = 10;
        StatusCode statusCode = null;
        while (statusCode != StatusCode.OK && counter > 0) {
            statusCode = driveMotor.getConfigurator().apply(config);
            counter--;
        }

        drivePositionSignal = driveMotor.getPosition().clone();
        driveVelocitySignal = driveMotor.getVelocity().clone();
        driveStatorCurrentSignal = driveMotor.getStatorCurrent().clone();
        driveVoltageSignal = driveMotor.getMotorVoltage().clone();
        drivePositionSignal.setUpdateFrequency(PoseEstimatorConstants.ODOMETRY_FREQUENCY_HERTZ);
        driveVelocitySignal.setUpdateFrequency(250);
        driveStatorCurrentSignal.setUpdateFrequency(100);
        driveVoltageSignal.setUpdateFrequency(100);
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
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        config.Feedback.FeedbackRemoteSensorID = steerEncoder.getDeviceID();

        config.Slot0.kP = STEER_MOTOR_P;
        config.Slot0.kI = STEER_MOTOR_I;
        config.Slot0.kD = STEER_MOTOR_D;
        config.ClosedLoopGeneral.ContinuousWrap = true;

        int counter = 10;
        StatusCode statusCode = null;
        while (statusCode != StatusCode.OK && counter > 0) {
            statusCode = steerMotor.getConfigurator().apply(config);
            counter--;
        }

        steerVoltageSignal = steerMotor.getMotorVoltage().clone();
        steerVoltageSignal.setUpdateFrequency(20);
        steerMotor.optimizeBusUtilization();
    }
}
