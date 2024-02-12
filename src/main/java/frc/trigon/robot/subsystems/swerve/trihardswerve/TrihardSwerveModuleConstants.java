package frc.trigon.robot.subsystems.swerve.trihardswerve;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.trigon.robot.commands.Commands;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.poseestimation.poseestimator.PoseEstimatorConstants;
import frc.trigon.robot.utilities.Conversions;

public class TrihardSwerveModuleConstants {
    static final double WHEEL_DIAMETER_METERS = 0.1016;
    static final double MAX_SPEED_REVOLUTIONS_PER_SECOND = Conversions.distanceToRevolutions(TrihardSwerveConstants.MAX_SPEED_METERS_PER_SECOND, WHEEL_DIAMETER_METERS);
    static final double VOLTAGE_COMPENSATION_SATURATION = 12;

    static final double
            DRIVE_GEAR_RATIO = 10.867,
            STEER_GEAR_RATIO = 12.8,
            COUPLING_RATIO = 0.048;

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

    private static final boolean
            DRIVE_MOTOR_INVERTED = true,
            STEER_MOTOR_INVERTED = false;
    private static final CANSparkBase.IdleMode
            DRIVE_MOTOR_IDLE_MODE = CANSparkBase.IdleMode.kBrake,
            STEER_MOTOR_IDLE_MODE_VALUE = CANSparkBase.IdleMode.kBrake;
    private static final int
            DRIVE_SLIP_CURRENT = 100,
            STEER_CURRENT_LIMIT = 50;

    static final double
            DRIVE_OPEN_LOOP_RAMP_RATE = 0.1,
            DRIVE_CLOSED_LOOP_RAMP_RATE = 0.1;

    //TODO: check gains
    private static final double
            STEER_MOTOR_P = 10,
            STEER_MOTOR_I = 0,
            STEER_MOTOR_D = 0;
    private static final double
            DRIVE_MOTOR_P = 0,
            DRIVE_MOTOR_I = 0,
            DRIVE_MOTOR_D = 0;

    private static final CANSparkLowLevel.MotorType MOTOR_TYPE = CANSparkLowLevel.MotorType.kBrushless;
    private static final CANSparkMax
            FRONT_LEFT_DRIVE_MOTOR = new CANSparkMax(FRONT_LEFT_DRIVE_MOTOR_ID, MOTOR_TYPE),
            FRONT_RIGHT_DRIVE_MOTOR = new CANSparkMax(FRONT_RIGHT_DRIVE_MOTOR_ID, MOTOR_TYPE),
            REAR_LEFT_DRIVE_MOTOR = new CANSparkMax(REAR_LEFT_DRIVE_MOTOR_ID, MOTOR_TYPE),
            REAR_RIGHT_DRIVE_MOTOR = new CANSparkMax(REAR_RIGHT_DRIVE_MOTOR_ID, MOTOR_TYPE);
    private static final CANSparkMax
            FRONT_LEFT_STEER_MOTOR = new CANSparkMax(FRONT_LEFT_STEER_MOTOR_ID, MOTOR_TYPE),
            FRONT_RIGHT_STEER_MOTOR = new CANSparkMax(FRONT_RIGHT_STEER_MOTOR_ID, MOTOR_TYPE),
            REAR_LEFT_STEER_MOTOR = new CANSparkMax(REAR_LEFT_STEER_MOTOR_ID, MOTOR_TYPE),
            REAR_RIGHT_STEER_MOTOR = new CANSparkMax(REAR_RIGHT_STEER_MOTOR_ID, MOTOR_TYPE);

    private static final double ENCODER_UPDATE_TIME_SECONDS = 5;
    private static final int ENCODER_CHANNEL_OFFSET = 1;
    private static final int
            FRONT_LEFT_ENCODER_CHANNEL = FRONT_LEFT_ID + ENCODER_CHANNEL_OFFSET,
            FRONT_RIGHT_ENCODER_CHANNEL = FRONT_RIGHT_ID + ENCODER_CHANNEL_OFFSET,
            REAR_LEFT_ENCODER_CHANNEL = REAR_LEFT_ID + ENCODER_CHANNEL_OFFSET,
            REAR_RIGHT_ENCODER_CHANNEL = REAR_RIGHT_ID + ENCODER_CHANNEL_OFFSET;
    private static final double
            FRONT_LEFT_ENCODER_OFFSET = Conversions.degreesToRevolutions(0),
            FRONT_RIGHT_ENCODER_OFFSET = Conversions.degreesToRevolutions(0),
            REAR_LEFT_ENCODER_OFFSET = Conversions.degreesToRevolutions(0),
            REAR_RIGHT_ENCODER_OFFSET = Conversions.degreesToRevolutions(0);
    private static final DutyCycleEncoder
            FRONT_LEFT_ENCODER = new DutyCycleEncoder(FRONT_LEFT_ENCODER_CHANNEL),
            FRONT_RIGHT_ENCODER = new DutyCycleEncoder(FRONT_RIGHT_ENCODER_CHANNEL),
            REAR_LEFT_ENCODER = new DutyCycleEncoder(REAR_LEFT_ENCODER_CHANNEL),
            REAR_RIGHT_ENCODER = new DutyCycleEncoder(REAR_RIGHT_ENCODER_CHANNEL);

    static final TrihardSwerveModuleConstants
            FRONT_LEFT_SWERVE_MODULE_CONSTANTS = new TrihardSwerveModuleConstants(
            FRONT_LEFT_DRIVE_MOTOR,
            FRONT_LEFT_STEER_MOTOR,
            FRONT_LEFT_ENCODER,
            FRONT_LEFT_ENCODER_OFFSET
    ),
            FRONT_RIGHT_SWERVE_MODULE_CONSTANTS = new TrihardSwerveModuleConstants(
                    FRONT_RIGHT_DRIVE_MOTOR,
                    FRONT_RIGHT_STEER_MOTOR,
                    FRONT_RIGHT_ENCODER,
                    FRONT_RIGHT_ENCODER_OFFSET
            ),
            REAR_LEFT_SWERVE_MODULE_CONSTANTS = new TrihardSwerveModuleConstants(
                    REAR_LEFT_DRIVE_MOTOR,
                    REAR_LEFT_STEER_MOTOR,
                    REAR_LEFT_ENCODER,
                    REAR_LEFT_ENCODER_OFFSET
            ),
            REAR_RIGHT_SWERVE_MODULE_CONSTANTS = new TrihardSwerveModuleConstants(
                    REAR_RIGHT_DRIVE_MOTOR,
                    REAR_RIGHT_STEER_MOTOR,
                    REAR_RIGHT_ENCODER,
                    REAR_RIGHT_ENCODER_OFFSET
            );

    final CANSparkMax driveMotor, steerMotor;
    final DutyCycleEncoder steerEncoder;
    final double encoderOffset;

    private TrihardSwerveModuleConstants(CANSparkMax driveMotor, CANSparkMax steerMotor, DutyCycleEncoder steerEncoder, double encoderOffset) {
        this.driveMotor = driveMotor;
        this.steerMotor = steerMotor;
        this.steerEncoder = steerEncoder;
        this.encoderOffset = encoderOffset;

        if (!RobotConstants.IS_REPLAY) {
            configureDriveMotor();
            configureSteerMotor();
        }
    }

    private void configureDriveMotor() {
        driveMotor.restoreFactoryDefaults();

        driveMotor.setInverted(DRIVE_MOTOR_INVERTED);
        driveMotor.setIdleMode(DRIVE_MOTOR_IDLE_MODE);
        driveMotor.enableVoltageCompensation(VOLTAGE_COMPENSATION_SATURATION);
        driveMotor.setClosedLoopRampRate(DRIVE_CLOSED_LOOP_RAMP_RATE);
        driveMotor.setOpenLoopRampRate(DRIVE_OPEN_LOOP_RAMP_RATE);
        driveMotor.setSmartCurrentLimit(DRIVE_SLIP_CURRENT);

        driveMotor.getEncoder().setPositionConversionFactor(1 / DRIVE_GEAR_RATIO);
        driveMotor.getEncoder().setVelocityConversionFactor(1 / DRIVE_GEAR_RATIO);
        driveMotor.getPIDController().setP(DRIVE_MOTOR_P);
        driveMotor.getPIDController().setI(DRIVE_MOTOR_I);
        driveMotor.getPIDController().setD(DRIVE_MOTOR_D);

        driveMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 255); // Applied output
        driveMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 10); // Motor movement
        driveMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, (int) (1000 / PoseEstimatorConstants.ODOMETRY_FREQUENCY_HERTZ)); // Motor position
        driveMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 1000); // Analog sensor
        driveMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus4, 1000); // Alternate encoder
        driveMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5, 100); // Duty cycle position

        burnFlashWithDelay(driveMotor);
    }

    private void configureSteerMotor() {
        steerMotor.restoreFactoryDefaults();

        steerMotor.setInverted(STEER_MOTOR_INVERTED);
        steerMotor.setIdleMode(STEER_MOTOR_IDLE_MODE_VALUE);
        steerMotor.enableVoltageCompensation(VOLTAGE_COMPENSATION_SATURATION);
        steerMotor.setSmartCurrentLimit(STEER_CURRENT_LIMIT);

        steerMotor.getEncoder().setPositionConversionFactor(1 / STEER_GEAR_RATIO);
        steerMotor.getEncoder().setVelocityConversionFactor(1 / STEER_GEAR_RATIO);
        steerMotor.getPIDController().setP(STEER_MOTOR_P);
        steerMotor.getPIDController().setI(STEER_MOTOR_I);
        steerMotor.getPIDController().setD(STEER_MOTOR_D);
        steerMotor.getPIDController().setPositionPIDWrappingEnabled(true);
        steerMotor.getPIDController().setPositionPIDWrappingMinInput(-0.5);
        steerMotor.getPIDController().setPositionPIDWrappingMaxInput(0.5);

        steerMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 255); // Applied output
        steerMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 10); // Motor movement
        steerMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, (int) (1000 / PoseEstimatorConstants.ODOMETRY_FREQUENCY_HERTZ)); // Motor position
        steerMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 1000); // Analog sensor
        steerMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus4, 1000); // Alternate encoder
        steerMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5, 100); // Duty cycle position

        Commands.getDelayedCommand(ENCODER_UPDATE_TIME_SECONDS, this::setSteerMotorPositionToAbsolute).schedule();
        burnFlashWithDelay(steerMotor);
    }

    private void setSteerMotorPositionToAbsolute() {
        final double offsettedRevolutions = Conversions.offsetRead(steerEncoder.getAbsolutePosition(), encoderOffset);
        steerMotor.getEncoder().setPosition(offsettedRevolutions);
    }

    private void burnFlashWithDelay(CANSparkMax motor) {
        Commands.getDelayedCommand(3, motor::burnFlash).schedule();
    }
}
