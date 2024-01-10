package frc.trigon.robot.subsystems.swerve.placeholderswere;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.swerve.SwerveConstants;
import frc.trigon.robot.subsystems.swerve.SwerveModuleIO;

public class PLACEHOLDERSwerveConstants extends SwerveConstants {
    // TODO: Calibrate values
    static final double
            MAX_SPEED_METERS_PER_SECOND = 4.25,
            MAX_ROTATIONAL_SPEED_RADIANS_PER_SECOND = 12.03;

    private static final double
            MODULE_FROM_MODULE_DISTANCE = 0.55,
            MODULE_XY_DISTANCE_FROM_CENTER_OF_BASE = MODULE_FROM_MODULE_DISTANCE / 2;
    private static final Translation2d[] LOCATIONS = {
            new Translation2d(MODULE_XY_DISTANCE_FROM_CENTER_OF_BASE, MODULE_XY_DISTANCE_FROM_CENTER_OF_BASE),
            new Translation2d(MODULE_XY_DISTANCE_FROM_CENTER_OF_BASE, -MODULE_XY_DISTANCE_FROM_CENTER_OF_BASE),
            new Translation2d(-MODULE_XY_DISTANCE_FROM_CENTER_OF_BASE, MODULE_XY_DISTANCE_FROM_CENTER_OF_BASE),
            new Translation2d(-MODULE_XY_DISTANCE_FROM_CENTER_OF_BASE, -MODULE_XY_DISTANCE_FROM_CENTER_OF_BASE)
    };
    private static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(LOCATIONS);

    private static final PLACEHOLDERSwerveModuleIO[] MODULES_IO = {
            new PLACEHOLDERSwerveModuleIO(PLACEHOLDERSwerveModuleConstants.FRONT_LEFT_SWERVE_MODULE_CONSTANTS, "FrontLeft"),
            new PLACEHOLDERSwerveModuleIO(PLACEHOLDERSwerveModuleConstants.FRONT_RIGHT_SWERVE_MODULE_CONSTANTS, "FrontRight"),
            new PLACEHOLDERSwerveModuleIO(PLACEHOLDERSwerveModuleConstants.REAR_LEFT_SWERVE_MODULE_CONSTANTS, "RearLeft"),
            new PLACEHOLDERSwerveModuleIO(PLACEHOLDERSwerveModuleConstants.REAR_RIGHT_SWERVE_MODULE_CONSTANTS, "RearRight")
    };

    private static final PIDConstants
            TRANSLATION_PID_CONSTANTS = new PIDConstants(5, 0, 0),
            PROFILED_ROTATION_PID_CONSTANTS = new PIDConstants(5, 0, 0),
            AUTO_TRANSLATION_PID_CONSTANTS = new PIDConstants(5, 0, 0),
            AUTO_ROTATION_PID_CONSTANTS = new PIDConstants(3, 0, 0);
    private static final double
            MAX_ROTATION_VELOCITY = 720,
            MAX_ROTATION_ACCELERATION = 720;
    private static final TrapezoidProfile.Constraints ROTATION_CONSTRAINTS = new TrapezoidProfile.Constraints(
            MAX_ROTATION_VELOCITY,
            MAX_ROTATION_ACCELERATION
    );
    private static final ProfiledPIDController PROFILED_ROTATION_PID_CONTROLLER = new ProfiledPIDController(
            PROFILED_ROTATION_PID_CONSTANTS.kP,
            PROFILED_ROTATION_PID_CONSTANTS.kI,
            PROFILED_ROTATION_PID_CONSTANTS.kD,
            ROTATION_CONSTRAINTS
    );
    private static final PIDController TRANSLATION_PID_CONTROLLER = new PIDController(
            TRANSLATION_PID_CONSTANTS.kP,
            TRANSLATION_PID_CONSTANTS.kI,
            TRANSLATION_PID_CONSTANTS.kD
    );

    private static final int PIGEON_ID = 0;
    private static final Rotation3d GYRO_MOUNT_POSITION = new Rotation3d(
            Units.degreesToRadians(0),
            Units.degreesToRadians(0),
            Units.degreesToRadians(0)
    );
    static final Pigeon2 GYRO = new Pigeon2(PIGEON_ID, RobotConstants.CANIVORE_NAME);

    private static final double DRIVE_RADIUS_METERS = Math.hypot(
            MODULE_XY_DISTANCE_FROM_CENTER_OF_BASE, MODULE_XY_DISTANCE_FROM_CENTER_OF_BASE
    );
    private static final ReplanningConfig REPLANNING_CONFIG = new ReplanningConfig(true, true);
    private static final HolonomicPathFollowerConfig HOLONOMIC_PATH_FOLLOWER_CONFIG = new HolonomicPathFollowerConfig(
            AUTO_TRANSLATION_PID_CONSTANTS,
            AUTO_ROTATION_PID_CONSTANTS,
            MAX_SPEED_METERS_PER_SECOND,
            DRIVE_RADIUS_METERS,
            REPLANNING_CONFIG
    );

    static final StatusSignal<Double>
            YAW_SIGNAL = GYRO.getYaw().clone(),
            PITCH_SIGNAL = GYRO.getPitch().clone(),
            X_ACCELERATION_SIGNAL = GYRO.getAccelerationX().clone(),
            Y_ACCELERATION_SIGNAL = GYRO.getAccelerationY().clone(),
            Z_ACCELERATION_SIGNAL = GYRO.getAccelerationZ().clone();

    static {
        if (!RobotConstants.IS_REPLAY)
            configureGyro();
    }

    private static void configureGyro() {
        final Pigeon2Configuration gyroConfig = new Pigeon2Configuration();

        gyroConfig.MountPose.MountPoseRoll = Units.radiansToDegrees(GYRO_MOUNT_POSITION.getX());
        gyroConfig.MountPose.MountPosePitch = Units.radiansToDegrees(GYRO_MOUNT_POSITION.getY());
        gyroConfig.MountPose.MountPoseYaw = Units.radiansToDegrees(GYRO_MOUNT_POSITION.getZ());

        GYRO.getConfigurator().apply(gyroConfig);

        YAW_SIGNAL.setUpdateFrequency(200);
        PITCH_SIGNAL.setUpdateFrequency(100);
        X_ACCELERATION_SIGNAL.setUpdateFrequency(50);
        Y_ACCELERATION_SIGNAL.setUpdateFrequency(50);
        Z_ACCELERATION_SIGNAL.setUpdateFrequency(50);
        GYRO.optimizeBusUtilization();
    }

    @Override
    public SwerveDriveKinematics getKinematics() {
        return KINEMATICS;
    }

    @Override
    protected SwerveModuleIO[] getModulesIO() {
        return MODULES_IO;
    }

    @Override
    protected HolonomicPathFollowerConfig getPathFollowerConfig() {
        return HOLONOMIC_PATH_FOLLOWER_CONFIG;
    }

    @Override
    protected double getMaxSpeedMetersPerSecond() {
        return MAX_SPEED_METERS_PER_SECOND;
    }

    @Override
    protected double getMaxRotationalSpeedRadiansPerSecond() {
        return MAX_ROTATIONAL_SPEED_RADIANS_PER_SECOND;
    }

    @Override
    protected ProfiledPIDController getProfiledRotationController() {
        return PROFILED_ROTATION_PID_CONTROLLER;
    }

    @Override
    protected double getRobotSideLength() {
        return MODULE_FROM_MODULE_DISTANCE;
    }

    @Override
    protected PIDController getTranslationsController() {
        return TRANSLATION_PID_CONTROLLER;
    }
}
