package frc.trigon.robot.subsystems.swerve.trihardswerve;

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
import frc.trigon.robot.poseestimation.poseestimator.PoseEstimatorConstants;
import frc.trigon.robot.subsystems.swerve.SwerveConstants;
import frc.trigon.robot.subsystems.swerve.SwerveModuleIO;

import java.util.Optional;

public class TrihardSwerveConstants extends SwerveConstants {
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

    private static final Optional<SwerveModuleIO[]> MODULES_IO = ofReplayable(() -> new SwerveModuleIO[]{
            new TrihardSwerveModuleIO(TrihardSwerveModuleConstants.FRONT_LEFT_SWERVE_MODULE_CONSTANTS, "FrontLeft"),
            new TrihardSwerveModuleIO(TrihardSwerveModuleConstants.FRONT_RIGHT_SWERVE_MODULE_CONSTANTS, "FrontRight"),
            new TrihardSwerveModuleIO(TrihardSwerveModuleConstants.REAR_LEFT_SWERVE_MODULE_CONSTANTS, "RearLeft"),
            new TrihardSwerveModuleIO(TrihardSwerveModuleConstants.REAR_RIGHT_SWERVE_MODULE_CONSTANTS, "RearRight")
    });

    private static final PIDConstants
            TRANSLATION_PID_CONSTANTS = new PIDConstants(5, 0, 0),
            PROFILED_ROTATION_PID_CONSTANTS = new PIDConstants(6, 0, 0),
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
            Units.degreesToRadians(-0.796127),
            Units.degreesToRadians(-0.95211),
            Units.degreesToRadians(90.0146)
    );
    static final Optional<Pigeon2> GYRO = ofReplayable(() -> new Pigeon2(PIGEON_ID));

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

    static StatusSignal<Double>
            YAW_SIGNAL = null,
            PITCH_SIGNAL = null,
            X_ACCELERATION_SIGNAL = null,
            Y_ACCELERATION_SIGNAL = null,
            Z_ACCELERATION_SIGNAL = null;

    static {
        if (!RobotConstants.IS_REPLAY)
            configureGyro();
    }

    private static void configureGyro() {
        final Pigeon2 gyro = GYRO.get();
        final Pigeon2Configuration config = new Pigeon2Configuration();

        config.MountPose.MountPoseRoll = Units.radiansToDegrees(GYRO_MOUNT_POSITION.getX());
        config.MountPose.MountPosePitch = Units.radiansToDegrees(GYRO_MOUNT_POSITION.getY());
        config.MountPose.MountPoseYaw = Units.radiansToDegrees(GYRO_MOUNT_POSITION.getZ());

        gyro.getConfigurator().apply(config);

        YAW_SIGNAL = gyro.getYaw();
        PITCH_SIGNAL = gyro.getPitch();
        X_ACCELERATION_SIGNAL = gyro.getAccelerationX();
        Y_ACCELERATION_SIGNAL = gyro.getAccelerationY();
        Z_ACCELERATION_SIGNAL = gyro.getAccelerationZ();
        PITCH_SIGNAL.setUpdateFrequency(100);
        YAW_SIGNAL.setUpdateFrequency(PoseEstimatorConstants.ODOMETRY_FREQUENCY_HERTZ);
        X_ACCELERATION_SIGNAL.setUpdateFrequency(50);
        Y_ACCELERATION_SIGNAL.setUpdateFrequency(50);
        Z_ACCELERATION_SIGNAL.setUpdateFrequency(50);
        gyro.optimizeBusUtilization();
    }

    @Override
    public SwerveDriveKinematics getKinematics() {
        return KINEMATICS;
    }

    @Override
    public Optional<Pigeon2> getPigeon() {
        return GYRO;
    }

    @Override
    protected Optional<SwerveModuleIO[]> getModulesIO() {
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
    protected PIDController getTranslationsController() {
        return TRANSLATION_PID_CONTROLLER;
    }
}
