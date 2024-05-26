package frc.trigon.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.swerve.simulationswerve.SimulationSwerveConstants;
import frc.trigon.robot.subsystems.swerve.trihardswerve.TrihardSwerveConstants;
import frc.trigon.robot.subsystems.swerve.triumphswerve.TriumphSwerveConstants;

import java.util.Optional;
import java.util.function.Supplier;

public abstract class SwerveConstants {
    static final double
            TRANSLATION_TOLERANCE_METERS = 0.05,
            ROTATION_TOLERANCE_DEGREES = 2,
            TRANSLATION_VELOCITY_TOLERANCE = 0.05,
            ROTATION_VELOCITY_TOLERANCE = 0.3;
    static final double
            DRIVE_NEUTRAL_DEADBAND = 0.2,
            ROTATION_NEUTRAL_DEADBAND = 0.2;

    protected static <T> Optional<T> ofReplayable(Supplier<T> value) {
        if (RobotConstants.IS_REPLAY)
            return Optional.empty();
        return Optional.of(value.get());
    }

    static SwerveConstants generateConstants() {
        if (RobotConstants.ROBOT_TYPE == RobotConstants.RobotType.TRIHARD)
            return new TrihardSwerveConstants();
        if (RobotConstants.ROBOT_TYPE == RobotConstants.RobotType.TRIUMPH)
            return new TriumphSwerveConstants();
        return new SimulationSwerveConstants();
    }

    public abstract double getDriveRadiusMeters();

    public abstract SwerveDriveKinematics getKinematics();

    protected abstract Optional<Pigeon2> getPigeon();

    protected abstract ProfiledPIDController getProfiledRotationController();

    protected abstract PIDController getTranslationsController();

    protected abstract Optional<SwerveModuleIO[]> getModulesIO();

    protected abstract HolonomicPathFollowerConfig getPathFollowerConfig();

    protected abstract double getMaxSpeedMetersPerSecond();

    protected abstract double getMaxRotationalSpeedRadiansPerSecond();
}
