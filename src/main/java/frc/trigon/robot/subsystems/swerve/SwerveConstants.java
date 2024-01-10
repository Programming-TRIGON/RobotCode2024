package frc.trigon.robot.subsystems.swerve;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.swerve.placeholderswere.PLACEHOLDERSwerveConstants;
import frc.trigon.robot.subsystems.swerve.simulationswerve.SimulationSwerveConstants;
import frc.trigon.robot.subsystems.swerve.trihardswerve.TrihardSwerveConstants;

public abstract class SwerveConstants {
    static final int MAX_SAVED_PREVIOUS_LOOP_TIMESTAMPS = 10;
    static final double
            TRANSLATION_TOLERANCE_METERS = 0.01,
            ROTATION_TOLERANCE_DEGREES = 1,
            TRANSLATION_VELOCITY_TOLERANCE = 0.05,
            ROTATION_VELOCITY_TOLERANCE = 0.05;
    static final double
            DRIVE_NEUTRAL_DEADBAND = 0.2,
            ROTATION_NEUTRAL_DEADBAND = 0.2;

    static SwerveConstants generateConstants() {
        if (RobotConstants.ROBOT_TYPE == RobotConstants.RobotType.TRIHARD)
            return new TrihardSwerveConstants();
        if (RobotConstants.ROBOT_TYPE == RobotConstants.RobotType.PLACEHOLDER)
            return new PLACEHOLDERSwerveConstants();
        return new SimulationSwerveConstants();
    }

    public abstract SwerveDriveKinematics getKinematics();

    /**
     * @return the swerve's robot side length in meters, (not including the bumpers)
     */
    protected abstract double getRobotSideLength();

    protected abstract ProfiledPIDController getProfiledRotationController();

    protected abstract PIDController getTranslationsController();

    protected abstract SwerveModuleIO[] getModulesIO();

    protected abstract HolonomicPathFollowerConfig getPathFollowerConfig();

    protected abstract double getMaxSpeedMetersPerSecond();

    protected abstract double getMaxRotationalSpeedRadiansPerSecond();
}
