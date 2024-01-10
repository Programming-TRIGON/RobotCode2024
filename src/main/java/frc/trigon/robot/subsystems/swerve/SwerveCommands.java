package frc.trigon.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.utilities.AllianceUtilities;
import frc.trigon.robot.utilities.InitExecuteCommand;

import java.util.List;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class SwerveCommands {
    private static final Swerve SWERVE = Swerve.getInstance();

    /**
     * Creates a command that drives the swerve with the given powers, relative to the field's frame of reference, in closed loop mode.
     *
     * @param xSupplier     the target forwards power
     * @param ySupplier     the target leftwards power
     * @param thetaSupplier the target theta power, CCW+
     * @return the command
     */
    public static Command getClosedLoopFieldRelativeDriveCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier) {
        return new InitExecuteCommand(
                () -> SWERVE.initializeDrive(true),
                () -> SWERVE.fieldRelativeDrive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), thetaSupplier.getAsDouble()),
                SWERVE
        );
    }

    /**
     * Creates a command that drives the swerve with the given powers, relative to the field's frame of reference, in closed loop mode.
     * This command will use pid to reach the target angle.
     *
     * @param xSupplier     the target forwards power
     * @param ySupplier     the target leftwards power
     * @param angleSupplier the target angle supplier
     * @return the command
     */
    public static Command getClosedLoopFieldRelativeDriveCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, Supplier<Rotation2d> angleSupplier) {
        return new InitExecuteCommand(
                () -> SWERVE.initializeDrive(true),
                () -> SWERVE.fieldRelativeDrive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), angleSupplier.get()),
                SWERVE
        );
    }

    /**
     * Creates a command that drives the swerve with the given powers, relative to the field's frame of reference, in closed open mode.
     *
     * @param xSupplier     the target forwards power
     * @param ySupplier     the target leftwards power
     * @param thetaSupplier the target theta power, CCW+
     * @return the command
     */
    public static Command getOpenLoopFieldRelativeDriveCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier) {
        return new InitExecuteCommand(
                () -> SWERVE.initializeDrive(false),
                () -> SWERVE.fieldRelativeDrive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), thetaSupplier.getAsDouble()),
                SWERVE
        );
    }

    /**
     * Creates a command that drives the swerve with the given powers, relative to the field's frame of reference, in open loop mode.
     * This command will use pid to reach the target angle.
     *
     * @param xSupplier     the target forwards power
     * @param ySupplier     the target leftwards power
     * @param angleSupplier the target angle supplier
     * @return the command
     */
    public static Command getOpenLoopFieldRelativeDriveCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, Supplier<Rotation2d> angleSupplier) {
        return new InitExecuteCommand(
                () -> SWERVE.initializeDrive(false),
                () -> SWERVE.fieldRelativeDrive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), angleSupplier.get()),
                SWERVE
        );
    }

    /**
     * Creates a command that drives the swerve with the given powers, relative to the robot's frame of reference, in closed loop mode.
     *
     * @param xSupplier     the target forwards power
     * @param ySupplier     the target leftwards power
     * @param thetaSupplier the target theta power, CCW+
     * @return the command
     */
    public static Command getClosedLoopSelfRelativeDriveCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier) {
        return new InitExecuteCommand(
                () -> SWERVE.initializeDrive(true),
                () -> SWERVE.selfRelativeDrive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), thetaSupplier.getAsDouble()),
                SWERVE
        );
    }

    /**
     * Creates a command that drives the swerve with the given powers, relative to the robot's frame of reference, in open loop mode.
     *
     * @param xSupplier     the target forwards power
     * @param ySupplier     the target leftwards power
     * @param thetaSupplier the target theta power, CCW+
     * @return the command
     */
    public static Command getOpenLoopSelfRelativeDriveCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier) {
        return new InitExecuteCommand(
                () -> SWERVE.initializeDrive(false),
                () -> SWERVE.selfRelativeDrive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), thetaSupplier.getAsDouble()),
                SWERVE
        );
    }

    public static Command getDriveToPoseCommand(Supplier<AllianceUtilities.AlliancePose2d> targetPose, PathConstraints constraints) {
        return new DeferredCommand(() -> getCurrentDriveToPoseCommand(targetPose.get(), constraints), Set.of(SWERVE));
    }

    private static Command getCurrentDriveToPoseCommand(AllianceUtilities.AlliancePose2d targetPose, PathConstraints constraints) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> SWERVE.initializeDrive(true)),
                getPathfindToPoseCommand(targetPose, constraints),
                getPIDToPoseCommand(targetPose)
        );
    }

    private static Command getPathfindToPoseCommand(AllianceUtilities.AlliancePose2d targetPose, PathConstraints pathConstraints) {
        final Pose2d targetMirroredAlliancePose = targetPose.toMirroredAlliancePose();
        final Pose2d currentBluePose = RobotContainer.POSE_ESTIMATOR.getCurrentPose().toBlueAlliancePose();
        if (currentBluePose.getTranslation().getDistance(targetMirroredAlliancePose.getTranslation()) < 0.35)
            return new InstantCommand();
        return AutoBuilder.pathfindToPose(targetMirroredAlliancePose, pathConstraints);
    }

    private static Command getGenerateOnTheFlyPathCommand(AllianceUtilities.AlliancePose2d targetPose, PathConstraints pathConstraints) {
        final List<PathPoint> pathPoints = List.of(
                new PathPoint(RobotContainer.POSE_ESTIMATOR.getCurrentPose().toBlueAlliancePose().getTranslation()),
                new PathPoint(targetPose.toBlueAlliancePose().getTranslation())
        );
        final PathPlannerPath path = PathPlannerPath.fromPathPoints(
                pathPoints, pathConstraints, new GoalEndState(0, targetPose.toBlueAlliancePose().getRotation())
        );
        return AutoBuilder.followPath(path);
    }

    private static Command getPIDToPoseCommand(AllianceUtilities.AlliancePose2d targetPose) {
        return new InstantCommand(SWERVE::resetRotationController)
                .andThen(new RunCommand(() -> SWERVE.pidToPose(targetPose.toMirroredAlliancePose()))
                .until(() -> SWERVE.atPose(targetPose.toMirroredAlliancePose())));
    }
}