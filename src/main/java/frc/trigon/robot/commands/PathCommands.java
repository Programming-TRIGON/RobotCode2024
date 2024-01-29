package frc.trigon.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.trigon.robot.RobotContainer;

import java.util.List;

public class PathCommands {
    public static Command pathfindToPose(Pose2d targetPose, double targetEndVelocity, double rotationDelayMeters) {
        return AutoBuilder.pathfindToPose(targetPose, getPathConstraints(), targetEndVelocity, rotationDelayMeters);
    }

    private static PathConstraints getPathConstraints() {
        return new PathConstraints(
                3,
                3,
                4,
                4
        );
    }

    public static Command createOnTheFlyPath(Pose2d targetPose) {
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                RobotContainer.POSE_ESTIMATOR.getCurrentPose().toAlliancePose(),
                targetPose
        );

        PathPlannerPath path = new PathPlannerPath(
                bezierPoints,
                getPathConstraints(),
                new GoalEndState(0, targetPose.getRotation())
        );

        path.preventFlipping = true;

        return AutoBuilder.followPath(path);
    }
}
