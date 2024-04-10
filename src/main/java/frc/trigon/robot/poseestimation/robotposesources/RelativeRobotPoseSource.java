package frc.trigon.robot.poseestimation.robotposesources;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class RelativeRobotPoseSource extends RobotPoseSource {
    private Pose2d lastPose = new Pose2d();
    private double previousResultTimestamp = -1;

    public RelativeRobotPoseSource(RobotPoseSourceConstants.RobotPoseSourceType robotPoseSourceType, String name, Transform3d robotCenterToCamera) {
        super(robotPoseSourceType, name, robotCenterToCamera);
    }

    @Override
    public Matrix<N3, N1> calculateStdDevs() {
        return RobotPoseSourceConstants.T265_AMBIGUITY;
    }

    public double getPreviousResultTimestamp() {
        final double currentResultTimestamp = super.getLastResultTimestamp();
        if (previousResultTimestamp == -1) {
            previousResultTimestamp = currentResultTimestamp;
            return currentResultTimestamp;
        }

        previousResultTimestamp = currentResultTimestamp;
        return previousResultTimestamp;
    }

    public Transform2d getLatestTransform() {
        final Pose2d currentPose = super.getRobotPose();
        if (currentPose == null)
            return null;
        final Transform2d transform2d = new Transform2d(lastPose, currentPose);
        this.lastPose = currentPose;
        return transform2d;
    }
}
