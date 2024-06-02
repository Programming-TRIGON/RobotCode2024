package frc.trigon.robot.poseestimation.robotposesources;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.trigon.robot.poseestimation.photonposeestimator.PhotonPoseEstimator;

import java.util.function.BiFunction;

public class RobotPoseSourceConstants {
    static final PhotonPoseEstimator.PoseStrategy
            PRIMARY_POSE_STRATEGY = PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            SECONDARY_POSE_STRATEGY = PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_HEADING;
    static final double MAXIMUM_AMBIGUITY = 0.2;
    static final Pose2d[] EMPTY_POSE_LIST = new Pose2d[0];

    public enum RobotPoseSourceType {
        PHOTON_CAMERA(AprilTagPhotonCameraIO::new),
        LIMELIGHT((name, transform3d) -> new AprilTagLimelightIO(name)),
        T265((name, transform3d) -> new T265IO(name));

        final BiFunction<String, Transform3d, RobotPoseSourceIO> createIOFunction;

        RobotPoseSourceType(BiFunction<String, Transform3d, RobotPoseSourceIO> createIOFunction) {
            this.createIOFunction = createIOFunction;
        }
    }
}
