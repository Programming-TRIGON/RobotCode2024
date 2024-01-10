package frc.trigon.robot.poseestimation.robotposesources;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import frc.trigon.robot.poseestimation.photonposeestimator.PhotonPoseEstimator;

import java.util.HashMap;
import java.util.function.BiFunction;

public class RobotPoseSourceConstants {
    public static final HashMap<Integer, Pose3d> TAG_ID_TO_POSE = new HashMap<>();
    static final PhotonPoseEstimator.PoseStrategy
            PRIMARY_POSE_STRATEGY = PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            SECONDARY_POSE_STRATEGY = PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_HEADING;
    static AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
    static final Pose2d OUT_OF_FIELD_POSE = new Pose2d(100, 100, new Rotation2d());

    static {
        for (AprilTag aprilTag : APRIL_TAG_FIELD_LAYOUT.getTags())
            TAG_ID_TO_POSE.put(aprilTag.ID, aprilTag.pose);
    }

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
