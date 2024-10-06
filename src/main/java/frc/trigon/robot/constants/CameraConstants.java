package frc.trigon.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.trigon.robot.misc.objectdetectioncamera.ObjectDetectionCamera;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCamera;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCameraConstants;

public class CameraConstants {
    public static final ObjectDetectionCamera NOTE_DETECTION_CAMERA = new ObjectDetectionCamera("Collection Camera");
    private static final Transform3d
            REAR_LEFT_CENTER_TO_CAMERA = new Transform3d(
            new Translation3d(-0.353, 0.298, 0.282),
            new Rotation3d(Units.degreesToRadians(180), Units.degreesToRadians(-20.5), Units.degreesToRadians(135))
    ),
            REAR_RIGHT_CENTER_TO_CAMERA = new Transform3d(
                    new Translation3d(-0.353, -0.298, 0.282),
                    new Rotation3d(Units.degreesToRadians(180), Units.degreesToRadians(-19), Units.degreesToRadians(225))
            ),
            FRONT_MIDDLE_CENTER_TO_CAMERA = new Transform3d(
                    new Translation3d(0.073, 0, 0.605),
                    new Rotation3d(0, Units.degreesToRadians(-31.7), 0)
            ),
            REAR_MIDDLE_CENTER_TO_CAMERA = new Transform3d(
                    new Translation3d(0, 0, 0.63),
                    new Rotation3d(0, Units.degreesToRadians(-23.7), Units.degreesToRadians(180))
            );
    public static final AprilTagCamera
//            REAR_LEFT_CAMERA = new AprilTagCamera(
//            AprilTagCameraConstants.RobotPoseSourceType.PHOTON_CAMERA,
//            "Rear Left Camera",
//            REAR_LEFT_CENTER_TO_CAMERA
//    ),
//            REAR_RIGHT_CAMERA = new AprilTagCamera(
//                    AprilTagCameraConstants.RobotPoseSourceType.PHOTON_CAMERA,
//                    "Rear Right Camera",
//                    REAR_RIGHT_CENTER_TO_CAMERA
//            ),
    //            FRONT_MIDDLE_CAMERA = new RobotPoseSource(
//                    RobotPoseSourceConstants.RobotPoseSourceType.PHOTON_CAMERA,
//                    "Front Middle Camera",
//                    FRONT_MIDDLE_CENTER_TO_CAMERA
//            ),
            REAR_MIDDLE_CAMERA = new AprilTagCamera(
            AprilTagCameraConstants.AprilTagCameraType.PHOTON_CAMERA,
            "Rear Middle Camera",
            REAR_MIDDLE_CENTER_TO_CAMERA,
            0.0004,
            0.001
    );
}
