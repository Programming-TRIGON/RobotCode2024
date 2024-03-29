package frc.trigon.robot.poseestimation.poseestimator;

import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.poseestimation.robotposesources.RobotPoseSource;
import frc.trigon.robot.poseestimation.robotposesources.RobotPoseSourceConstants;
import frc.trigon.robot.utilities.AllianceUtilities;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;

/**
 * A class that estimates the robot's pose using team 6328's custom pose estimator.
 */
public class PoseEstimator implements AutoCloseable {
    private final Field2d field = new Field2d();
    private final RobotPoseSource[] robotPoseSources;
    private final PoseEstimator6328 swerveDrivePoseEstimator = PoseEstimator6328.getInstance();
    private AllianceUtilities.AlliancePose2d robotPose = PoseEstimatorConstants.DEFAULT_POSE;

    /**
     * Constructs a new PoseEstimator.
     *
     * @param robotPoseSources the sources that should update the pose estimator apart from the odometry. This should be cameras etc.
     */
    public PoseEstimator(RobotPoseSource... robotPoseSources) {
        this.robotPoseSources = robotPoseSources;
        putAprilTagsOnFieldWidget();
        SmartDashboard.putData("Field", field);
        resetPose(PoseEstimatorConstants.DEFAULT_POSE);
        PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));
    }

    @Override
    public void close() {
        field.close();
    }

    public void periodic() {
        updatePoseEstimatorFromVision();
        robotPose = AllianceUtilities.AlliancePose2d.fromBlueAlliancePose(swerveDrivePoseEstimator.getEstimatedPose());
        Logger.recordOutput("Poses/Robot/RobotPose", robotPose.toBlueAlliancePose());
        field.setRobotPose(getCurrentPose().toBlueAlliancePose());
    }

    /**
     * Resets the pose estimator to the given pose, and the gyro to the given pose's heading.
     *
     * @param currentPose the pose to reset to, as an {@link AllianceUtilities.AlliancePose2d}
     */
    public void resetPose(AllianceUtilities.AlliancePose2d currentPose) {
        final Pose2d currentBluePose = currentPose.toBlueAlliancePose();
        RobotContainer.SWERVE.setHeading(currentBluePose.getRotation());
        swerveDrivePoseEstimator.resetPose(currentBluePose);
    }

    /**
     * @return the estimated pose of the robot, as an {@link AllianceUtilities.AlliancePose2d}
     */
    public AllianceUtilities.AlliancePose2d getCurrentPose() {
        return robotPose;
    }

    /**
     * Updates the pose estimator with the given swerve wheel positions and gyro rotations.
     * This function accepts an array of swerve wheel positions and an array of gyro rotations because the odometry can be updated at a faster rate than the main loop (which is 50 hertz).
     * This means you could have a couple of odometry updates per main loop, and you would want to update the pose estimator with all of them.
     *
     * @param swerveWheelPositions the swerve wheel positions accumulated since the last update
     * @param gyroRotations        the gyro rotations accumulated since the last update
     */
    public void updatePoseEstimatorStates(SwerveDriveWheelPositions[] swerveWheelPositions, Rotation2d[] gyroRotations, double[] timestamps) {
        for (int i = 0; i < swerveWheelPositions.length; i++)
            swerveDrivePoseEstimator.addOdometryObservation(new PoseEstimator6328.OdometryObservation(swerveWheelPositions[i], gyroRotations[i], timestamps[i]));
    }

    private void updatePoseEstimatorFromVision() {
        for (RobotPoseSource robotPoseSource : robotPoseSources) {
            final PoseEstimator6328.VisionObservation visionObservation = getVisionObservation(robotPoseSource);
            if (visionObservation != null)
                swerveDrivePoseEstimator.addVisionObservation(visionObservation);
        }
    }

    private PoseEstimator6328.VisionObservation getVisionObservation(RobotPoseSource robotPoseSource) {
        final Pose2d robotPose = robotPoseSource.getRobotPose();
        if (robotPose == null)
            return null;

        return new PoseEstimator6328.VisionObservation(
                robotPose,
                robotPoseSource.getLastResultTimestamp(),
                averageDistanceToStdDevs(robotPoseSource.getAverageDistanceFromTags(), robotPoseSource.getVisibleTags())
        );
    }

    private Matrix<N3, N1> averageDistanceToStdDevs(double averageDistance, int visibleTags) {
        final double translationStd = PoseEstimatorConstants.TRANSLATIONS_STD_EXPONENT * Math.pow(averageDistance, 2) / visibleTags;
        final double thetaStd = PoseEstimatorConstants.THETA_STD_EXPONENT * Math.pow(averageDistance, 2) / visibleTags;

        return VecBuilder.fill(translationStd, translationStd, thetaStd);
    }

    private void putAprilTagsOnFieldWidget() {
        final HashMap<Integer, Pose3d> tagsIdToPose = RobotPoseSourceConstants.TAG_ID_TO_POSE;

        for (Integer currentID : tagsIdToPose.keySet()) {
            final Pose2d tagPose = tagsIdToPose.get(currentID).toPose2d();
            field.getObject("Tag " + currentID).setPose(tagPose);
        }
    }
}
