package frc.trigon.robot.poseestimation.poseestimator;

import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.poseestimation.robotposesources.RobotPoseSource;
import frc.trigon.robot.poseestimation.robotposesources.RobotPoseSourceConstants;
import frc.trigon.robot.subsystems.swerve.Swerve;
import frc.trigon.robot.utilities.AllianceUtilities;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

/**
 * A class that estimates the robot's pose using team 6328's custom pose estimator.
 */
public class PoseEstimator extends SubsystemBase implements AutoCloseable {
    private final Swerve swerve = Swerve.getInstance();
    private final SwerveDriveKinematics kinematics = swerve.getConstants().getKinematics();
    private final Field2d field = new Field2d();
    private final RobotPoseSource[] robotPoseSources;
    private final PoseEstimator6328 swerveDrivePoseEstimator = new PoseEstimator6328(PoseEstimatorConstants.STATES_AMBIGUITY);
    private AllianceUtilities.AlliancePose2d robotPose = PoseEstimatorConstants.DEFAULT_POSE;
    private SwerveDriveWheelPositions previousSwerveWheelsPosition = PoseEstimatorConstants.DEFAULT_WHEEL_POSITIONS;
    private Rotation2d previousGyroRotation = new Rotation2d();

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
    }

    @Override
    public void close() {
        field.close();
    }

    @Override
    public void periodic() {
        updatePoseEstimator();
        robotPose = AllianceUtilities.AlliancePose2d.fromBlueAlliancePose(swerveDrivePoseEstimator.getLatestPose());
        Logger.recordOutput("Poses/Robot/RobotPose", robotPose.toBlueAlliancePose());
    }

    /**
     * Resets the pose estimator to the given pose, and the gyro to the given pose's heading.
     *
     * @param currentPose the pose to reset to, as an {@link AllianceUtilities.AlliancePose2d}
     */
    public void resetPose(AllianceUtilities.AlliancePose2d currentPose) {
        final Pose2d currentBluePose = currentPose.toBlueAlliancePose();
        swerve.setHeading(currentBluePose.getRotation());
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
    public void updatePoseEstimatorStates(SwerveDriveWheelPositions[] swerveWheelPositions, Rotation2d[] gyroRotations) {
        final Twist2d[] swerveTwists = toTwists(swerveWheelPositions, gyroRotations);
        final Twist2d updateTwist = getUpdateTwist(swerveTwists);
        swerveDrivePoseEstimator.addDriveData(Timer.getFPGATimestamp(), updateTwist);
    }

    private void updatePoseEstimator() {
        final List<PoseEstimator6328.TimestampedVisionUpdate> visionData = getAllVisionData();
        if (!visionData.isEmpty())
            swerveDrivePoseEstimator.addVisionData(getAllVisionData());
        field.setRobotPose(getCurrentPose().toBlueAlliancePose());
        PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));
    }

    private List<PoseEstimator6328.TimestampedVisionUpdate> getAllVisionData() {
        final List<PoseEstimator6328.TimestampedVisionUpdate> visionData = new ArrayList<>();
        for (RobotPoseSource robotPoseSource : robotPoseSources) {
            robotPoseSource.update();
            if (robotPoseSource.hasNewResult())
                visionData.add(poseSourceToCurrentVisionUpdate(robotPoseSource));
        }
        return visionData;
    }

    private PoseEstimator6328.TimestampedVisionUpdate poseSourceToCurrentVisionUpdate(RobotPoseSource robotPoseSource) {
        final Pose2d robotPose = robotPoseSource.getRobotPose();
        if (robotPose == null)
            return null;

        return new PoseEstimator6328.TimestampedVisionUpdate(
                robotPoseSource.getLastResultTimestamp(),
                robotPose,
                averageDistanceToStdDevs(robotPoseSource.getAverageDistanceFromTags(), robotPoseSource.getVisibleTags())
        );
    }

    private Matrix<N3, N1> averageDistanceToStdDevs(double averageDistance, int visibleTags) {
        final double translationStd = PoseEstimatorConstants.TRANSLATIONS_STD_EXPONENT * Math.pow(averageDistance, 2) / visibleTags;
        final double thetaStd = PoseEstimatorConstants.THETA_STD_EXPONENT * Math.pow(averageDistance, 2) / visibleTags;

        return VecBuilder.fill(translationStd, translationStd, thetaStd);
    }

    private Twist2d getUpdateTwist(Twist2d[] swerveTwists) {
        Pose2d loopPoseDifference = new Pose2d();
        for (Twist2d swerveTwist : swerveTwists)
            loopPoseDifference = loopPoseDifference.exp(swerveTwist);
        return new Pose2d().log(loopPoseDifference);
    }

    private Twist2d[] toTwists(SwerveDriveWheelPositions[] swerveWheelPositions, Rotation2d[] gyroRotations) {
        final Twist2d[] swerveTwists = new Twist2d[swerveWheelPositions.length];
        for (int i = 0; i < swerveWheelPositions.length; i++) {
            Twist2d twist = kinematics.toTwist2d(previousSwerveWheelsPosition, swerveWheelPositions[i]);
            twist = new Twist2d(twist.dx, twist.dy, gyroRotations[i].minus(previousGyroRotation).getRadians());
            previousSwerveWheelsPosition = swerveWheelPositions[i];
            previousGyroRotation = gyroRotations[i];
            swerveTwists[i] = twist;
        }
        return swerveTwists;
    }

    private void putAprilTagsOnFieldWidget() {
        final HashMap<Integer, Pose3d> tagsIdToPose = RobotPoseSourceConstants.TAG_ID_TO_POSE;

        for (Integer currentID : tagsIdToPose.keySet()) {
            final Pose2d tagPose = tagsIdToPose.get(currentID).toPose2d();
            field.getObject("Tag " + currentID).setPose(tagPose);
        }
    }
}
