package frc.trigon.robot.poseestimation.poseestimator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.trigon.robot.poseestimation.robotposesources.RobotPoseSource;
import frc.trigon.robot.poseestimation.robotposesources.RobotPoseSourceConstants;
import frc.trigon.robot.subsystems.swerve.Swerve;
import frc.trigon.robot.utilities.AllianceUtilities;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;

/**
 * A class that estimates the robot's pose using a {@link SwerveDrivePoseEstimator}, and robot pose sources.
 * This pose estimator will provide you the robot's pose relative to the current driver station.
 *
 * @author Shriqui - Captain, Omer - Programing Captain
 */
public class PoseEstimator implements AutoCloseable {
    private final Notifier periodicNotifier = new Notifier(this::periodic);
    private final Swerve swerve = Swerve.getInstance();
    private final SwerveDrivePoseEstimator swerveDrivePoseEstimator;
    private final Field2d field = new Field2d();
    private final RobotPoseSource[] robotPoseSources;
    private AllianceUtilities.AlliancePose2d robotPose = PoseEstimatorConstants.DEFAULT_POSE;

    /**
     * Constructs a new PoseEstimator.
     *
     * @param robotPoseSources the sources that should update the pose estimator apart from the odometry. This should be cameras etc.
     */
    public PoseEstimator(RobotPoseSource... robotPoseSources) {
        this.robotPoseSources = robotPoseSources;
        swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
                swerve.getConstants().getKinematics(),
                swerve.getHeading(),
                swerve.getModulePositions(),
                PoseEstimatorConstants.DEFAULT_POSE.toBlueAlliancePose(),
                PoseEstimatorConstants.STATES_AMBIGUITY,
                VecBuilder.fill(0, 0, 0)
        );

        putAprilTagsOnFieldWidget();
        periodicNotifier.startPeriodic(PoseEstimatorConstants.POSE_ESTIMATOR_UPDATE_RATE);
        SmartDashboard.putData("Field", field);
    }

    @Override
    public void close() {
        field.close();
        periodicNotifier.close();
    }

    /**
     * Resets the pose estimator to the given pose, and the gyro to the given pose's heading.
     *
     * @param currentPose the pose to reset to, as an {@link AllianceUtilities.AlliancePose2d}
     */
    public void resetPose(AllianceUtilities.AlliancePose2d currentPose) {
        final Pose2d currentBluePose = currentPose.toBlueAlliancePose();
        swerve.setHeading(currentBluePose.getRotation());

        resetPoseEstimator(currentBluePose);
    }

    /**
     * @return the estimated pose of the robot, as an {@link AllianceUtilities.AlliancePose2d}
     */
    public AllianceUtilities.AlliancePose2d getCurrentPose() {
        return robotPose;
    }

    private void periodic() {
        updatePoseEstimator();
        robotPose = AllianceUtilities.AlliancePose2d.fromBlueAlliancePose(swerveDrivePoseEstimator.getEstimatedPosition());
        Logger.recordOutput("Poses/Robot/RobotPose", robotPose.toBlueAlliancePose());
    }

    private void resetPoseEstimator(Pose2d currentPose) {
        swerveDrivePoseEstimator.resetPosition(
                currentPose.getRotation(),
                swerve.getModulePositions(),
                currentPose
        );
    }

    private void updatePoseEstimator() {
        updatePoseEstimatorStates();
        attemptToUpdateWithRobotPoseSources();
        field.setRobotPose(getCurrentPose().toBlueAlliancePose());
    }

    private void attemptToUpdateWithRobotPoseSources() {
        for (RobotPoseSource robotPoseSource : robotPoseSources) {
            robotPoseSource.update();
            if (robotPoseSource.hasNewResult())
                updateFromPoseSource(robotPoseSource);
        }
    }

    private void updateFromPoseSource(RobotPoseSource robotPoseSource) {
        final AllianceUtilities.AlliancePose2d robotPose = robotPoseSource.getRobotPose();
        if (robotPose == null)
            return;

        field.getObject(robotPoseSource.getName()).setPose(robotPose.toBlueAlliancePose());
        swerveDrivePoseEstimator.addVisionMeasurement(
                robotPose.toBlueAlliancePose(),
                robotPoseSource.getLastResultTimestamp(),
                averageDistanceToStdDevs(robotPoseSource.getAverageDistanceFromTags(), robotPoseSource.getVisibleTags())
        );
    }

    private Matrix<N3, N1> averageDistanceToStdDevs(double averageDistance, int visibleTags) {
        final double translationStd = PoseEstimatorConstants.TRANSLATIONS_STD_EXPONENT * Math.pow(averageDistance, 2) / visibleTags;
        final double thetaStd = PoseEstimatorConstants.THETA_STD_EXPONENT * Math.pow(averageDistance, 2) / visibleTags;

        return VecBuilder.fill(translationStd, translationStd, thetaStd);
    }

    private void updatePoseEstimatorStates() {
        swerveDrivePoseEstimator.update(swerve.getHeading(), swerve.getModulePositions());
    }

    private void putAprilTagsOnFieldWidget() {
        final HashMap<Integer, Pose3d> tagsIdToPose = RobotPoseSourceConstants.TAG_ID_TO_POSE;

        for (Integer currentID : tagsIdToPose.keySet()) {
            final Pose2d tagPose = tagsIdToPose.get(currentID).toPose2d();
            field.getObject("Tag " + currentID).setPose(tagPose);
        }
    }
}
