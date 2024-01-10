package frc.trigon.robot.poseestimation.poseestimator;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import frc.trigon.robot.utilities.AllianceUtilities;

public class PoseEstimatorConstants {
    /**
     * The vector represents how ambiguous each value is.
     * The first value represents how ambiguous is the x,
     * the second one for the y, and the third one is for the theta (rotation).
     * Increase these numbers to trust the estimate less.
     */
    static final Vector<N3> STATES_AMBIGUITY = VecBuilder.fill(0.003, 0.003, 0.0002);
    static final double
            TRANSLATIONS_STD_EXPONENT = 0.01,
            THETA_STD_EXPONENT = 0.01;
    static final AllianceUtilities.AlliancePose2d DEFAULT_POSE = AllianceUtilities.AlliancePose2d.fromBlueAlliancePose(new Pose2d(5, 5, new Rotation2d()));
    static final double POSE_ESTIMATOR_UPDATE_RATE = 0.02;
}

