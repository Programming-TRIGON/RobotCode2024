package frc.trigon.robot.poseestimation.robotposesources;

import edu.wpi.first.math.geometry.*;
import frc.trigon.robot.Robot;
import frc.trigon.robot.utilities.AllianceUtilities;
import org.littletonrobotics.junction.Logger;

/**
 * A pose source is a class that provides the robot's pose, from a camera.
 */
public class RobotPoseSource {
    protected final String name;
    private final RobotPoseSourceInputsAutoLogged inputs = new RobotPoseSourceInputsAutoLogged();
    private final Transform3d robotCenterToCamera;
    private final RobotPoseSourceIO robotPoseSourceIO;
    private double lastUpdatedTimestamp;
    private AllianceUtilities.AlliancePose2d cachedPose = null;

    public RobotPoseSource(RobotPoseSourceConstants.RobotPoseSourceType robotPoseSourceType, String name, Transform3d robotCenterToCamera) {
        this.name = name;
        if (robotPoseSourceType != RobotPoseSourceConstants.RobotPoseSourceType.PHOTON_CAMERA)
            this.robotCenterToCamera = robotCenterToCamera;
        else
            this.robotCenterToCamera = new Transform3d();

        if (Robot.IS_REAL)
            robotPoseSourceIO = robotPoseSourceType.createIOFunction.apply(name, robotCenterToCamera);
        else
            robotPoseSourceIO = new RobotPoseSourceIO();
    }

    public static double[] pose3dToDoubleArray(Pose3d pose) {
        if (pose == null)
            return new double[0];

        return new double[]{
                pose.getTranslation().getX(),
                pose.getTranslation().getY(),
                pose.getTranslation().getZ(),
                pose.getRotation().getX(),
                pose.getRotation().getY(),
                pose.getRotation().getZ()
        };
    }

    public void update() {
        robotPoseSourceIO.updateInputs(inputs);
        Logger.processInputs(name, inputs);
        cachedPose = getUnCachedRobotPose();
        if (!inputs.hasResult || cachedPose == null)
            Logger.recordOutput("Poses/Robot/" + name + "Pose", RobotPoseSourceConstants.OUT_OF_FIELD_POSE);
        else
            Logger.recordOutput("Poses/Robot/" + name + "Pose", cachedPose.toBlueAlliancePose());
    }

    public int getVisibleTags() {
        return inputs.visibleTags;
    }

    public double getAverageDistanceFromTags() {
        return inputs.averageDistanceFromTags;
    }

    public boolean hasNewResult() {
        return isNewTimestamp() && inputs.hasResult;
    }

    public AllianceUtilities.AlliancePose2d getRobotPose() {
        return cachedPose;
    }

    public String getName() {
        return name;
    }

    public double getLastResultTimestamp() {
        return inputs.lastResultTimestamp;
    }

    private AllianceUtilities.AlliancePose2d getUnCachedRobotPose() {
        final Pose3d cameraPose = doubleArrayToPose3d(inputs.cameraPose);
        if (cameraPose == null)
            return null;

        return AllianceUtilities.AlliancePose2d.fromBlueAlliancePose(cameraPose.transformBy(robotCenterToCamera.inverse()).toPose2d());
    }

    private boolean isNewTimestamp() {
        if (lastUpdatedTimestamp == getLastResultTimestamp())
            return false;

        lastUpdatedTimestamp = getLastResultTimestamp();
        return true;
    }

    private Pose3d doubleArrayToPose3d(double[] doubleArray) {
        if (doubleArray == null || doubleArray.length != 6)
            return null;

        return new Pose3d(
                new Translation3d(doubleArray[0], doubleArray[1], doubleArray[2]),
                new Rotation3d(doubleArray[3], doubleArray[4], doubleArray[5])
        );
    }
}
