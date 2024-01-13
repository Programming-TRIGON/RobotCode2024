package frc.trigon.robot.components.objectdetectioncamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.trigon.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;

public class SimulationObjectDetectionCameraIO extends ObjectDetectionCameraIO {
    private static final Rotation2d HORIZONTAL_FOV = Rotation2d.fromDegrees(75);
    private static final double
            MAXIMUM_DISTANCE_METERS = 5,
            MINIMUM_DISTANCE_METERS = 0.05;
    private static final Translation2d[] OBJECT_PLACEMENTS = {
            new Translation2d(5, 5)
    };

    private final String hostname;

    protected SimulationObjectDetectionCameraIO(String hostname) {
        this.hostname = hostname;
    }

    @Override
    protected void updateInputs(ObjectDetectionCameraInputsAutoLogged inputs) {
        final Rotation2d closestObjectYaw = getClosestVisibleObjectYaw(RobotContainer.POSE_ESTIMATOR.getCurrentPose().toBlueAlliancePose());
        if (closestObjectYaw == null) {
            inputs.hasTargets = false;
        } else {
            inputs.hasTargets = true;
            inputs.bestObjectYaw = closestObjectYaw.getDegrees();
        }
    }

    private Rotation2d getClosestVisibleObjectYaw(Pose2d robotPose) {
        Translation2d closestObject = null;
        Rotation2d closestObjectYaw = null;
        double closestDistance = Double.POSITIVE_INFINITY;

        for (Translation2d objectPlacement : OBJECT_PLACEMENTS) {
            final Rotation2d angleToObject = getAngleToObject(objectPlacement, robotPose);
            if (!isWithinHorizontalFOV(angleToObject, robotPose) || !isWithinDistance(objectPlacement, robotPose))
                continue;

            final double distance = getObjectDistance(objectPlacement, robotPose);
            if (distance < closestDistance) {
                closestObject = objectPlacement;
                closestObjectYaw = angleToObject.minus(robotPose.getRotation());
                closestDistance = distance;
            }
        }

        logObjectPlacement(closestObject, robotPose);
        return closestObjectYaw;
    }

    private void logObjectPlacement(Translation2d objectPlacement, Pose2d robotPose) {
        if (objectPlacement != null)
            Logger.recordOutput(hostname + "/ClosestObject", objectPlacement);
        else
            Logger.recordOutput(hostname + "/ClosestObject", robotPose.getTranslation());
    }

    private boolean isWithinHorizontalFOV(Rotation2d objectYaw, Pose2d robotPose) {
        return Math.abs(objectYaw.minus(robotPose.getRotation()).getRadians()) <= HORIZONTAL_FOV.getRadians() / 2;
    }

    private boolean isWithinDistance(Translation2d objectPlacement, Pose2d robotPose) {
        final double distance = getObjectDistance(objectPlacement, robotPose);
        return distance <= MAXIMUM_DISTANCE_METERS && distance >= MINIMUM_DISTANCE_METERS;
    }

    private double getObjectDistance(Translation2d objectPlacement, Pose2d robotPose) {
        final Translation2d difference = objectPlacement.minus(robotPose.getTranslation());
        return difference.getNorm();
    }

    private Rotation2d getAngleToObject(Translation2d objectPlacement, Pose2d robotPose) {
        final Translation2d difference = objectPlacement.minus(robotPose.getTranslation());
        return Rotation2d.fromRadians(Math.atan2(difference.getY(), difference.getX()));
    }

}
