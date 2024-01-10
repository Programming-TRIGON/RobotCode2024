package frc.trigon.robot.poseestimation.robotposesources;

import frc.trigon.robot.components.FiducialLimelight;

public class AprilTagLimelightIO extends RobotPoseSourceIO {
    private final FiducialLimelight fiducialLimelight;

    protected AprilTagLimelightIO(String hostname) {
        fiducialLimelight = new FiducialLimelight(hostname);
    }

    @Override
    protected void updateInputs(RobotPoseSourceInputsAutoLogged inputs) {
        inputs.hasResult = fiducialLimelight.hasResults();
        if (inputs.hasResult)
            inputs.cameraPose = RobotPoseSource.pose3dToDoubleArray(fiducialLimelight.getRobotPoseFromJsonDump());
        inputs.lastResultTimestamp = fiducialLimelight.getLastResultTimestamp();
    }
}
