package frc.trigon.robot.components.objectdetectioncamera;

import frc.trigon.robot.constants.RobotConstants;
import org.littletonrobotics.junction.AutoLog;

public class ObjectDetectionCameraIO {
    protected ObjectDetectionCameraIO() {
    }

    static ObjectDetectionCameraIO generateIO(String hostname) {
        if (RobotConstants.IS_REPLAY)
            return new ObjectDetectionCameraIO();
        if (RobotConstants.ROBOT_TYPE == RobotConstants.RobotType.SIMULATION)
            return new SimulationObjectDetectionCameraIO(hostname);
        return new PhotonObjectDetectionCameraIO(hostname);
    }

    protected void updateInputs(ObjectDetectionCameraInputsAutoLogged inputs) {
    }

    @AutoLog
    public static class ObjectDetectionCameraInputs {
        public boolean hasTargets = false;
        public double bestObjectYaw = 0;
    }
}
