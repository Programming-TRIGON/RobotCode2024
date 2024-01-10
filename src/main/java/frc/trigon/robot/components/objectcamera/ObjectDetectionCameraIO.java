package frc.trigon.robot.components.objectcamera;

import frc.trigon.robot.Robot;
import org.littletonrobotics.junction.AutoLog;

public class ObjectDetectionCameraIO {
    protected ObjectDetectionCameraIO() {
    }

    static ObjectDetectionCameraIO generateIO(String hostname) {
        if (!Robot.IS_REAL)
            return new ObjectDetectionCameraIO();
        return new PhotonObjectDetectionCameraIO(hostname);
    }

    @AutoLog
    public static class ObjectDetectionCameraInputs {
        public boolean hasTargets = false;
        public double bestObjectYaw = 0;
    }

    protected void updateInputs(ObjectDetectionCameraInputsAutoLogged inputs) {
    }
}
