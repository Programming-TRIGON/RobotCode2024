package frc.trigon.robot.components.objectcamera;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ObjectDetectionCamera extends SubsystemBase {
    private final ObjectDetectionCameraInputsAutoLogged objectDetectionCameraInputs = new ObjectDetectionCameraInputsAutoLogged();
    private final ObjectDetectionCameraIO objectDetectionCameraIO;
    private final String hostname;

    public ObjectDetectionCamera(String hostname) {
        this.hostname = hostname;
        objectDetectionCameraIO = ObjectDetectionCameraIO.generateIO(hostname);
    }

    @Override
    public void periodic() {
        objectDetectionCameraIO.updateInputs(objectDetectionCameraInputs);
        Logger.processInputs(hostname, objectDetectionCameraInputs);
    }

    public boolean hasTargets() {
        return objectDetectionCameraInputs.hasTargets;
    }

    /**
     * @return the yaw (x-axis position) of the target object
     */
    public double getObjectYaw() {
        return objectDetectionCameraInputs.bestObjectYaw;
    }
}
