package frc.trigon.robot.components.objectcamera;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class PhotonObjectDetectionCameraIO extends ObjectDetectionCameraIO {
    private final PhotonCamera photonCamera;

    protected PhotonObjectDetectionCameraIO(String hostname) {
        photonCamera = new PhotonCamera(hostname);
    }

    @Override
    protected void updateInputs(ObjectDetectionCameraInputsAutoLogged inputs) {
        final PhotonPipelineResult result = photonCamera.getLatestResult();

        inputs.hasTargets = result.hasTargets();
        inputs.bestObjectYaw = result.getBestTarget().getYaw();
    }
}
