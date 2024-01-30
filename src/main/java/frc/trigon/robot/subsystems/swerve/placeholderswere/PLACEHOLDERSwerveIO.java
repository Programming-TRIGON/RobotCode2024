package frc.trigon.robot.subsystems.swerve.placeholderswere;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.poseestimation.poseestimator.TalonFXOdometryThread;
import frc.trigon.robot.subsystems.swerve.SwerveIO;
import frc.trigon.robot.subsystems.swerve.SwerveInputsAutoLogged;

import java.util.Queue;

public class PLACEHOLDERSwerveIO extends SwerveIO {
    private final Pigeon2 gyro = PLACEHOLDERSwerveConstants.GYRO.get();
    private final Queue<Double> yawQueue = TalonFXOdometryThread.getInstance().registerSignal(gyro, PLACEHOLDERSwerveConstants.YAW_SIGNAL);

    @Override
    protected void updateInputs(SwerveInputsAutoLogged inputs) {
        refreshStatusSignals();

        inputs.gyroYawDegrees = PLACEHOLDERSwerveConstants.YAW_SIGNAL.getValue();
        inputs.gyroPitchDegrees = PLACEHOLDERSwerveConstants.PITCH_SIGNAL.getValue();
        inputs.accelerationX = PLACEHOLDERSwerveConstants.X_ACCELERATION_SIGNAL.getValue();
        inputs.accelerationY = PLACEHOLDERSwerveConstants.Y_ACCELERATION_SIGNAL.getValue();
        inputs.accelerationZ = PLACEHOLDERSwerveConstants.Z_ACCELERATION_SIGNAL.getValue();
        inputs.odometryYawsDegrees = yawQueue.stream().mapToDouble(Double::doubleValue).toArray();
    }

    @Override
    protected void setHeading(Rotation2d heading) {
        gyro.setYaw(heading.getDegrees());
    }

    private void refreshStatusSignals() {
        BaseStatusSignal.refreshAll(
                PLACEHOLDERSwerveConstants.YAW_SIGNAL,
                PLACEHOLDERSwerveConstants.PITCH_SIGNAL,
                PLACEHOLDERSwerveConstants.X_ACCELERATION_SIGNAL,
                PLACEHOLDERSwerveConstants.Y_ACCELERATION_SIGNAL,
                PLACEHOLDERSwerveConstants.Z_ACCELERATION_SIGNAL
        );
    }
}
