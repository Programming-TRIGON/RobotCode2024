package frc.trigon.robot.subsystems.swerve.triumphswerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.poseestimation.poseestimator.TalonFXOdometryThread6328;
import frc.trigon.robot.subsystems.swerve.SwerveIO;
import frc.trigon.robot.subsystems.swerve.SwerveInputsAutoLogged;

import java.util.Queue;

public class TriumphSwerveIO extends SwerveIO {
    private final Pigeon2 gyro = TriumphSwerveConstants.GYRO.get();
    private final Queue<Double>
            yawQueue = TalonFXOdometryThread6328.getInstance().registerSignal(gyro, TriumphSwerveConstants.YAW_SIGNAL),
            timestampQueue = TalonFXOdometryThread6328.getInstance().getTimestampQueue();

    @Override
    protected void updateInputs(SwerveInputsAutoLogged inputs) {
        refreshStatusSignals();

        inputs.gyroYawDegrees = TriumphSwerveConstants.YAW_SIGNAL.getValue();
        inputs.gyroPitchDegrees = TriumphSwerveConstants.PITCH_SIGNAL.getValue();
        inputs.accelerationX = TriumphSwerveConstants.X_ACCELERATION_SIGNAL.getValue();
        inputs.accelerationY = TriumphSwerveConstants.Y_ACCELERATION_SIGNAL.getValue();
        inputs.accelerationZ = TriumphSwerveConstants.Z_ACCELERATION_SIGNAL.getValue();
        inputs.odometryUpdatesYawDegrees = yawQueue.stream().mapToDouble(Double::doubleValue).toArray();

        inputs.odometryUpdatesTimestamp = timestampQueue.stream().mapToDouble(Double::doubleValue).toArray();

        yawQueue.clear();
        timestampQueue.clear();
    }

    @Override
    protected void setHeading(Rotation2d heading) {
        gyro.setYaw(heading.getDegrees());
    }

    private void refreshStatusSignals() {
        BaseStatusSignal.refreshAll(
                TriumphSwerveConstants.YAW_SIGNAL,
                TriumphSwerveConstants.PITCH_SIGNAL,
                TriumphSwerveConstants.X_ACCELERATION_SIGNAL,
                TriumphSwerveConstants.Y_ACCELERATION_SIGNAL,
                TriumphSwerveConstants.Z_ACCELERATION_SIGNAL
        );
    }
}
