package frc.trigon.robot.subsystems.swerve.placeholderswere;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.poseestimation.poseestimator.TalonFXOdometryThread;
import frc.trigon.robot.subsystems.swerve.SwerveIO;
import frc.trigon.robot.subsystems.swerve.SwerveInputsAutoLogged;

import java.util.Queue;

public class PLACEHOLDERSwerveIO extends SwerveIO {
    private final Pigeon2 gyro = PLACEHOLDERSwerveConstants.GYRO;
    private final Queue<Double> yawQueue = TalonFXOdometryThread.getInstance().registerSignal(gyro, PLACEHOLDERSwerveConstants.YAW_SIGNAL);

    @Override
    protected void updateInputs(SwerveInputsAutoLogged inputs) {
        inputs.gyroYawDegrees = PLACEHOLDERSwerveConstants.YAW_SIGNAL.refresh().getValue();
        inputs.gyroPitchDegrees = PLACEHOLDERSwerveConstants.PITCH_SIGNAL.refresh().getValue();
        inputs.accelerationX = PLACEHOLDERSwerveConstants.X_ACCELERATION_SIGNAL.refresh().getValue();
        inputs.accelerationY = PLACEHOLDERSwerveConstants.Y_ACCELERATION_SIGNAL.refresh().getValue();
        inputs.accelerationZ = PLACEHOLDERSwerveConstants.Z_ACCELERATION_SIGNAL.refresh().getValue();
        inputs.odometryYawsDegrees = yawQueue.stream().mapToDouble(Double::doubleValue).toArray();
    }

    @Override
    protected void setHeading(Rotation2d heading) {
        gyro.setYaw(heading.getDegrees());
    }
}
