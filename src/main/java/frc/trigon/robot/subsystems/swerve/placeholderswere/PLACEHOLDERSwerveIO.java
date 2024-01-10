package frc.trigon.robot.subsystems.swerve.placeholderswere;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.subsystems.swerve.SwerveIO;
import frc.trigon.robot.subsystems.swerve.SwerveInputsAutoLogged;

public class PLACEHOLDERSwerveIO extends SwerveIO {
    private final Pigeon2 gyro = PLACEHOLDERSwerveConstants.GYRO;

    @Override
    protected void updateInputs(SwerveInputsAutoLogged inputs) {
        inputs.gyroYawDegrees = PLACEHOLDERSwerveConstants.YAW_SIGNAL.refresh().getValue();
        inputs.gyroPitchDegrees = PLACEHOLDERSwerveConstants.PITCH_SIGNAL.refresh().getValue();
        inputs.accelerationX = PLACEHOLDERSwerveConstants.X_ACCELERATION_SIGNAL.refresh().getValue();
        inputs.accelerationY = PLACEHOLDERSwerveConstants.Y_ACCELERATION_SIGNAL.refresh().getValue();
        inputs.accelerationZ = PLACEHOLDERSwerveConstants.Z_ACCELERATION_SIGNAL.refresh().getValue();
    }

    @Override
    protected void setHeading(Rotation2d heading) {
        gyro.setYaw(heading.getDegrees());
    }
}
