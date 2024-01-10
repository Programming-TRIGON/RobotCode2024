package frc.trigon.robot.subsystems.swerve.trihardswerve;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.subsystems.swerve.SwerveIO;
import frc.trigon.robot.subsystems.swerve.SwerveInputsAutoLogged;

public class TrihardSwerveIO extends SwerveIO {
    private final Pigeon2 gyro = TrihardSwerveConstants.GYRO;

    @Override
    protected void updateInputs(SwerveInputsAutoLogged inputs) {
        inputs.gyroYawDegrees = TrihardSwerveConstants.YAW_SIGNAL.refresh().getValue();
        inputs.gyroPitchDegrees = TrihardSwerveConstants.PITCH_SIGNAL.refresh().getValue();
        inputs.accelerationX = TrihardSwerveConstants.X_ACCELERATION_SIGNAL.refresh().getValue();
        inputs.accelerationY = TrihardSwerveConstants.Y_ACCELERATION_SIGNAL.refresh().getValue();
        inputs.accelerationZ = TrihardSwerveConstants.Z_ACCELERATION_SIGNAL.refresh().getValue();
    }

    @Override
    protected void setHeading(Rotation2d heading) {
        gyro.setYaw(heading.getDegrees());
    }
}
