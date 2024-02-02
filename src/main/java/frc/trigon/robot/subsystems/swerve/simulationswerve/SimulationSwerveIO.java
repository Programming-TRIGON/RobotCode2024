package frc.trigon.robot.subsystems.swerve.simulationswerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.simulation.GyroSimulation;
import frc.trigon.robot.subsystems.swerve.SwerveIO;
import frc.trigon.robot.subsystems.swerve.SwerveInputsAutoLogged;

public class SimulationSwerveIO extends SwerveIO {
    private final GyroSimulation gyro = SimulationSwerveConstants.GYRO;

    @Override
    protected void updateInputs(SwerveInputsAutoLogged inputs) {
        gyro.update(RobotContainer.SWERVE.getSelfRelativeVelocity().omegaRadiansPerSecond, RobotConstants.PERIODIC_TIME_SECONDS);

        inputs.gyroYawDegrees = gyro.getGyroYawDegrees();
        inputs.odometryUpdatesYawDegrees = new double[]{inputs.gyroYawDegrees};
        inputs.odometryUpdatesTimestamp = new double[]{Timer.getFPGATimestamp()};
    }

    @Override
    protected void setHeading(Rotation2d heading) {
        gyro.setHeading(heading);
    }
}
