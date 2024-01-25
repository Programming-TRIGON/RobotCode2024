package frc.trigon.robot.subsystems.swerve.simulationswerve;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.simulation.GyroSimulation;
import frc.trigon.robot.subsystems.swerve.Swerve;
import frc.trigon.robot.subsystems.swerve.SwerveIO;
import frc.trigon.robot.subsystems.swerve.SwerveInputsAutoLogged;

public class SimulationSwerveIO extends SwerveIO {
    private final GyroSimulation gyro = SimulationSwerveConstants.GYRO;

    @Override
    protected void updateInputs(SwerveInputsAutoLogged inputs) {
        gyro.update(Swerve.getInstance().getSelfRelativeVelocity().omegaRadiansPerSecond, RobotConstants.PERIODIC_TIME_SECONDS);

        inputs.gyroYawDegrees = gyro.getGyroYawDegrees();
        inputs.odometryYawsDegrees = new double[]{inputs.gyroYawDegrees};
    }

    @Override
    protected void setHeading(Rotation2d heading) {
        gyro.setHeading(heading);
    }
}
