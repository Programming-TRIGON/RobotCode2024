package frc.trigon.robot.subsystems.swerve.simulationswerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.swerve.Swerve;
import frc.trigon.robot.subsystems.swerve.SwerveIO;
import frc.trigon.robot.subsystems.swerve.SwerveInputsAutoLogged;

public class SimulationSwerveIO extends SwerveIO {
    private double simulationRadians = 0;

    @Override
    protected void updateInputs(SwerveInputsAutoLogged inputs) {
        updateSimulation();
        inputs.gyroYawDegrees = Units.radiansToDegrees(simulationRadians);
    }

    private void updateSimulation() {
        simulationRadians += Swerve.getInstance().getSelfRelativeVelocity().omegaRadiansPerSecond * RobotConstants.PERIODIC_TIME_SECONDS;
    }

    @Override
    protected void setHeading(Rotation2d heading) {
        simulationRadians = heading.getRadians();
    }
}
