package frc.trigon.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.swerve.simulationswerve.SimulationSwerveIO;
import frc.trigon.robot.subsystems.swerve.trihardswerve.TrihardSwerveIO;
import frc.trigon.robot.subsystems.swerve.triumphswerve.TriumphSwerveIO;
import org.littletonrobotics.junction.AutoLog;

public class SwerveIO {
    static SwerveIO generateIO() {
        if (RobotConstants.IS_REPLAY)
            return new SwerveIO();
        if (RobotConstants.ROBOT_TYPE == RobotConstants.RobotType.SIMULATION)
            return new SimulationSwerveIO();
        if (RobotConstants.ROBOT_TYPE == RobotConstants.RobotType.TRIHARD)
            return new TrihardSwerveIO();
        if (RobotConstants.ROBOT_TYPE == RobotConstants.RobotType.TRIUMPH)
            return new TriumphSwerveIO();
        return new SwerveIO();
    }

    protected void updateInputs(SwerveInputsAutoLogged inputs) {
    }

    protected void setHeading(Rotation2d heading) {
    }

    @AutoLog
    protected static class SwerveInputs {
        public double[] odometryUpdatesYawDegrees = new double[0];
        public double gyroYawDegrees = 0;
        public double gyroPitchDegrees = 0;
        public double accelerationX = 0;
        public double accelerationY = 0;
        public double accelerationZ = 0;

        public double[] odometryUpdatesTimestamp = new double[0];
    }
}
