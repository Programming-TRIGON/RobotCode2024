package frc.trigon.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.swerve.placeholderswere.PLACEHOLDERSwerveIO;
import frc.trigon.robot.subsystems.swerve.simulationswerve.SimulationSwerveIO;
import frc.trigon.robot.subsystems.swerve.trihardswerve.TrihardSwerveIO;
import org.littletonrobotics.junction.AutoLog;

public class SwerveIO {
    static SwerveIO generateIO() {
        if (RobotConstants.IS_REPLAY)
            return new SwerveIO();
        if (RobotConstants.ROBOT_TYPE == RobotConstants.RobotType.TRIHARD)
            return new TrihardSwerveIO();
        if (RobotConstants.ROBOT_TYPE == RobotConstants.RobotType.PLACEHOLDER)
            return new PLACEHOLDERSwerveIO();
        return new SimulationSwerveIO();
    }

    protected void updateInputs(SwerveInputsAutoLogged inputs) {
    }

    protected void setHeading(Rotation2d heading) {
    }

    @AutoLog
    protected static class SwerveInputs {
        public double gyroYawDegrees = 0;
        public double gyroPitchDegrees = 0;
        public double gyroRollDegrees = 0;
        public double gyroPitchVelocity = 0;
        public double gyroRollVelocity = 0;
        public double accelerationX = 0;
        public double accelerationY = 0;
        public double accelerationZ = 0;
    }
}
