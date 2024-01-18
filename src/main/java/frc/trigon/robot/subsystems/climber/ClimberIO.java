package frc.trigon.robot.subsystems.climber;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.climber.placeholderclimber.PlaceholderClimberIO;
import frc.trigon.robot.subsystems.climber.simulationclimber.SimulationClimberIO;
import org.littletonrobotics.junction.AutoLog;

public class ClimberIO {
    static ClimberIO generateIO() {
        if (RobotConstants.IS_REPLAY)
            return new ClimberIO();
        if (RobotConstants.ROBOT_TYPE == RobotConstants.RobotType.PLACEHOLDER)
            return new PlaceholderClimberIO();
        return new SimulationClimberIO();
    }

    protected void updateInputs(ClimberInputsAutoLogged inputs) {
    }

    protected void setPosition(Rotation2d averagePosition, Rotation2d differentialPosition) {
    }

    protected void stop() {
    }

    @AutoLog
    protected static class ClimberInputs {
        public double rightMotorPositionDegrees = 0;
        public double rightMotorVelocityDegreesPerSecond = 0;
        public double rightMotorProfiledSetpointDegrees = 0;
        public double rightMotorVoltage = 0;
        public double rightMotorCurrent = 0;

        public double leftMotorPositionDegrees = 0;
        public double leftMotorVelocityDegreesPerSecond = 0;
        public double leftMotorProfiledSetpointDegrees = 0;
        public double leftMotorVoltage = 0;
        public double leftMotorCurrent = 0;
    }
}
