package frc.trigon.robot.subsystems.climber;

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

    protected void setTargetPositionMeters(double averagePositionMeters) {
    }

    protected void stop() {
    }

    @AutoLog
    protected static class ClimberInputs {
        public double rightMotorPositionMeters = 0;
        public double rightMotorVelocityMetersPerSecond = 0;
        public double rightMotorProfiledSetpointMeters = 0;
        public double rightMotorVoltage = 0;
        public double rightMotorCurrent = 0;

        public double leftMotorPositionMeters = 0;
        public double leftMotorVelocityMetersPerSecond = 0;
        public double leftMotorProfiledSetpointMeters = 0;
        public double leftMotorVoltage = 0;
        public double leftMotorCurrent = 0;
    }
}
