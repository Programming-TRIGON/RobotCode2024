package frc.trigon.robot.subsystems.climber;

import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.climber.simulationclimber.SimulationClimberIO;
import frc.trigon.robot.subsystems.climber.triumphclimber.TriumphClimberIO;
import org.littletonrobotics.junction.AutoLog;

public class ClimberIO {
    static ClimberIO generateIO() {
        if (RobotConstants.IS_REPLAY)
            return new ClimberIO();
        if (RobotConstants.ROBOT_TYPE == RobotConstants.RobotType.TRIUMPH)
            return new TriumphClimberIO();
        if (RobotConstants.ROBOT_TYPE == RobotConstants.RobotType.SIMULATION)
            return new SimulationClimberIO();
        return new ClimberIO();
    }

    protected void updateInputs(ClimberInputsAutoLogged inputs) {
    }

    protected void setTargetPositionMeters(double targetPositionMeters, boolean affectedByWeight) {
    }

    protected void setTargetVoltage(double targetVoltage) {
    }

    protected void stop() {
    }

    protected void setBrake(boolean brake) {
    }

    @AutoLog
    protected static class ClimberInputs {
        public double encoderPositionMeters = 0;
        public double encoderVelocityMetersPerSecond = 0;
        public double motorProfiledSetpointMeters = 0;
        public double motorVoltage = 0;
        public double motorCurrent = 0;
    }
}
