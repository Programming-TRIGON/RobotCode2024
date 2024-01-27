package frc.trigon.robot.subsystems.elevator;

import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.elevator.placeholderelevator.PLACEHOLDERElevatorIO;
import frc.trigon.robot.subsystems.elevator.simulationelevator.SimulationElevatorIO;
import org.littletonrobotics.junction.AutoLog;

public class ElevatorIO {
    static ElevatorIO generateIO() {
        if (RobotConstants.IS_REPLAY)
            return new ElevatorIO();
        if (RobotConstants.ROBOT_TYPE == RobotConstants.RobotType.PLACEHOLDER)
            return new PLACEHOLDERElevatorIO();
        return new SimulationElevatorIO();
    }

    protected void updateInputs(ElevatorInputsAutoLogged inputs) {
    }

    protected void setTargetPosition(double targetPositionMeters) {
    }

    protected void setTargetVoltage(double voltage) {
    }

    protected void stop() {
    }

    protected void setBrake(boolean brake) {
    }

    @AutoLog
    protected static class ElevatorInputs {
        public double motorVoltage = 0;
        public double positionMeters = 0;
        public double velocityMetersPerSecond = 0;
        public double profiledSetpointMeters = 0;
    }
}
