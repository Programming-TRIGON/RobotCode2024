package frc.trigon.robot.subsystems.elevator;

import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.elevator.simulationelevator.SimulationElevatorIO;
import frc.trigon.robot.subsystems.elevator.triumphelevator.TriumphElevatorIO;
import org.littletonrobotics.junction.AutoLog;

public class ElevatorIO {
    static ElevatorIO generateIO() {
        if (RobotConstants.IS_REPLAY)
            return new ElevatorIO();
        if (RobotConstants.ROBOT_TYPE == RobotConstants.RobotType.TRIUMPH)
            return new TriumphElevatorIO();
        if (RobotConstants.ROBOT_TYPE == RobotConstants.RobotType.SIMULATION)
            return new SimulationElevatorIO();
        return new ElevatorIO();
    }

    protected void updateInputs(ElevatorInputsAutoLogged inputs) {
    }

    protected void setTargetPosition(double targetPositionRevolutions, double speedPercentage) {
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
        public double positionRevolutions = 0;
        public double velocityRevolutionsPerSecond = 0;
        public double profiledSetpointRevolutions = 0;
        public double supplyCurrent = 0;
    }
}
