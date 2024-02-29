package frc.trigon.robot.subsystems.climber;

import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.climber.simulationclimber.SimulationClimberIO;
import frc.trigon.robot.subsystems.climber.triumphclimber.TriumphClimberIO;
import org.littletonrobotics.junction.AutoLog;

public class ClimberIO {
    static ClimberIO generateIO() {
        if (RobotConstants.IS_REPLAY)
            return new ClimberIO();
//        if (RobotConstants.ROBOT_TYPE == RobotConstants.RobotType.TRIUMPH)
//            return new TriumphClimberIO();
        if (RobotConstants.ROBOT_TYPE == RobotConstants.RobotType.SIMULATION)
            return new SimulationClimberIO();
        return new ClimberIO();
    }

    protected void updateInputs(ClimberInputsAutoLogged inputs) {
    }

    protected void setTargetPosition(double targetPositionRevolutions, boolean affectedByWeight) {
    }

    protected void setTargetVoltage(double targetVoltage) {
    }

    protected void stop() {
    }

    protected void setBrake(boolean brake) {
    }

    protected void resetPosition() {
    }

    @AutoLog
    protected static class ClimberInputs {
        public double positionRevolutions = 0;
        public double velocityRevolutionsPerSecond = 0;
        public double profiledSetpointRevolutions = 0;
        public double motorVoltage = 0;
        public double motorCurrent = 0;

        public boolean limitSwitchPressed = false;
    }
}
