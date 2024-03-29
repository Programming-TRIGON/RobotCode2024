package frc.trigon.robot.subsystems.roller;

import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.roller.simulationroller.SimulationRollerIO;
import frc.trigon.robot.subsystems.roller.triumphroller.TriumphRollerIO;
import org.littletonrobotics.junction.AutoLog;

public class RollerIO {
    static RollerIO generateIO() {
        if (RobotConstants.IS_REPLAY)
            return new RollerIO();
        if (RobotConstants.ROBOT_TYPE == RobotConstants.RobotType.TRIUMPH)
            return new TriumphRollerIO();
        return new SimulationRollerIO();
    }

    protected void updateInputs(RollerInputsAutoLogged inputs) {
    }

    protected void setTargetVelocity(double targetVelocityRevolutionsPerSecond) {
    }

    protected void stopMotor() {
    }

    @AutoLog
    protected static class RollerInputs {
        public double motorVoltage = 0;
        public double motorCurrent = 0;
        public double motorVelocityRevolutionsPerSecond = 0;

        public boolean infraredSensorTriggered = false;
    }
}
