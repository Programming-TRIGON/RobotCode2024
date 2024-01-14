package frc.trigon.robot.subsystems.roller;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.roller.placeholderroller.PLACEHOLDERRollerIO;
import frc.trigon.robot.subsystems.roller.simulationroller.SimulationRollerIO;
import org.littletonrobotics.junction.AutoLog;

public class RollerIO {
    static RollerIO generateIO() {
        if (RobotConstants.IS_REPLAY)
            return new RollerIO();
        if (RobotConstants.ROBOT_TYPE == RobotConstants.RobotType.PLACEHOLDER)
            return new PLACEHOLDERRollerIO();
        return new SimulationRollerIO();
    }

    protected void updateInputs(RollerInputsAutoLogged inputs) {
    }

    protected void setTargetVelocityState(double velocity) {
    }

    protected void stopMotor() {
    }

    @AutoLog
    protected static class RollerInputs {
        public double motorVoltage = 0;
        public double motorCurrent = 0;
        public double motorCurrentVelocity = 0;

        public boolean infraredSensorTriggered = false;
    }
}
