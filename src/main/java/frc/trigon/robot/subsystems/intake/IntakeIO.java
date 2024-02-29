package frc.trigon.robot.subsystems.intake;

import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.intake.simulationintake.SimulationIntakeIO;
import frc.trigon.robot.subsystems.intake.triumphintake.TriumphIntakeIO;
import org.littletonrobotics.junction.AutoLog;

public class IntakeIO {
    static IntakeIO generateIO() {
        if (RobotConstants.IS_REPLAY)
            return new IntakeIO();
        if (RobotConstants.ROBOT_TYPE == RobotConstants.RobotType.TRIUMPH)
            return new TriumphIntakeIO();
        if (RobotConstants.ROBOT_TYPE == RobotConstants.RobotType.SIMULATION)
            return new SimulationIntakeIO();
        return new IntakeIO();
    }

    protected void updateInputs(IntakeInputsAutoLogged inputs) {
    }

    protected void setTargetVoltage(double voltage) {
    }

    protected void stop() {
    }

    @AutoLog
    protected static class IntakeInputs {
        public double velocityRevolutionsPerSecond = 0;
        public double voltage = 0;
        public double current = 0;
    }
}
