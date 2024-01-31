package frc.trigon.robot.subsystems.shooter;

import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.shooter.placeholdershooter.PLACEHOLDERShooterIO;
import frc.trigon.robot.subsystems.shooter.simulationshooter.SimulationShooterIO;
import org.littletonrobotics.junction.AutoLog;

public class ShooterIO {
    static ShooterIO generateIO() {
        if (RobotConstants.IS_REPLAY)
            return new ShooterIO();
        if (RobotConstants.ROBOT_TYPE == RobotConstants.RobotType.PLACEHOLDER)
            return new PLACEHOLDERShooterIO();
        return new SimulationShooterIO();
    }

    protected void updateInputs(ShooterInputsAutoLogged inputs) {
    }

    protected void setTargetVoltage(double targetVoltage) {
    }

    protected void stop() {
    }

    @AutoLog
    protected static class ShooterInputs {
        public double positionRevolutions = 0;
        public double velocityRevolutionsPerSecond = 0;
        public double voltage = 0;
        public double current = 0;
    }
}
