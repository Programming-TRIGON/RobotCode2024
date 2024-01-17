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

    protected void setTargetTopVelocity(double targetVelocityRevolutionsPerSecond) {
    }

    protected void setTargetBottomVelocity(double targetVelocityRevolutionsPerSecond) {
    }

    protected void setTargetTopVoltage(double targetVoltage) {
    }

    protected void setTargetBottomVoltage(double targetVoltage) {
    }

    protected void stop() {
    }

    @AutoLog
    protected static class ShooterInputs {
        public double topPositionRevolutions = 0;
        public double topVelocityRevolutionsPerSecond = 0;
        public double topVoltage = 0;

        public double bottomPositionRevolutions = 0;
        public double bottomVelocityRevolutionsPerSecond = 0;
        public double bottomVoltage = 0;
    }
}
