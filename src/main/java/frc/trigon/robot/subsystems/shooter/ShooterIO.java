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

    protected void setTargetShootingVelocity(double targetVelocityRevolutionsPerSecond) {
    }

    protected void setTargetFeedingMotorVoltage(double voltage) {
    }

    protected void stop() {
    }

    @AutoLog
    protected static class ShooterInputs {
        public double shootingVelocityRevolutionsPerSecond = 0;
        public double feedingVoltage = 0;
    }
}
