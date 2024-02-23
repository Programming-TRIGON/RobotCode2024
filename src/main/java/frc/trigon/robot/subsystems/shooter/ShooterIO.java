package frc.trigon.robot.subsystems.shooter;

import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.shooter.simulationshooter.SimulationShooterIO;
import frc.trigon.robot.subsystems.shooter.triumphshooter.TriumphShooterIO;
import org.littletonrobotics.junction.AutoLog;

public class ShooterIO {
    static ShooterIO generateIO() {
        if (RobotConstants.IS_REPLAY)
            return new ShooterIO();
        if (RobotConstants.ROBOT_TYPE == RobotConstants.RobotType.TRIUMPH)
            return new TriumphShooterIO();
        if (RobotConstants.ROBOT_TYPE == RobotConstants.RobotType.SIMULATION)
            return new SimulationShooterIO();
        return new ShooterIO();
    }

    protected void updateInputs(ShooterInputsAutoLogged inputs) {
    }

    protected void setTargetVelocity(double targetVelocity) {
    }

    protected void setTargetVoltage(double targetVoltage) {
    }

    protected void enableSupplyCurrentLimit() {
    }

    protected void disableSupplyCurrentLimit() {
    }

    protected void stop() {
    }

    @AutoLog
    protected static class ShooterInputs {
        public double positionRevolutions = 0;
        public double velocityRevolutionsPerSecond = 0;
        public double voltage = 0;
        public double current = 0;
        public double acceleration = 0;
    }
}
