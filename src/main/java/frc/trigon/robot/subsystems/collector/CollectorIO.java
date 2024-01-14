package frc.trigon.robot.subsystems.collector;

import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.collector.placeholdercollector.PlaceholderCollectorIO;
import frc.trigon.robot.subsystems.collector.simulationcollector.SimulationCollectorIO;
import org.littletonrobotics.junction.AutoLog;

public class CollectorIO {
    static CollectorIO generateIO() {
        if (RobotConstants.IS_REPLAY)
            return new CollectorIO();
        if (RobotConstants.ROBOT_TYPE == RobotConstants.RobotType.PLACEHOLDER)
            return new PlaceholderCollectorIO();
        return new SimulationCollectorIO();
    }

    protected void updateInputs() {
    }

    protected void setVoltage(double voltage) {
    }

    protected void stop() {
    }

    @AutoLog
    protected static class CollectorInputs {
        public double collectionMotorPositionDegrees = 0;
        public double collectionMotorVelocityDegreesPerSecond = 0;
        public double collectionMotorVoltage = 0;
        public double collectionMotorCurrent = 0;

        public double angleMotorPositionDegrees = 0;
        public double angleMotorVelocityDegreesPerSecond = 0;
        public double angleMotorVoltage = 0;
        public double angleMotorCurrent = 0;
    }
}
