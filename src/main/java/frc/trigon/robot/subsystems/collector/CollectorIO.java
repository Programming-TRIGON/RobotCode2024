package frc.trigon.robot.subsystems.collector;

import edu.wpi.first.math.geometry.Rotation2d;
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

    protected void updateInputs(CollectorInputsAutoLogged inputs) {
    }

    protected void setTargetCollectionVoltage(double voltage) {
    }

    protected void setTargetAngleMotorVoltage(double voltage) {
    }

    protected void setTargetAngle(Rotation2d targetAngle) {
    }

    protected void stop() {
    }

    protected void setBrake(boolean brake) {
    }

    @AutoLog
    protected static class CollectorInputs {
        public double anglePositionDegrees = 0;
        public double angleVelocityDegreesPerSecond = 0;
        public double angleMotorProfiledSetpointDegrees = 0;
        public double angleMotorVoltage = 0;
        public double angleMotorCurrent = 0;

        public double collectionMotorVelocityRevolutionsPerSecond = 0;
        public double collectionMotorVoltage = 0;
        public double collectionMotorCurrent = 0;
    }
}
