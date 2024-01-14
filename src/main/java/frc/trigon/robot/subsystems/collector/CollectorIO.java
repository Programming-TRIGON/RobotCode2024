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

    protected void setCollectionVoltage(double voltage) {
    }

    protected void setAngleMotorVoltage(double voltage) {
    }

    protected void setTargetAngle(Rotation2d targetAngle) {
    }

    protected void stopCollectionMotor() {
    }

    protected void stopAngleMotor() {
    }

    protected void setBrake(boolean brake) {
    }

    @AutoLog

    protected static class CollectorInputs {
        public double angleMotorPositionDegrees = 0;
        public double angleMotorVelocityDegreesPerSecond = 0;
        public double angleMotorVoltage = 0;
        public double angleMotorCurrent = 0;

        public double collectionMotorVoltage = 0;
        public double collectionMotorCurrent = 0;
    }
}
