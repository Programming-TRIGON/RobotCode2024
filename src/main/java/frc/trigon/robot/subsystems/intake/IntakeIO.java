package frc.trigon.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
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
        return new SimulationIntakeIO();
    }

    protected void updateInputs(IntakeInputsAutoLogged inputs) {
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
    protected static class IntakeInputs {
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
