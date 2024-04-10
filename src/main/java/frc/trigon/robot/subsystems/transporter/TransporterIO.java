package frc.trigon.robot.subsystems.transporter;

import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.transporter.simulationtransporter.SimulationTransporterIO;
import frc.trigon.robot.subsystems.transporter.triumphtransporter.TriumphTransporterIO;
import org.littletonrobotics.junction.AutoLog;

public class TransporterIO {
    static TransporterIO generateIO() {
        if (RobotConstants.IS_REPLAY)
            return new TransporterIO();
        if (RobotConstants.ROBOT_TYPE == RobotConstants.RobotType.TRIUMPH)
            return new TriumphTransporterIO();
        if (RobotConstants.ROBOT_TYPE == RobotConstants.RobotType.SIMULATION)
            return new SimulationTransporterIO();
        return new TransporterIO();
    }

    protected void updateInputs(TransporterInputsAutoLogged inputs) {
    }

    protected void setTargetVoltage(double targetVoltage) {
    }

    protected void stopMotor() {
    }

    @AutoLog
    protected static class TransporterInputs {
        public double motorVoltage = 0;
        public double motorCurrent = 0;
        public double motorVelocityRevolutionsPerSecond = 0;
        public boolean sensorTriggered = false;
    }
}
