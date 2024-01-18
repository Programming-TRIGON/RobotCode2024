package frc.trigon.robot.subsystems.elevator;

import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.elevator.placeholderelevator.PLACEHOLDERElevatorIO;
import frc.trigon.robot.subsystems.elevator.simulationelevator.SimulationElevatorIO;
import org.littletonrobotics.junction.AutoLog;

public class ElevatorIO {
    static ElevatorIO generateIO() {
        if (RobotConstants.IS_REPLAY)
            return new ElevatorIO();
        if (RobotConstants.ROBOT_TYPE == RobotConstants.RobotType.PLACEHOLDER)
            return new PLACEHOLDERElevatorIO();
        return new SimulationElevatorIO();
    }

    protected void updateInputs(ElevatorInputsAutoLogged inputs) {
    }

    protected void setTargetState(double targetStateMeters) {
    }

    protected void stopMotors() {
    }

    @AutoLog
    protected static class ElevatorInputs {
        public double masterMotorVoltage = 0;
        public double followerMotorVoltage = 0;

        public double masterMotorPositionMeters = 0;
        public double followerMotorPositionMeters = 0;

        public double masterMotorVelocityMetersPerSecond = 0;
        public double followerMotorVelocityMetersPerSecond = 0;
    }
}
