package frc.trigon.robot.subsystems.pitcher;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.pitcher.simulationpitcher.SimulationPitcherIO;
import frc.trigon.robot.subsystems.pitcher.triumphpitcher.TriumphPitcherIO;
import org.littletonrobotics.junction.AutoLog;

public class PitcherIO {
    static PitcherIO generateIO() {
        if (RobotConstants.IS_REPLAY)
            return new PitcherIO();
        if (RobotConstants.ROBOT_TYPE == RobotConstants.RobotType.TRIUMPH)
            return new TriumphPitcherIO();
        if (RobotConstants.ROBOT_TYPE == RobotConstants.RobotType.SIMULATION)
            return new SimulationPitcherIO();
        return new PitcherIO();
    }

    protected void updateInputs(PitcherInputsAutoLogged inputs) {
    }

    protected void setTargetPitch(Rotation2d pitch) {
    }

    protected void setTargetVoltage(double voltage) {
    }

    protected void setBrake(boolean brake) {
    }

    protected void stop() {
    }

    @AutoLog
    protected static class PitcherInputs {
        public double voltage = 0;
        public double pitchDegrees = 0;
        public double velocityDegreesPerSecond = 0;
        public double profiledSetpointDegrees = 0;
    }
}
