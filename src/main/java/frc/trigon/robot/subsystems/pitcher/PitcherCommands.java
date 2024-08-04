package frc.trigon.robot.subsystems.pitcher;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;
import org.trigon.commands.ExecuteEndCommand;
import org.trigon.commands.NetworkTablesCommand;

public class PitcherCommands {
    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                (pitchDegrees) -> PitcherCommands.getSetTargetPitchCommand(Rotation2d.fromDegrees(pitchDegrees)),
                false,
                "Debugging/TargetDebuggingPitcherPitchDegrees"
        );
    }

    public static Command getSetTargetPitchCommand(Rotation2d targetPitch) {
        return new StartEndCommand(
                () -> RobotContainer.PITCHER.setTargetPitch(targetPitch),
                RobotContainer.PITCHER::stop,
                RobotContainer.PITCHER
        );
    }

    public static Command getPitchToShootingTargetCommand() {
        return new ExecuteEndCommand(
                RobotContainer.PITCHER::pitchToShootingTarget,
                RobotContainer.PITCHER::stop,
                RobotContainer.PITCHER
        );
    }
}
