package frc.trigon.robot.subsystems.pitcher;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.ExecuteEndCommand;

public class PitcherCommands {
    private static final Pitcher PITCHER = RobotContainer.PITCHER;

    public static Command getSetTargetPitchCommand(Rotation2d targetPitch) {
        return new StartEndCommand(
                () -> PITCHER.setTargetPitch(targetPitch),
                PITCHER::stop,
                PITCHER
        );
    }

    public static Command getPitchToSpeakerCommand() {
        return new ExecuteEndCommand(
                PITCHER::pitchToSpeaker,
                PITCHER::stop,
                PITCHER
        );
    }
}
