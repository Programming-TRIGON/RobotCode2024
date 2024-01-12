package frc.trigon.robot.subsystems.pitcher;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

import java.util.function.DoubleSupplier;

public class PitcherCommands {
    private static final Pitcher PITCHER = Pitcher.getInstance();

    public static Command getSetTargetPitchCommand(Rotation2d targetPitch) {
        return new StartEndCommand(
                () -> PITCHER.setTargetPitch(targetPitch),
                () -> {
                },
                PITCHER
        );
    }

    public static Command getPitchToSpeakerCommand(DoubleSupplier distanceToSpeakerSupplier) {
        return new RunCommand(
                () -> PITCHER.pitchToSpeaker(distanceToSpeakerSupplier.getAsDouble()),
                PITCHER
        );
    }
}
