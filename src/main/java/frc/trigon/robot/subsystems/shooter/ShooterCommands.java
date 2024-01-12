package frc.trigon.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

import java.util.function.DoubleSupplier;

public class ShooterCommands {
    private static final Shooter SHOOTER = Shooter.getInstance();

    public static Command getSetTargetShootingVelocityCommand(double targetVelocityRevolutionsPerSecond) {
        return new StartEndCommand(
                () -> SHOOTER.setTargetShootingVelocity(targetVelocityRevolutionsPerSecond),
                () -> {
                },
                SHOOTER
        );
    }

    public static Command getShootAtSpeakerCommand(DoubleSupplier distanceToSpeakerSupplier) {
        return new RunCommand(
                () -> SHOOTER.shootAtSpeaker(distanceToSpeakerSupplier.getAsDouble()),
                SHOOTER
        );
    }

    public static Command getStopShootingCommand() {
        return new StartEndCommand(
                SHOOTER::stop,
                () -> {
                },
                SHOOTER
        );
    }
}
