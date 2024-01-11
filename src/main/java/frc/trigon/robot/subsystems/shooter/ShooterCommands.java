package frc.trigon.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

public class ShooterCommands {
    private static final Shooter SHOOTER = Shooter.getInstance();

    public static Command getSetTargetShootingVelocityCommand(double targetVelocityRotationsPerSecond) {
        return new StartEndCommand(
                () -> SHOOTER.setTargetShootingVelocity(targetVelocityRotationsPerSecond),
                () -> {},
                SHOOTER
        );
    }

    public static Command getShootAtSpeakerCommand() {
        return new RunCommand(
                SHOOTER::shootAtSpeaker,
                SHOOTER
        );
    }

    public static Command getStopShootingCommand() {
        return new StartEndCommand(
                SHOOTER::stop,
                () -> {},
                SHOOTER
        );
    }
}
