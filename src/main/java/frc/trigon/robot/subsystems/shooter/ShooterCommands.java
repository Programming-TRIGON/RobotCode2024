package frc.trigon.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.commands.InitExecuteCommand;

import java.util.function.DoubleSupplier;

public class ShooterCommands {
    private static final Shooter SHOOTER = Shooter.getInstance();

    public static Command getSetTargetShootingVelocityCommand(DoubleSupplier targetVelocityRevolutionsPerSecond) {
        return new InitExecuteCommand(
                SHOOTER::resetController,
                () -> SHOOTER.setTargetVelocity(targetVelocityRevolutionsPerSecond.getAsDouble()),
                SHOOTER
        );
    }

    public static Command getShootAtSpeakerCommand() {
        return new InitExecuteCommand(
                SHOOTER::resetController,
                SHOOTER::shootAtSpeaker,
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
