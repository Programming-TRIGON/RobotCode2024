package frc.trigon.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.utilities.InitExecuteCommand;

import java.util.function.DoubleSupplier;

public class ShooterCommands {
    private static final Shooter SHOOTER = Shooter.getInstance();

    public static Command getSetTargetShootingVelocityCommand(DoubleSupplier targetTopVelocityRevolutionsPerSecond, DoubleSupplier targetBottomVelocityRevolutionsPerSecond) {
        return new InitExecuteCommand(
                SHOOTER::resetControllers,
                () -> SHOOTER.setTargetVelocity(targetTopVelocityRevolutionsPerSecond.getAsDouble(), targetBottomVelocityRevolutionsPerSecond.getAsDouble()),
                SHOOTER
        );
    }

    public static Command getShootAtSpeakerCommand() {
        return new InitExecuteCommand(
                SHOOTER::resetControllers,
                () -> SHOOTER.shootAtSpeaker(),
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
