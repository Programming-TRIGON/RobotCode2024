package frc.trigon.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.NetworkTablesCommand;

public class ShooterCommands {
    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                ShooterCommands::getSetTargetShootingVelocityCommand,
                false,
                "Debugging/TargetDebuggingShootingVelocity"
        );
    }

    public static Command getSetTargetShootingVelocityCommand(double targetVelocityRevolutionsPerSecond) {
        return new StartEndCommand(
                () -> RobotContainer.SHOOTER.setTargetVelocity(targetVelocityRevolutionsPerSecond),
                RobotContainer.SHOOTER::stop,
                RobotContainer.SHOOTER
        );
    }

    public static Command getShootAtSpeakerCommand() {
        return new FunctionalCommand(
                RobotContainer.SHOOTER::initializeShootingAtSpeaker,
                RobotContainer.SHOOTER::shootAtSpeaker,
                (interrupted) -> RobotContainer.SHOOTER.stop(),
                () -> false,
                RobotContainer.SHOOTER
        );
    }

    public static Command getStopShootingCommand() {
        return new StartEndCommand(
                RobotContainer.SHOOTER::stop,
                () -> {
                },
                RobotContainer.SHOOTER
        );
    }
}
