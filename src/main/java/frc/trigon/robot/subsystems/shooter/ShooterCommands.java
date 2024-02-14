package frc.trigon.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.InitExecuteCommand;

public class ShooterCommands {
    private static final Shooter SHOOTER = RobotContainer.SHOOTER;

    public static Command getResetControllerCommand() {
        return new InstantCommand(SHOOTER::resetController, SHOOTER);
    }

    public static Command getSetTargetShootingVelocityCommand(double targetVelocityRevolutionsPerSecond) {
        return new InitExecuteCommand(
                SHOOTER::resetController,
                () -> SHOOTER.setTargetVelocity(targetVelocityRevolutionsPerSecond),
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
