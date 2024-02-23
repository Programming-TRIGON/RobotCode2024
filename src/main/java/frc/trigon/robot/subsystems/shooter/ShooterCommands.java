package frc.trigon.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;

public class ShooterCommands {
    private static final Shooter SHOOTER = RobotContainer.SHOOTER;

    public static Command getSetTargetShootingVelocityCommand(double targetVelocityRevolutionsPerSecond) {
        return new StartEndCommand(
                () -> SHOOTER.setTargetVelocity(targetVelocityRevolutionsPerSecond),
                SHOOTER::stop,
                SHOOTER
        );
    }

    public static Command getShootAtSpeakerCommand() {
        return new FunctionalCommand(
                SHOOTER::limitCurrent,
                SHOOTER::shootAtSpeaker,
                (interrupted) -> SHOOTER.stop(),
                () -> false,
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
