package frc.trigon.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;

public class ClimberCommands {
    private static final Climber CLIMBER = RobotContainer.CLIMBER;

    public Command getSetTargetPositionCommand(double targetPositionMeters) {
        return new StartEndCommand(
                () -> CLIMBER.setTargetPosition(targetPositionMeters),
                CLIMBER::stop,
                CLIMBER
        );
    }

    public static Command getSetTargetStateCommand(ClimberConstants.ClimberState targetState) {
        return new StartEndCommand(
                () -> CLIMBER.setTargetState(targetState),
                CLIMBER::stop,
                CLIMBER
        );
    }
}
