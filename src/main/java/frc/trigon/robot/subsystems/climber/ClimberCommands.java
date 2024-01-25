package frc.trigon.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

public class ClimberCommands {
    private static final Climber CLIMBER = Climber.getInstance();

    public static Command getSetTargetStateCommand(ClimberConstants.ClimberState targetState) {
        return new StartEndCommand(
                () -> CLIMBER.setTargetState(targetState),
                CLIMBER::stop,
                CLIMBER
        );
    }
}
