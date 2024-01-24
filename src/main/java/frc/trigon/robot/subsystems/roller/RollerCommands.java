package frc.trigon.robot.subsystems.roller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

public class RollerCommands {
    private static final Roller ROLLER = Roller.getInstance();

    public static Command getSetTargetStateCommand(RollerConstants.RollerState targetState) {
        return new StartEndCommand(
                () -> ROLLER.setTargetState(targetState),
                ROLLER::stop,
                ROLLER
        );
    }
}
