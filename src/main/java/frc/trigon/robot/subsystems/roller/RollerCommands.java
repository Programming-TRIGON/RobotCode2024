package frc.trigon.robot.subsystems.roller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

public class RollerCommands {
    private static final Roller ROLLER = Roller.getInstance();

    public static Command getSetTargetStateCommand(RollerConstants.RollerState state) {
        return new StartEndCommand(
                () -> ROLLER.setTargetState(state),
                ROLLER::stop,
                ROLLER
        );
    }
}
