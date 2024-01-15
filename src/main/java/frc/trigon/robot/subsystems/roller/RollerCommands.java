package frc.trigon.robot.subsystems.roller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

public class RollerCommands {
    private static final Roller ROLLER = Roller.getInstance();

    public Command getSetTargetStateCommand(RollerConstants.RollerState state) {
        return new StartEndCommand(
                () -> ROLLER.setTargetVelocity(state.velocityRevolutionsPerSecond),
                ROLLER::stop,
                ROLLER
        );
    }
}
