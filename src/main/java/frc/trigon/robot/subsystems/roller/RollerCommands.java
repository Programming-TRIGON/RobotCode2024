package frc.trigon.robot.subsystems.roller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

public class RollerCommands {
    private static final Roller ROLLER = Roller.getInstance();

    public Command getSetTargetVelocityCommand(double velocity) {
        return new StartEndCommand(
                () -> ROLLER.setTargetVelocity(velocity),
                ROLLER::stop,
                ROLLER
        );
    }

    public Command getSetTargetStateCommand(RollerConstants.RollerState state) {
        return getSetTargetVelocityCommand(state.velocityRevolutionsPerSecond);
    }
}
