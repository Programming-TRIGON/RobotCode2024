package frc.trigon.robot.subsystems.roller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;

public class RollerCommands {
    private static final Roller ROLLER = RobotContainer.ROLLER;

    public static Command getSetTargetVoltageCommand(double targetVoltage) {
        return new StartEndCommand(
                () -> ROLLER.setTargetVoltage(targetVoltage),
                ROLLER::stop,
                ROLLER
        );
    }

    public static Command getSetTargetStateCommand(RollerConstants.RollerState targetState) {
        return new StartEndCommand(
                () -> ROLLER.setTargetState(targetState),
                ROLLER::stop,
                ROLLER
        );
    }
}
