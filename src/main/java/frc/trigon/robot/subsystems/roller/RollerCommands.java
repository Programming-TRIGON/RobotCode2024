package frc.trigon.robot.subsystems.roller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.NetworkTablesCommand;

public class RollerCommands {
    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                RollerCommands::getSetTargetVoltageCommand,
                false,
                "Debugging/TargetDebuggingRollerVoltage"
        );
    }

    public static Command getSetTargetVoltageCommand(double targetVoltage) {
        return new StartEndCommand(
                () -> RobotContainer.ROLLER.setTargetVoltage(targetVoltage),
                RobotContainer.ROLLER::stop,
                RobotContainer.ROLLER
        );
    }

    public static Command getSetTargetStateCommand(RollerConstants.RollerState targetState) {
        return new StartEndCommand(
                () -> RobotContainer.ROLLER.setTargetState(targetState),
                RobotContainer.ROLLER::stop,
                RobotContainer.ROLLER
        );
    }
}
