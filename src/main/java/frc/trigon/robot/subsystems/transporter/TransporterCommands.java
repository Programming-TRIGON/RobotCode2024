package frc.trigon.robot.subsystems.transporter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.NetworkTablesCommand;

public class TransporterCommands {
    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                TransporterCommands::getSetTargetVoltageCommand,
                false,
                "Debugging/TargetDebuggingTransporterVoltage"
        );
    }

    public static Command getSetTargetVoltageCommand(double targetVoltage) {
        return new StartEndCommand(
                () -> RobotContainer.TRANSPORTER.setTargetVoltage(targetVoltage),
                RobotContainer.TRANSPORTER::stop,
                RobotContainer.TRANSPORTER
        );
    }

    public static Command getSetTargetStateCommand(TransporterConstants.TransporterState targetState) {
        return new StartEndCommand(
                () -> RobotContainer.TRANSPORTER.setTargetState(targetState),
                RobotContainer.TRANSPORTER::stop,
                RobotContainer.TRANSPORTER
        );
    }
}
