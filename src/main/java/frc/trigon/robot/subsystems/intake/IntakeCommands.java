package frc.trigon.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;
import org.trigon.commands.NetworkTablesCommand;

public class IntakeCommands {
    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                IntakeCommands::getSetTargetVoltageCommand,
                false,
                "Debugging/TargetDebuggingIntakeVoltage"
        );
    }

    public static Command getSetTargetStateCommand(IntakeConstants.IntakeState targetState) {
        return getSetTargetVoltageCommand(targetState.voltage);
    }

    public static Command getSetTargetVoltageCommand(double collectorVoltage) {
        return new StartEndCommand(
                () -> RobotContainer.INTAKE.setTargetVoltage(collectorVoltage),
                RobotContainer.INTAKE::stop,
                RobotContainer.INTAKE
        );
    }

    public static Command getStopIntakeCommand() {
        return new StartEndCommand(
                RobotContainer.INTAKE::stop,
                () -> {
                },
                RobotContainer.INTAKE
        );
    }
}
