package frc.trigon.robot.subsystems.climber;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.NetworkTablesCommand;

public class ClimberCommands {
    public static Command getClimbCommand() {
        return new FunctionalCommand(
                RobotContainer.CLIMBER::climb,
                () -> {},
                (interrupted) -> RobotContainer.CLIMBER.stop(),
                RobotContainer.CLIMBER::atTargetState
        );
    }

    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                (positionMeters, affectedByWeight) -> ClimberCommands.getSetTargetPositionCommand(positionMeters, affectedByWeight == 1),
                false,
                "Debugging/TargetDebuggingClimberPositionMeters",
                "Debugging/TargetDebuggingClimberPositionAffectedByWeight"
        );
    }
    
    public static Command getSetTargetPositionCommand(double targetPositionMeters, boolean affectedByWeight) {
        return new StartEndCommand(
                () -> RobotContainer.CLIMBER.setTargetPosition(targetPositionMeters, affectedByWeight),
                RobotContainer.CLIMBER::stop,
                RobotContainer.CLIMBER
        );
    }

    public static Command getStopCommand() {
        return new StartEndCommand(
                RobotContainer.CLIMBER::stop,
                () -> {},
                RobotContainer.CLIMBER
        );
    }

    public static Command getSetTargetStateCommand(ClimberConstants.ClimberState targetState) {
        return new StartEndCommand(
                () -> RobotContainer.CLIMBER.setTargetState(targetState),
                RobotContainer.CLIMBER::stop,
                RobotContainer.CLIMBER
        );
    }

    public static Command getSetTargetVoltageCommand(double targetVoltage) {
        return new StartEndCommand(
                () -> RobotContainer.CLIMBER.drive(Units.Volt.of(targetVoltage)),
                RobotContainer.CLIMBER::stop,
                RobotContainer.CLIMBER
        );
    }
}
