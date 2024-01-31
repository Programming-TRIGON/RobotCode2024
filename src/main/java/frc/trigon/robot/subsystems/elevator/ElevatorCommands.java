package frc.trigon.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;

public class ElevatorCommands {
    private static final Elevator ELEVATOR = RobotContainer.ELEVATOR;

    public static Command getSetTargetPositionCommand(double targetPositionMeters) {
        return new StartEndCommand(
                () -> ELEVATOR.setTargetPosition(targetPositionMeters),
                ELEVATOR::stop,
                ELEVATOR
        );
    }

    public static Command getSetTargetStateCommand(ElevatorConstants.ElevatorState targetState) {
        return new StartEndCommand(
                () -> ELEVATOR.setTargetState(targetState),
                ELEVATOR::stop,
                ELEVATOR
        );
    }

    public static Command getStayInPlaceCommand() {
        return new StartEndCommand(
                ELEVATOR::stayInPlace,
                ELEVATOR::stop,
                ELEVATOR
        );
    }
}
