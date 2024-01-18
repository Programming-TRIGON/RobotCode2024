package frc.trigon.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

public class ElevatorCommands {
    private static final Elevator ELEVATOR = Elevator.getInstance();

    public static Command getSetTargetStateCommand(ElevatorConstants.ElevatorState targetState) {
        return new StartEndCommand(
                () -> ELEVATOR.setTargetState(targetState),
                ELEVATOR::stop
        );
    }
}
