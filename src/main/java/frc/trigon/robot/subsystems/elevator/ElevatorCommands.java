package frc.trigon.robot.subsystems.elevator;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;
import org.trigon.commands.GearRatioCalculationCommand;
import org.trigon.commands.NetworkTablesCommand;

public class ElevatorCommands {
    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                ElevatorCommands::getSetTargetPositionCommand,
                false,
                "Debugging/TargetDebuggingElevatorPositionMeters"
        );
    }

    public static Command getGearRatioCalibrationCommand() {
        return new GearRatioCalculationCommand(
                RobotContainer.ELEVATOR::getRotorPosition,
                RobotContainer.ELEVATOR::getEncoderPosition,
                (voltage) -> RobotContainer.ELEVATOR.drive(Units.Volt.of(voltage)),
                RobotContainer.ELEVATOR
        );
    }

    public static Command getSetTargetPositionCommand(double targetPositionMeters) {
        return new StartEndCommand(
                () -> RobotContainer.ELEVATOR.setTargetPosition(targetPositionMeters, 100),
                RobotContainer.ELEVATOR::stop,
                RobotContainer.ELEVATOR
        );
    }

    public static Command getSetTargetStateCommand(ElevatorConstants.ElevatorState targetState) {
        return new StartEndCommand(
                () -> RobotContainer.ELEVATOR.setTargetState(targetState),
                RobotContainer.ELEVATOR::stop,
                RobotContainer.ELEVATOR
        );
    }
}
