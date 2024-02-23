package frc.trigon.robot.subsystems.climber;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;

public class ClimberCommands {
    private static final Climber CLIMBER = RobotContainer.CLIMBER;

    public static Command getSetTargetPositionCommand(double targetPositionMeters, boolean affectedByWeight) {
        return new StartEndCommand(
                () -> CLIMBER.setTargetPosition(targetPositionMeters, affectedByWeight),
                CLIMBER::stop,
                CLIMBER
        );
    }

    public static Command getSetTargetStateCommand(ClimberConstants.ClimberState targetState) {
        return new StartEndCommand(
                () -> CLIMBER.setTargetState(targetState),
                CLIMBER::stop,
                CLIMBER
        );
    }

    public static Command getSetTargetVoltageCommand(double targetVoltage) {
        return new StartEndCommand(
                () -> CLIMBER.drive(Units.Volt.of(targetVoltage)),
                CLIMBER::stop,
                CLIMBER
        );
    }
}
