package frc.trigon.robot.subsystems.collector;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;

public class CollectorCommands {
    private static final Collector COLLECTOR = RobotContainer.COLLECTOR;

    public static Command getSetTargetStateCommand(CollectorConstants.CollectorState targetState) {
        return getSetTargetStateCommand(targetState.angle, targetState.collectionVoltage);
    }

    public static Command getSetTargetStateCommand(Rotation2d targetAngle, double collectorVoltage) {
        return new StartEndCommand(
                () -> COLLECTOR.setTargetState(targetAngle, collectorVoltage),
                COLLECTOR::stop,
                COLLECTOR
        );
    }
}
