package frc.trigon.robot.subsystems.collector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

public class CollectorCommands {
    private static final Collector COLLECTOR = Collector.getInstance();

    public static Command getSetTargetStateCommand(CollectorConstants.CollectorState state) {
        return new StartEndCommand(
                () -> COLLECTOR.setTargetState(state),
                () -> {
                },
                COLLECTOR
        );
    }
}
