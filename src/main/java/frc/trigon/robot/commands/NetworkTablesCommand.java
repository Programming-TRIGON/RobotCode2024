package frc.trigon.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import java.util.function.Consumer;
import java.util.function.Function;

public class NetworkTablesCommand extends Command {
    private final Consumer<Double> toRun;
    private final LoggedDashboardNumber dashboardNumber;

    public NetworkTablesCommand(String key, Consumer<Double> toRun) {
        this.toRun = toRun;
        dashboardNumber = new LoggedDashboardNumber(key);
    }

    public NetworkTablesCommand(String key, Function<Double, Command> commandCreator) {
        this(key, (Double value) -> commandCreator.apply(value).initialize());
    }

    @Override
    public void initialize() {
        toRun.accept(dashboardNumber.get());
    }
}
