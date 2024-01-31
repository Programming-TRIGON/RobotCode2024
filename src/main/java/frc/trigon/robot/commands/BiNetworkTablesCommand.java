package frc.trigon.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import java.util.function.BiConsumer;
import java.util.function.BiFunction;

public class BiNetworkTablesCommand extends Command {
    private final BiConsumer<Double, Double> toRun;
    private final LoggedDashboardNumber dashboardNumber1, dashboardNumber2;

    public BiNetworkTablesCommand(String key1, String key2, BiConsumer<Double, Double> toRun) {
        this.toRun = toRun;
        dashboardNumber1 = new LoggedDashboardNumber(key1);
        dashboardNumber2 = new LoggedDashboardNumber(key2);
    }

    public BiNetworkTablesCommand(String key1, String key2, BiFunction<Double, Double, Command> commandCreator) {
        this(key1, key2, (Double value1, Double value2) -> commandCreator.apply(value1, value2).initialize());
    }

    @Override
    public void initialize() {
        toRun.accept(dashboardNumber1.get(), dashboardNumber2.get());
    }
}
