package frc.trigon.robot.constants;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.wpilibj2.command.PrintCommand;

public class AutonomousConstants {
    public static final PathConstraints REAL_TIME_CONSTRAINTS = new PathConstraints(2.5, 2.5, 4, 4);

    static {
        registerCommands();
    }

    private static void registerCommands() {
//        NamedCommands.registerCommand("RollerCollection", RollerCommands.getSetTargetStateCommand(RollerConstants.RollerState.COLLECTING));
//        NamedCommands.registerCommand("StopShooting", ShooterCommands.getStopShootingCommand());
//        NamedCommands.registerCommand("PrepareShooting", Commands.getPrepareShootingForAutoCommand());
//        NamedCommands.registerCommand("FeedNote", RollerCommands.getSetTargetStateCommand(RollerConstants.RollerState.FEEDING));
//        NamedCommands.registerCommand("IntakeCollection", IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.COLLECTING));
//        NamedCommands.registerCommand("IntakeOpening", IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.OPENING));
        NamedCommands.registerCommand("RollerCollection", new PrintCommand("RollerCollection"));
        NamedCommands.registerCommand("StopShooting", new PrintCommand("StopShooting"));
        NamedCommands.registerCommand("PrepareShooting", new PrintCommand("PrepareShooting"));
        NamedCommands.registerCommand("FeedNote", new PrintCommand("FeedNote"));
        NamedCommands.registerCommand("IntakeCollection", new PrintCommand("IntakeCollection"));
        NamedCommands.registerCommand("IntakeOpening", new PrintCommand("IntakeOpening"));
        NamedCommands.registerCommand("sout", new PrintCommand("sout"));

    }
}