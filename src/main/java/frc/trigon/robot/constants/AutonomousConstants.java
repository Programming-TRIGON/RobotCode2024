package frc.trigon.robot.constants;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import frc.trigon.robot.commands.Commands;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.intake.IntakeConstants;
import frc.trigon.robot.subsystems.roller.RollerCommands;
import frc.trigon.robot.subsystems.roller.RollerConstants;
import frc.trigon.robot.subsystems.shooter.ShooterCommands;

public class AutonomousConstants {
    public static final PathConstraints REAL_TIME_CONSTRAINTS = new PathConstraints(2.5, 2.5, 4, 4);

    static {
        registerCommands();
    }

    /**
     * This method ensures the autonomous constants class is initialized early in the robot boot process.
     * If this method is not called, the autonomous constants will initialize after pathplanner, creating some unexpected issues.
     */
    public static void init() {
    }

    private static void registerCommands() {
        NamedCommands.registerCommand("RollerCollection", RollerCommands.getSetTargetStateCommand(RollerConstants.RollerState.COLLECTING));
        NamedCommands.registerCommand("StopShooting", ShooterCommands.getStopShootingCommand());
        NamedCommands.registerCommand("PrepareShooting", Commands.getPrepareShootingForAutoCommand());
        NamedCommands.registerCommand("FeedNote", RollerCommands.getSetTargetStateCommand(RollerConstants.RollerState.FEEDING));
        NamedCommands.registerCommand("IntakeCollection", IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.COLLECTING));
        NamedCommands.registerCommand("IntakeOpening", IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.OPENING));
    }
}