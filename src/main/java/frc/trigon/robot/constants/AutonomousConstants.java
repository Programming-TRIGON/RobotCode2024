package frc.trigon.robot.constants;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.Commands;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.intake.IntakeConstants;
import frc.trigon.robot.subsystems.transporter.TransporterCommands;
import frc.trigon.robot.subsystems.transporter.TransporterConstants;
import frc.trigon.robot.subsystems.shooter.ShooterCommands;

public class AutonomousConstants {
    public static final PathConstraints REAL_TIME_CONSTRAINTS = new PathConstraints(3.3, 3.3, 4, 4);

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
        NamedCommands.registerCommand("RollerCollection", TransporterCommands.getSetTargetStateCommand(TransporterConstants.TransporterState.COLLECTING));
        NamedCommands.registerCommand("StopShooting", ShooterCommands.getStopShootingCommand());
        NamedCommands.registerCommand("PrepareShooting", Commands.getPrepareShootingForAutoCommand());
        NamedCommands.registerCommand("FeedNote", TransporterCommands.getSetTargetStateCommand(TransporterConstants.TransporterState.FEEDING).until(() -> RobotContainer.SHOOTER.didShootNote() || !RobotContainer.TRANSPORTER.didCollectNote()));
        NamedCommands.registerCommand("FeedNoteWithoutWaiting", TransporterCommands.getSetTargetStateCommand(TransporterConstants.TransporterState.AUTONOMOUS_FEEDING));
        NamedCommands.registerCommand("IntakeCollection", IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.COLLECTING));
        NamedCommands.registerCommand("IntakeStopping", IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.STOPPED));
    }
}