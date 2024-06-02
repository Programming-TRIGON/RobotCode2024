package frc.trigon.robot.constants;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.Commands;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.intake.IntakeConstants;
import frc.trigon.robot.subsystems.pitcher.PitcherCommands;
import frc.trigon.robot.subsystems.shooter.ShooterCommands;
import frc.trigon.robot.subsystems.transporter.TransporterCommands;
import frc.trigon.robot.subsystems.transporter.TransporterConstants;

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
        NamedCommands.registerCommand("TransporterCollection", TransporterCommands.getSetTargetStateCommand(TransporterConstants.TransporterState.COLLECTING));
        NamedCommands.registerCommand("StopShooting", ShooterCommands.getStopShootingCommand());
        NamedCommands.registerCommand("PrepareShooting", Commands.getReachSpeakerShootingTargetForAutoCommand());
        NamedCommands.registerCommand("FeedNote", TransporterCommands.getSetTargetStateCommand(TransporterConstants.TransporterState.FEEDING).until(() -> !RobotContainer.TRANSPORTER.isNoteDetected()).alongWith(Commands.getVisualizeNoteShootingCommand()));
        NamedCommands.registerCommand("FeedNoteWithoutWaiting", TransporterCommands.getSetTargetStateCommand(TransporterConstants.TransporterState.AUTONOMOUS_FEEDING).alongWith(Commands.getVisualizeNoteShootingCommand()));
        NamedCommands.registerCommand("IntakeCollection", IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.COLLECTING));
        NamedCommands.registerCommand("IntakeStopping", IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.STOPPED));
        NamedCommands.registerCommand("ShootingEject", ShooterCommands.getSetTargetShootingVelocityCommand(30).alongWith(PitcherCommands.getSetTargetPitchCommand(Rotation2d.fromDegrees(26))));
    }
}