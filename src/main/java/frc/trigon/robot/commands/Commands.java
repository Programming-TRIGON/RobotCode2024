package frc.trigon.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.trigon.robot.constants.CommandConstants;
import frc.trigon.robot.subsystems.MotorSubsystem;
import frc.trigon.robot.subsystems.pitcher.Pitcher;
import frc.trigon.robot.subsystems.pitcher.PitcherCommands;
import frc.trigon.robot.subsystems.shooter.Shooter;
import frc.trigon.robot.subsystems.shooter.ShooterCommands;
import frc.trigon.robot.subsystems.swerve.Swerve;

public class Commands {
    private static final Pitcher PITCHER = Pitcher.getInstance();
    private static final Shooter SHOOTER = Shooter.getInstance();
    private static boolean IS_BRAKING = true;

    public static Command getPrepareShootingCommand() {
        return new ParallelCommandGroup(
                PitcherCommands.getPitchToSpeakerCommand(),
                ShooterCommands.getShootAtSpeakerCommand()
        ).until(() -> PITCHER.atTargetPitch() && SHOOTER.atTargetShootingVelocity());
    }

    /**
     * @return a command that toggles between the swerve's default command, from field relative to self relative
     */
    public static Command getToggleFieldAndSelfRelativeDriveCommand() {
        return new InstantCommand(() -> {
            if (Swerve.getInstance().getDefaultCommand().equals(CommandConstants.FIELD_RELATIVE_DRIVE_COMMAND))
                Swerve.getInstance().setDefaultCommand(CommandConstants.SELF_RELATIVE_DRIVE_COMMAND);
            else
                Swerve.getInstance().setDefaultCommand(CommandConstants.FIELD_RELATIVE_DRIVE_COMMAND);

            Swerve.getInstance().getDefaultCommand().schedule();
        });
    }

    public static Command getToggleBrakeCommand() {
        return new InstantCommand(() -> {
            IS_BRAKING = !IS_BRAKING;
            MotorSubsystem.setAllSubsystemsBrakeAsync(IS_BRAKING);
        }).ignoringDisable(true);
    }

    public static Command getDelayedCommand(double delaySeconds, Runnable toRun) {
        return new WaitCommand(delaySeconds).andThen(toRun).ignoringDisable(true);
    }
}
