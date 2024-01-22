package frc.trigon.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.CommandConstants;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.constants.ShootingConstants;
import frc.trigon.robot.subsystems.MotorSubsystem;
import frc.trigon.robot.subsystems.pitcher.PitcherCommands;
import frc.trigon.robot.subsystems.shooter.ShooterCommands;
import frc.trigon.robot.subsystems.swerve.Swerve;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import frc.trigon.robot.utilities.ShootingCalculations;

public class Commands {
    private static boolean IS_BRAKING = true;

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

    public static Command getPrepareShootingCommand() {
        return new ParallelCommandGroup(
                PitcherCommands.getPitchToSpeakerCommand(),
                ShooterCommands.getShootAtSpeakerCommand(),
                SwerveCommands.getClosedLoopFieldRelativeDriveCommand(
                        () -> CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getLeftY()),
                        () -> CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getLeftX()),
                        ShootingCalculations::calculateTargetRobotAngle
                )
        );
    }

    private static double getDistanceToSpeaker() {
        final Pose2d mirroredAlliancePose = RobotContainer.POSE_ESTIMATOR.getCurrentPose().toMirroredAlliancePose();
        final double distanceOffset = getDistanceOffsetToSpeaker();
        return mirroredAlliancePose.getTranslation().getDistance(FieldConstants.SPEAKER_TRANSLATION) + distanceOffset;
    }

    private static Rotation2d getAngleToSpeaker(double yOffset) {
        final Pose2d mirroredAlliancePose = RobotContainer.POSE_ESTIMATOR.getCurrentPose().toMirroredAlliancePose();
        final Translation2d difference = mirroredAlliancePose.getTranslation().minus(FieldConstants.SPEAKER_TRANSLATION.plus(new Translation2d(0, yOffset)));
        return Rotation2d.fromRadians(Math.atan2(difference.getY(), difference.getX()));
    }

    private static double getDistanceOffsetToSpeaker() {
        final Rotation2d angleToMiddleOfSpeaker = getAngleToSpeaker(0);
        final int signum = (int) Math.signum(angleToMiddleOfSpeaker.getDegrees());
        return ShootingConstants.DISTANCE_OFFSET_INTERPOLATION.predict(angleToMiddleOfSpeaker.getDegrees() * signum) * signum;
    }
}
