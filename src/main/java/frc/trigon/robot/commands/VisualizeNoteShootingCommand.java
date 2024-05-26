package frc.trigon.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.ShootingConstants;
import frc.trigon.robot.utilities.ShootingCalculations;
import frc.trigon.robot.utilities.mirrorable.MirrorableRotation2d;
import org.littletonrobotics.junction.Logger;

/**
 * A command to visualize note shooting.
 * This command will get the physical information from subsystems when we begin the shot, and calculate the note's position at each timestamp using physics.
 */
public class VisualizeNoteShootingCommand extends Command {
    private static final ShootingCalculations SHOOTING_CALCULATIONS = ShootingCalculations.getInstance();
    private double startingTimeSeconds;
    private Pose3d startingEndEffectorFieldRelativePoseNoPitch;
    private Rotation2d startingPitch;
    private double initialXYVelocity, initialZVelocity;
    private double noteZ;

    @Override
    public void initialize() {
        startingTimeSeconds = Timer.getFPGATimestamp();
        startingPitch = RobotContainer.PITCHER.getCurrentPitch();
        final Rotation2d currentRobotAngle = RobotContainer.POSE_ESTIMATOR.getCurrentPose().getRotation();
        final Pose3d startingEndEffectorFieldRelativePoseWithPitch = SHOOTING_CALCULATIONS.calculateShooterEndEffectorFieldRelativePose(startingPitch, SHOOTING_CALCULATIONS.getPredictedTranslation(), new MirrorableRotation2d(currentRobotAngle, false));
        startingEndEffectorFieldRelativePoseNoPitch = new Pose3d(startingEndEffectorFieldRelativePoseWithPitch.getTranslation(), new Rotation3d(0, 0, currentRobotAngle.getRadians() + Math.PI));
        final double startingTangentialVelocity = SHOOTING_CALCULATIONS.angularVelocityToTangentialVelocity(RobotContainer.SHOOTER.getCurrentVelocityRevolutionsPerSecond());
        initialXYVelocity = startingPitch.getCos() * startingTangentialVelocity;
        initialZVelocity = startingPitch.getSin() * startingTangentialVelocity;
    }

    @Override
    public void execute() {
        final double t = Timer.getFPGATimestamp() - startingTimeSeconds;
        final double zPosition = (initialZVelocity * t) + (-0.5 * ShootingConstants.G_FORCE * Math.pow(t, 2));
        final double xyPosition = initialXYVelocity * t;
        final Transform3d transform = new Transform3d(xyPosition, 0, zPosition, new Rotation3d(0, -startingPitch.getRadians(), 0));
        final Pose3d notePose = startingEndEffectorFieldRelativePoseNoPitch.plus(transform);
        noteZ = notePose.getTranslation().getZ();
        Logger.recordOutput("Poses/GamePieces/ShotNotePose", notePose);
    }

    @Override
    public boolean isFinished() {
        return noteZ < 0;
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("Poses/GamePieces/ShotNotePose", new Pose3d(0, 0, 50, new Rotation3d()));
    }
}
