package frc.trigon.robot.commands;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.subsystems.ledstrip.LEDStripCommands;
import frc.trigon.robot.subsystems.ledstrip.LEDStripConstants;

import java.util.function.Supplier;

/**
 * A command that sets the LED strips to a color based on the robot's position and orientation relative to the starting
 * pose of the selected autonomous path.
 * This is very useful for placing the robot in the correct starting position and orientation for autonomous mode before a match.
 */
public class LEDAutoSetupCommand extends ParallelCommandGroup {
    private static final double
            TOLERANCE_METERS = 0.02,
            TOLERANCE_DEGREES = 2;
    private final Supplier<String> autoName;
    private Pose2d autoStartPose;

    /**
     * Constructs a new LEDAutoSetupCommand.
     *
     * @param autoName a supplier that returns the name of the selected autonomous path
     */
    public LEDAutoSetupCommand(Supplier<String> autoName) {
        this.autoName = autoName;

        addCommands(
                getUpdateAutoStartPoseCommand(),
                LEDStripCommands.getThreeSectionColorCommand(
                        () -> getLeftStripColor(this.autoStartPose.getRotation().getDegrees() - getCurrentRobotPose().getRotation().getDegrees(), TOLERANCE_DEGREES),
                        () -> getLeftStripColor(this.autoStartPose.getX() - getCurrentRobotPose().getX(), TOLERANCE_METERS),
                        () -> getLeftStripColor(this.autoStartPose.getY() - getCurrentRobotPose().getY(), TOLERANCE_METERS),
                        LEDStripConstants.LEFT_STRIP
                ),
                LEDStripCommands.getThreeSectionColorCommand(
                        () -> getRightStripColor(this.autoStartPose.getRotation().getDegrees() - getCurrentRobotPose().getRotation().getDegrees(), TOLERANCE_DEGREES),
                        () -> getRightStripColor(this.autoStartPose.getX() - getCurrentRobotPose().getX(), TOLERANCE_METERS),
                        () -> getRightStripColor(this.autoStartPose.getY() - getCurrentRobotPose().getY(), TOLERANCE_METERS),
                        LEDStripConstants.RIGHT_STRIP
                )
        );
    }

    private Command getUpdateAutoStartPoseCommand() {
        return new InstantCommand(
                () -> this.autoStartPose = PathPlannerAuto.getStaringPoseFromAutoFile(autoName.get())
        );
    }

    private Color getLeftStripColor(double difference, double tolerance) {
        if (difference < -tolerance)
            return Color.kBlack;
        else if (difference > tolerance)
            return Color.kRed;
        return Color.kGreen;
    }

    private Color getRightStripColor(double difference, double tolerance) {
        if (difference > tolerance)
            return Color.kBlack;
        else if (difference < -tolerance)
            return Color.kRed;
        return Color.kGreen;
    }

    private Pose2d getCurrentRobotPose() {
        return RobotContainer.POSE_ESTIMATOR.getCurrentPose().toBlueAlliancePose();
    }
}
