package frc.trigon.robot.commands;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.subsystems.ledstrip.LEDStrip;
import frc.trigon.robot.subsystems.ledstrip.LEDStripCommands;
import frc.trigon.robot.subsystems.ledstrip.LEDStripConstants;

import java.util.function.Supplier;

public class LEDAutoSetupCommand extends ParallelCommandGroup {
    private static final double
            TOLERANCE_METERS = 0.02,
            TOLERANCE_DEGREES = 2;
    private final String autoName;
    private final LEDStrip
            leftStrip = LEDStripConstants.LEFT_STRIP,
            rightStrip = LEDStripConstants.RIGHT_STRIP;
    private Pose2d autoStartPose;

    public LEDAutoSetupCommand(Supplier<String> autoName) {
        this.autoName = autoName.get();
        addCommands(
                getUpdateAutoStartPoseCommand(),
                LEDStripCommands.getThreeSectionColorCommand(
                        () -> getLeftSectionColor(this.autoStartPose.getRotation().getDegrees() - getCurrentRobotPose().getRotation().getDegrees(), TOLERANCE_DEGREES),
                        () -> getLeftSectionColor(this.autoStartPose.getX() - getCurrentRobotPose().getX(), TOLERANCE_METERS),
                        () -> getLeftSectionColor(this.autoStartPose.getY() - getCurrentRobotPose().getY(), TOLERANCE_METERS),
                        leftStrip
                ),
                LEDStripCommands.getThreeSectionColorCommand(
                        () -> getRightSectionColor(this.autoStartPose.getRotation().getDegrees() - getCurrentRobotPose().getRotation().getDegrees(), TOLERANCE_DEGREES),
                        () -> getRightSectionColor(this.autoStartPose.getX() - getCurrentRobotPose().getX(), TOLERANCE_METERS),
                        () -> getRightSectionColor(this.autoStartPose.getY() - getCurrentRobotPose().getY(), TOLERANCE_METERS),
                        rightStrip
                )
        );
    }

    private Command getUpdateAutoStartPoseCommand() {
        return new InstantCommand(
                () -> this.autoStartPose = PathPlannerAuto.getStaringPoseFromAutoFile(autoName)
        );
    }

    private Color getLeftSectionColor(double difference, double tolerance) {
        if (difference < tolerance) {
            return Color.kBlack;
        } else if (difference > tolerance) {
            return Color.kRed;
        }
        return Color.kGreen;
    }

    private Color getRightSectionColor(double difference, double tolerance) {
        if (difference > tolerance) {
            return Color.kBlack;
        } else if (difference < tolerance) {
            return Color.kRed;
        }
        return Color.kGreen;
    }

    private Pose2d getCurrentRobotPose() {
        return RobotContainer.POSE_ESTIMATOR.getCurrentPose().toBlueAlliancePose();
    }
}
