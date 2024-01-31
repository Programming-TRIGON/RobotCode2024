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
    private Pose2d targetPose;
    private final String autoName;
    private final LEDStrip
            leftStrip = LEDStripConstants.LEFT_STRIP,
            rightStrip = LEDStripConstants.RIGHT_STRIP;

    public LEDAutoSetupCommand(Supplier<String> autoName) {
        this.autoName = autoName.get();
        addCommands(
                getSetTargetPoseCommand(),
                LEDStripCommands.getThreeSectionColorCommand(this::getLeftFirstSectionColor, this::getLeftSecondSectionColor, this::getLeftThirdSectionColor, leftStrip),
                LEDStripCommands.getThreeSectionColorCommand(this::getRightFirstSectionColor, this::getRightSecondSectionColor, this::getRightThirdSectionColor, rightStrip)
        );
    }

    private Command getSetTargetPoseCommand() {
        return new InstantCommand(
                () -> this.targetPose = PathPlannerAuto.getStaringPoseFromAutoFile(autoName)
        );
    }

    private Color getLeftFirstSectionColor() {
        if (this.targetPose.getRotation().getDegrees() - getCurrentRobotPose().getRotation().getDegrees() < TOLERANCE_DEGREES) {
            if (this.targetPose.getRotation().getDegrees() - getCurrentRobotPose().getRotation().getDegrees() > -TOLERANCE_DEGREES)
                return Color.kGreen;
            return Color.kBlack;
        }
        return Color.kRed;
    }

    private Color getLeftSecondSectionColor() {
        if (this.targetPose.getX() - getCurrentRobotPose().getX() < TOLERANCE_METERS) {
            if (this.targetPose.getX() - getCurrentRobotPose().getX() > -TOLERANCE_METERS)
                return Color.kGreen;
            return Color.kBlack;
        }
        return Color.kRed;
    }

    private Color getLeftThirdSectionColor() {
        if (this.targetPose.getY() - getCurrentRobotPose().getY() < TOLERANCE_METERS) {
            if (this.targetPose.getY() - getCurrentRobotPose().getY() > -TOLERANCE_METERS)
                return Color.kGreen;
            return Color.kBlack;
        }
        return Color.kRed;
    }

    private Color getRightFirstSectionColor() {
        if (this.targetPose.getRotation().getDegrees() - getCurrentRobotPose().getRotation().getDegrees() > TOLERANCE_DEGREES) {
            if (this.targetPose.getRotation().getDegrees() - getCurrentRobotPose().getRotation().getDegrees() < -TOLERANCE_DEGREES)
                return Color.kGreen;
            return Color.kBlack;
        }
        return Color.kRed;
    }

    private Color getRightSecondSectionColor() {
        if (this.targetPose.getX() - getCurrentRobotPose().getX() > TOLERANCE_METERS) {
            if (this.targetPose.getX() - getCurrentRobotPose().getX() < -TOLERANCE_METERS)
                return Color.kGreen;
            return Color.kBlack;
        }
        return Color.kRed;
    }

    private Color getRightThirdSectionColor() {
        if (this.targetPose.getY() - getCurrentRobotPose().getY() > TOLERANCE_METERS) {
            if (this.targetPose.getY() - getCurrentRobotPose().getY() < -TOLERANCE_METERS)
                return Color.kGreen;
            return Color.kBlack;
        }
        return Color.kRed;
    }

    private Pose2d getCurrentRobotPose() {
        return RobotContainer.POSE_ESTIMATOR.getCurrentPose().toBlueAlliancePose();
    }
}
