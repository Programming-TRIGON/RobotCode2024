package frc.trigon.robot.commands.LEDAutoSetup;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.subsystems.ledstrip.LEDStrip;
import frc.trigon.robot.subsystems.ledstrip.LEDStripCommands;
import frc.trigon.robot.subsystems.ledstrip.LEDStripConstants;

import java.util.function.Supplier;

public class LEDAutoSetupCommand extends ParallelCommandGroup {
    private static final double TOLERANCE_METERS = 0.02;
    private final Pose2d targetPose;
    private final LEDStrip
            leftStrip = LEDStripConstants.LEFT_STRIP,
            rightStrip = LEDStripConstants.RIGHT_STRIP;

    public LEDAutoSetupCommand(Supplier<Pose2d> targetPose) {
        this.targetPose = targetPose.get();
        addCommands(
                LEDStripCommands.getThreeSectionColorCommand(this::getLeftFirstSectionColor, this::getLeftSecondSectionColor, this::getLeftThirdSectionColor, leftStrip),
                LEDStripCommands.getThreeSectionColorCommand(this::getRightFirstSectionColor, this::getRightSecondSectionColor, this::getRightThirdSectionColor, rightStrip)
        );
    }

    private Color getLeftFirstSectionColor() {
        if (Math.abs(this.targetPose.getRotation().getDegrees() - getCurrentRobotPose().getRotation().getDegrees()) < TOLERANCE_METERS)
            return Color.kYellow;
        return Color.kGreen;
    }

    private Color getLeftSecondSectionColor() {
        if (Math.abs(this.targetPose.getX() - getCurrentRobotPose().getX()) < TOLERANCE_METERS)
            return Color.kYellow;
        return Color.kGreen;
    }

    private Color getLeftThirdSectionColor() {
        if (Math.abs(this.targetPose.getY() - getCurrentRobotPose().getY()) < TOLERANCE_METERS)
            return Color.kYellow;
        return Color.kGreen;
    }

    private Color getRightFirstSectionColor() {
        if (Math.abs(this.targetPose.getRotation().getDegrees() - getCurrentRobotPose().getRotation().getDegrees()) > TOLERANCE_METERS)
            return Color.kYellow;
        return Color.kGreen;
    }

    private Color getRightSecondSectionColor() {
        if (Math.abs(this.targetPose.getX() - getCurrentRobotPose().getX()) > TOLERANCE_METERS)
            return Color.kYellow;
        return Color.kGreen;
    }

    private Color getRightThirdSectionColor() {
        if (Math.abs(this.targetPose.getY() - getCurrentRobotPose().getY()) > TOLERANCE_METERS)
            return Color.kYellow;
        return Color.kGreen;
    }

    private Pose2d getCurrentRobotPose() {
        return RobotContainer.POSE_ESTIMATOR.getCurrentPose().toAlliancePose();
    }
}
