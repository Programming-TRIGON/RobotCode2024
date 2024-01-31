package frc.trigon.robot.commands.LEDAutoSetup;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.subsystems.ledstrip.LEDStrip;
import frc.trigon.robot.subsystems.ledstrip.LEDStripCommands;
import frc.trigon.robot.subsystems.ledstrip.LEDStripConstants;

public class LEDAutoSetup extends ParallelCommandGroup {
    private final Pose2d targetPose;
    private final LEDStrip
            leftStrip = LEDStripConstants.LEFT_STRIP,
            rightStrip = LEDStripConstants.RIGHT_STRIP;
    private final double toleranceMeters = 0.02;

    public LEDAutoSetup(Pose2d targetPose) {
        this.targetPose = targetPose;
        addCommands(
                getSetLEDStripsCommand()
        );
    }

    private Command getSetLEDStripsCommand() {
        return new StartEndCommand(
                this::getSetLeftStripCommand,
                this::getSetRightStripCommand
        );
    }

    private void getSetLeftStripCommand() {
        final Color firstSectionColor, secondSectionColor, thirdSectionColor;

        if (Math.abs(this.targetPose.getRotation().getDegrees() - getCurrentRobotPose().getRotation().getDegrees()) < toleranceMeters)
            firstSectionColor = Color.kYellow;
        else
            firstSectionColor = Color.kGreen;
        if (Math.abs(this.targetPose.getX() - getCurrentRobotPose().getX()) < toleranceMeters)
            secondSectionColor = Color.kYellow;
        else
            secondSectionColor = Color.kGreen;
        if (Math.abs(this.targetPose.getY() - getCurrentRobotPose().getY()) < toleranceMeters)
            thirdSectionColor = Color.kYellow;
        else
            thirdSectionColor = Color.kGreen;

        LEDStripCommands.getThreeSectionColorCommand(firstSectionColor, secondSectionColor, thirdSectionColor, leftStrip);
    }

    private void getSetRightStripCommand() {
        final Color firstSectionColor, secondSectionColor, thirdSectionColor;

        if (Math.abs(this.targetPose.getRotation().getDegrees() - getCurrentRobotPose().getRotation().getDegrees()) > toleranceMeters)
            firstSectionColor = Color.kYellow;
        else
            firstSectionColor = Color.kGreen;
        if (Math.abs(this.targetPose.getX() - getCurrentRobotPose().getX()) > toleranceMeters)
            secondSectionColor = Color.kYellow;
        else
            secondSectionColor = Color.kGreen;
        if (Math.abs(this.targetPose.getY() - getCurrentRobotPose().getY()) > toleranceMeters)
            thirdSectionColor = Color.kYellow;
        else
            thirdSectionColor = Color.kGreen;

        LEDStripCommands.getThreeSectionColorCommand(firstSectionColor, secondSectionColor, thirdSectionColor, rightStrip);
    }

    private Pose2d getCurrentRobotPose() {
        return RobotContainer.POSE_ESTIMATOR.getCurrentPose().toAlliancePose();
    }
}
