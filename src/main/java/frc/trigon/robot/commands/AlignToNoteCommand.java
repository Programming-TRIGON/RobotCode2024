package frc.trigon.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.components.objectdetectioncamera.ObjectDetectionCamera;
import frc.trigon.robot.constants.CameraConstants;
import frc.trigon.robot.constants.CommandConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.ledstrip.LEDStripCommands;
import frc.trigon.robot.subsystems.ledstrip.LEDStripConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;

import java.awt.*;

public class AlignToNoteCommand extends ParallelCommandGroup {
    private static final ObjectDetectionCamera CAMERA = CameraConstants.NOTE_DETECTION_CAMERA;
    private static final PIDController Y_PID_CONTROLLER = new PIDController(0.005, 0, 0);

    public AlignToNoteCommand() {
        addCommands(
                getCurrentLEDColorCommand().asProxy(),
                Commands.getContinuousConditionalCommand(getDriveWhileAligningToNoteCommand(), Commands.duplicate(CommandConstants.SELF_RELATIVE_DRIVE_COMMAND), CAMERA::hasTargets).asProxy()
        );
    }

    private Command getCurrentLEDColorCommand() {
        return Commands.getContinuousConditionalCommand(
                LEDStripCommands.getStaticColorCommand(Color.green, LEDStripConstants.LED_STRIPS),
                LEDStripCommands.getStaticColorCommand(Color.red, LEDStripConstants.LED_STRIPS),
                CAMERA::hasTargets
        );
    }

    private Command getDriveWhileAligningToNoteCommand() {
        return SwerveCommands.getClosedLoopSelfRelativeDriveCommand(
                () -> CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getLeftY()),
                () -> Y_PID_CONTROLLER.calculate(-CAMERA.getObjectYaw()),
                this::getTargetAngle
        );
    }

    private Rotation2d getTargetAngle() {
        final Rotation2d currentRotation = RobotContainer.POSE_ESTIMATOR.getCurrentPose().toBlueAlliancePose().getRotation();
        return currentRotation.plus(Rotation2d.fromDegrees(CAMERA.getObjectYaw()));
    }
}
