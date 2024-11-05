package frc.trigon.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.CameraConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.misc.objectdetectioncamera.ObjectDetectionCamera;
import frc.trigon.robot.subsystems.ledstrip.LEDStripCommands;
import frc.trigon.robot.subsystems.ledstrip.LEDStripConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import org.trigon.hardware.RobotHardwareStats;
import org.trigon.utilities.mirrorable.MirrorableRotation2d;

import java.awt.*;

public class AlignToNoteCommand extends ParallelCommandGroup {
    private static final ObjectDetectionCamera CAMERA = CameraConstants.NOTE_DETECTION_CAMERA;
    private static final PIDController Y_PID_CONTROLLER = RobotHardwareStats.isSimulation() ?
            new PIDController(0.015, 0, 0) :
            new PIDController(0.005, 0, 0);

    public AlignToNoteCommand() {
        addCommands(
                getSetCurrentLEDColorCommand().asProxy(),
                Commands.getContinuousConditionalCommand(getDriveWhileAligningToNoteCommand(), Commands.duplicate(CommandConstants.FIELD_RELATIVE_DRIVE_COMMAND), this::hasTarget).asProxy(),
                new RunCommand(CAMERA::trackObject)
        );
    }

    private Command getSetCurrentLEDColorCommand() {
        return Commands.getContinuousConditionalCommand(
                LEDStripCommands.getStaticColorCommand(Color.green, LEDStripConstants.LED_STRIPS),
                LEDStripCommands.getStaticColorCommand(Color.red, LEDStripConstants.LED_STRIPS),
                CAMERA::hasTargets
        );
    }

    private Command getDriveWhileAligningToNoteCommand() {
        return SwerveCommands.getClosedLoopSelfRelativeDriveCommand(
                () -> fieldRelativePowersToSelfRelativeXPower(OperatorConstants.DRIVER_CONTROLLER.getLeftY(), OperatorConstants.DRIVER_CONTROLLER.getLeftX()),
                () -> -Y_PID_CONTROLLER.calculate(CAMERA.getTrackedObjectYaw().getDegrees()),
                this::getTargetAngle
        );
    }

    private double fieldRelativePowersToSelfRelativeXPower(double xPower, double yPower) {
        final Rotation2d driveRelativeAngle = RobotContainer.SWERVE.getDriveRelativeAngle();
        return (CommandConstants.calculateDriveStickAxisValue(xPower) * driveRelativeAngle.getCos()) + (CommandConstants.calculateDriveStickAxisValue(yPower) * driveRelativeAngle.getSin());
    }

    private MirrorableRotation2d getTargetAngle() {
        final Rotation2d currentRotation = RobotContainer.POSE_ESTIMATOR.getCurrentPose().getRotation();
        return new MirrorableRotation2d(currentRotation.plus(CAMERA.getTrackedObjectYaw()), false);
    }

    private boolean hasTarget() {
        return CAMERA.hasTargets() && !RobotContainer.INTAKE.isEarlyNoteCollectionDetected();
    }
}