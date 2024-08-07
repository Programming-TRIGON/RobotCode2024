package frc.trigon.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.subsystems.climber.ClimberCommands;
import frc.trigon.robot.subsystems.ledstrip.LEDStripCommands;
import frc.trigon.robot.subsystems.ledstrip.LEDStripConstants;
import frc.trigon.robot.subsystems.pitcher.PitcherCommands;
import frc.trigon.robot.subsystems.pitcher.PitcherConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import org.littletonrobotics.junction.Logger;
import org.trigon.hardware.misc.XboxController;
import org.trigon.utilities.mirrorable.Mirrorable;
import org.trigon.utilities.mirrorable.MirrorableRotation2d;

import java.awt.*;

public class CommandConstants {
    public static boolean
            SHOULD_ALIGN_TO_NOTE = true,
            IS_CLIMBING = false;
    private static final XboxController DRIVER_CONTROLLER = OperatorConstants.DRIVER_CONTROLLER;
    private static final double
            MINIMUM_TRANSLATION_SHIFT_POWER = 0.18,
            MINIMUM_ROTATION_SHIFT_POWER = 0.4;

    public static final Command
            FIELD_RELATIVE_DRIVE_COMMAND = SwerveCommands.getOpenLoopFieldRelativeDriveCommand(
            () -> calculateDriveStickAxisValue(DRIVER_CONTROLLER.getLeftY()),
            () -> calculateDriveStickAxisValue(DRIVER_CONTROLLER.getLeftX()),
            () -> calculateRotationStickAxisValue(DRIVER_CONTROLLER.getRightX())
    ),
            SELF_RELATIVE_DRIVE_COMMAND = SwerveCommands.getOpenLoopSelfRelativeDriveCommand(
                    () -> calculateDriveStickAxisValue(DRIVER_CONTROLLER.getLeftY()),
                    () -> calculateDriveStickAxisValue(DRIVER_CONTROLLER.getLeftX()),
                    () -> calculateRotationStickAxisValue(DRIVER_CONTROLLER.getRightX())
            ),
            RESET_HEADING_COMMAND = new InstantCommand(() -> RobotContainer.POSE_ESTIMATOR.resetPose(changeRotation(RobotContainer.POSE_ESTIMATOR.getCurrentPose(), new MirrorableRotation2d(0, true)))),
            SELF_RELATIVE_DRIVE_FROM_DPAD_COMMAND = SwerveCommands.getOpenLoopSelfRelativeDriveCommand(
                    () -> getXPowerFromPov(DRIVER_CONTROLLER.getPov()) / OperatorConstants.POV_DIVIDER / calculateShiftModeValue(MINIMUM_TRANSLATION_SHIFT_POWER),
                    () -> getYPowerFromPov(DRIVER_CONTROLLER.getPov()) / OperatorConstants.POV_DIVIDER / calculateShiftModeValue(MINIMUM_TRANSLATION_SHIFT_POWER),
                    () -> 0
            ),
            PITCHER_RESTING_COMMAND = PitcherCommands.getSetTargetPitchCommand(PitcherConstants.DEFAULT_PITCH),
            STATIC_WHITE_LED_COLOR_COMMAND = LEDStripCommands.getStaticColorCommand(Color.white, LEDStripConstants.LED_STRIPS),
            DRIVE_TO_AMP_COMMAND = SwerveCommands.getDriveToPoseCommand(
                    () -> FieldConstants.IN_FRONT_OF_AMP_POSE,
                    AutonomousConstants.REAL_TIME_CONSTRAINTS
            ),
            FACE_AMP_COMMAND = SwerveCommands.getClosedLoopFieldRelativeDriveCommand(
                    () -> CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getLeftY()),
                    () -> CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getLeftX()),
                    () -> MirrorableRotation2d.fromDegrees(90, true)
            ),
            FACE_SPEAKER_COMMAND = SwerveCommands.getClosedLoopFieldRelativeDriveCommand(
                    () -> CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getLeftY()),
                    () -> CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getLeftX()),
                    () -> MirrorableRotation2d.fromDegrees(0, true)
            ),
            AMPLIFY_LEDS_COMMAND = LEDStripCommands.getAnimateSingleFadeCommand(
                    Color.magenta,
                    1,
                    LEDStripConstants.LED_STRIPS
            ),
            MOVE_CLIMBER_DOWN_MANUALLY_COMMAND = ClimberCommands.getSetTargetVoltageCommand(-4),
            MOVE_CLIMBER_UP_MANUALLY_COMMAND = ClimberCommands.getSetTargetVoltageCommand(4),
            TURN_AUTOMATIC_NOTE_ALIGNING_ON_COMMAND = new InstantCommand(() -> {
                SHOULD_ALIGN_TO_NOTE = true;
                Logger.recordOutput("ShouldAlignToNote", true);
            }).ignoringDisable(true),
            TURN_AUTOMATIC_NOTE_ALIGNING_OFF_COMMAND = new InstantCommand(() -> {
                SHOULD_ALIGN_TO_NOTE = false;
                Logger.recordOutput("ShouldAlignToNote", false);
            }).ignoringDisable(true),
            OVERRIDE_IS_CLIMBING_COMMAND = new InstantCommand(() -> {
                IS_CLIMBING = false;
                Logger.recordOutput("IsClimbing", false);
            }).ignoringDisable(true),
            ALIGN_TO_RIGHT_STAGE_COMMAND = SwerveCommands.getClosedLoopFieldRelativeDriveCommand(
                    () -> CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getLeftY()),
                    () -> CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getLeftX()),
                    () -> Mirrorable.isRedAlliance() ? MirrorableRotation2d.fromDegrees(60, false) : MirrorableRotation2d.fromDegrees(-120, false)
            ),
            ALIGN_TO_LEFT_STAGE_COMMAND = SwerveCommands.getClosedLoopFieldRelativeDriveCommand(
                    () -> CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getLeftY()),
                    () -> CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getLeftX()),
                    () -> Mirrorable.isRedAlliance() ? MirrorableRotation2d.fromDegrees(-60, false) : MirrorableRotation2d.fromDegrees(120, false)
            ),
            ALIGN_TO_MIDDLE_STAGE_COMMAND = SwerveCommands.getClosedLoopFieldRelativeDriveCommand(
                    () -> CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getLeftY()),
                    () -> CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getLeftX()),
                    () -> MirrorableRotation2d.fromDegrees(180, true)
            );

    public static double calculateDriveStickAxisValue(double axisValue) {
        return axisValue / OperatorConstants.STICKS_SPEED_DIVIDER / calculateShiftModeValue(MINIMUM_TRANSLATION_SHIFT_POWER);
    }

    public static double calculateRotationStickAxisValue(double axisValue) {
        return axisValue / OperatorConstants.STICKS_SPEED_DIVIDER / calculateShiftModeValue(MINIMUM_ROTATION_SHIFT_POWER);
    }

    /**
     * The shift mode is a mode of the robot that slows down the robot relative to how much the right trigger axis is pressed.
     * This method will take the given power, and slow it down relative to how much the right trigger is pressed.
     *
     * @param minimumPower the minimum amount of power the shift mode can limit (as an absolute number)
     * @return the power to apply to the robot
     */
    public static double calculateShiftModeValue(double minimumPower) {
        final double squaredShiftModeValue = DRIVER_CONTROLLER.getRightTriggerAxis();
        final double minimumShiftValueCoefficient = 1 - (1 / minimumPower);

        return 1 - squaredShiftModeValue * minimumShiftValueCoefficient;
    }

    private static double getXPowerFromPov(double pov) {
        final double povRadians = Units.degreesToRadians(pov);
        return Math.cos(povRadians);
    }

    private static double getYPowerFromPov(double pov) {
        final double povRadians = Units.degreesToRadians(pov);
        return Math.sin(-povRadians);
    }

    private static Pose2d changeRotation(Pose2d pose2d, MirrorableRotation2d newRotation) {
        return new Pose2d(pose2d.getTranslation(), newRotation.get());
    }
}
