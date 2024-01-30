package frc.trigon.robot.constants;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.trigon.robot.components.KeyboardController;
import frc.trigon.robot.components.XboxController;

public class OperatorConstants {
    private static final int
            DRIVER_CONTROLLER_PORT = 0;
    private static final int DRIVER_CONTROLLER_EXPONENT = 1;
    private static final double DRIVER_CONTROLLER_DEADBAND = 0.1;
    public static final XboxController DRIVER_CONTROLLER = new XboxController(
            DRIVER_CONTROLLER_PORT, DRIVER_CONTROLLER_EXPONENT, DRIVER_CONTROLLER_DEADBAND
    );
    public static final KeyboardController OPERATOR_CONTROLLER = new KeyboardController();

    public static final double
            POV_DIVIDER = 2,
            STICKS_SPEED_DIVIDER = 1;

    public static final Trigger
            RESET_HEADING_TRIGGER = DRIVER_CONTROLLER.y(),
            TOGGLE_BRAKE_TRIGGER = OPERATOR_CONTROLLER.g().or(RobotController::getUserButton),
            TOGGLE_FIELD_AND_SELF_RELATIVE_DRIVE_TRIGGER = DRIVER_CONTROLLER.b(),
            DRIVE_FROM_DPAD_TRIGGER = new Trigger(() -> DRIVER_CONTROLLER.getPov() != -1),
            COLLECT_TRIGGER = DRIVER_CONTROLLER.leftTrigger(),
            CONTINUE_TRIGGER = DRIVER_CONTROLLER.leftBumper().or(OPERATOR_CONTROLLER.k()),
            SECOND_CONTINUE_TRIGGER = DRIVER_CONTROLLER.rightBumper(),
            DRIVE_TO_AMP_TRIGGER = DRIVER_CONTROLLER.a(),
            CLOSE_SHOT_TRIGGER = DRIVER_CONTROLLER.x(),
            TURN_AUTOMATIC_NOTE_ALIGNING_ON_TRIGGER = OPERATOR_CONTROLLER.o(),
            TURN_AUTOMATIC_NOTE_ALIGNING_OFF_TRIGGER = OPERATOR_CONTROLLER.p(),
            SCORE_IN_AMP_TRIGGER = OPERATOR_CONTROLLER.a(),
            SHOOT_AT_SPEAKER_TRIGGER = OPERATOR_CONTROLLER.s(),
            CLIMB_TRIGGER = OPERATOR_CONTROLLER.c(),
            OVERRIDE_IS_CLIMBING_TRIGGER = OPERATOR_CONTROLLER.i();
}
