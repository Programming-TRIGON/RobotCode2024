package frc.trigon.robot.constants;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.trigon.robot.hardware.misc.KeyboardController;
import frc.trigon.robot.hardware.misc.XboxController;

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
            DEBUGGING_BUTTON = OPERATOR_CONTROLLER.f3(),
            RESET_HEADING_TRIGGER = DRIVER_CONTROLLER.y(),
            TOGGLE_BRAKE_TRIGGER = OPERATOR_CONTROLLER.g().or(RobotController::getUserButton),
            TOGGLE_FIELD_AND_SELF_RELATIVE_DRIVE_TRIGGER = DRIVER_CONTROLLER.b(),
            DRIVE_FROM_DPAD_TRIGGER = new Trigger(() -> DRIVER_CONTROLLER.getPov() != -1),
            COLLECT_TRIGGER = DRIVER_CONTROLLER.leftTrigger()/*.or(DEBUGGING_BUTTON)*/,
            CONTINUE_TRIGGER = DRIVER_CONTROLLER.leftBumper().or(OPERATOR_CONTROLLER.k()),
            FACE_AMP_TRIGGER = DRIVER_CONTROLLER.x(),
            FACE_SPEAKER_TRIGGER = DRIVER_CONTROLLER.a(),
            SECOND_CONTINUE_TRIGGER = OPERATOR_CONTROLLER.m(),
            CLOSE_SHOT_TRIGGER = OPERATOR_CONTROLLER.x(),
            TURN_AUTOMATIC_NOTE_ALIGNING_ON_TRIGGER = OPERATOR_CONTROLLER.o(),
            TURN_AUTOMATIC_NOTE_ALIGNING_OFF_TRIGGER = OPERATOR_CONTROLLER.p(),
            SCORE_IN_AMP_TRIGGER = OPERATOR_CONTROLLER.a(),
            AUTONOMOUS_SCORE_IN_AMP_TRIGGER = OPERATOR_CONTROLLER.z(),
            SHOOT_AT_SPEAKER_TRIGGER = OPERATOR_CONTROLLER.s().or(DRIVER_CONTROLLER.rightBumper()),
            CLIMB_TRIGGER = OPERATOR_CONTROLLER.c(),
            OVERRIDE_IS_CLIMBING_TRIGGER = OPERATOR_CONTROLLER.i(),
            FORWARD_QUASISTATIC_CHARACTERIZATION_TRIGGER = OPERATOR_CONTROLLER.right(),
            BACKWARD_QUASISTATIC_CHARACTERIZATION_TRIGGER = OPERATOR_CONTROLLER.left(),
            FORWARD_DYNAMIC_CHARACTERIZATION_TRIGGER = OPERATOR_CONTROLLER.up(),
            BACKWARD_DYNAMIC_CHARACTERIZATION_TRIGGER = OPERATOR_CONTROLLER.down(),
            LED_AUTO_SETUP_TRIGGER = OPERATOR_CONTROLLER.backtick(),
            RESET_AUTO_POSE_TRIGGER = OPERATOR_CONTROLLER.period(),
            EJECT_NOTE_TRIGGER = OPERATOR_CONTROLLER.e(),
            AMPLIFY_LEDS_TRIGGER = OPERATOR_CONTROLLER.h(),
            WARM_SPEAKER_SHOOTING_TRIGGER = OPERATOR_CONTROLLER.w(),
            MOVE_CLIMBER_DOWN_MANUALLY_TRIGGER = OPERATOR_CONTROLLER.f1(),
            MOVE_CLIMBER_UP_MANUALLY_TRIGGER = OPERATOR_CONTROLLER.f2(),
            DELIVERY_TRIGGER = OPERATOR_CONTROLLER.d(),
            ALIGN_TO_RIGHT_STAGE = OPERATOR_CONTROLLER.j(),
            ALIGN_TO_LEFT_STAGE = OPERATOR_CONTROLLER.h(),
            ALIGN_TO_MIDDLE_STAGE = OPERATOR_CONTROLLER.u();
}
