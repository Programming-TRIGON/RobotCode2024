package frc.trigon.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.trigon.robot.utilities.LinearInterpolation;
import frc.trigon.robot.utilities.SpeedMechanism2d;
import frc.trigon.robot.utilities.StateSpaceFlywheelController;

public class ShooterConstants {
    public static final double GEAR_RATIO = 1;
    static final double TOP_TO_BOTTOM_SHOOTING_RATIO = 1;
    public static final double
            TOP_MOMENT_OF_INERTIA = 0.0006677406,
            BOTTOM_MOMENT_OF_INERTIA = 0.0006677406;

    static final double TOLERANCE_REVOLUTIONS = 0.1;
    static final LinearInterpolation VELOCITY_INTERPOLATION = generateInterpolation();

    private static final double MAX_DISPLAYABLE_VELOCITY = 100;
    static final SpeedMechanism2d
            TOP_SHOOTING_MECHANISM = new SpeedMechanism2d("Mechanisms/TopShooterMechanism", MAX_DISPLAYABLE_VELOCITY),
            BOTTOM_SHOOTING_MECHANISM = new SpeedMechanism2d("Mechanisms/BottomShooterMechanism", MAX_DISPLAYABLE_VELOCITY);


    private static final double
            MODEL_ACCURACY = 3.0,
            ENCODER_ACCURACY = 0.01;
    static final StateSpaceFlywheelController
            TOP_CONTROLLER = new StateSpaceFlywheelController(
            DCMotor.getKrakenX60Foc(1),
            TOP_MOMENT_OF_INERTIA,
            GEAR_RATIO,
            MODEL_ACCURACY,
            ENCODER_ACCURACY
    ),
            BOTTOM_CONTROLLER = new StateSpaceFlywheelController(
                    DCMotor.getKrakenX60Foc(1),
                    BOTTOM_MOMENT_OF_INERTIA,
                    GEAR_RATIO,
                    MODEL_ACCURACY,
                    ENCODER_ACCURACY
            );
}
