package frc.trigon.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.trigon.robot.utilities.SpeedMechanism2d;
import frc.trigon.robot.utilities.StateSpaceFlywheelController;

public class ShooterConstants {
    static final double TOLERANCE_REVOLUTIONS = 0.1;

    static final double SHOOTING_CURRENT = 40;
    static final double SHOOTING_TIME_THRESHOLD = 0.5;
    static final double
            SHOOTING_RUMBLE_DURATION_SECONDS = 0.4,
            SHOOTING_RUMBLE_POWER = 1;

    private static final double MAX_DISPLAYABLE_VELOCITY = 100;
    static final SpeedMechanism2d SHOOTING_MECHANISM = new SpeedMechanism2d("Mechanisms/ShooterMechanism", MAX_DISPLAYABLE_VELOCITY);

    public static final double GEAR_RATIO = 1;
    public static final double MOMENT_OF_INERTIA = 0.0006677406;
    private static final int MOTOR_AMOUNT = 2;
    public static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(MOTOR_AMOUNT);
    private static final double
            MODEL_ACCURACY = 3.0,
            ENCODER_ACCURACY = 0.01;
    static final StateSpaceFlywheelController STATE_SPACE_CONTROLLER = new StateSpaceFlywheelController(
            DCMotor.getKrakenX60Foc(1),
            MOMENT_OF_INERTIA,
            GEAR_RATIO,
            MODEL_ACCURACY,
            ENCODER_ACCURACY
    );
}
