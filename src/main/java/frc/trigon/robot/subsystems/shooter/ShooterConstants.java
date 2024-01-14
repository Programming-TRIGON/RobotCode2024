package frc.trigon.robot.subsystems.shooter;

import frc.trigon.robot.constants.ShootingConstants;
import frc.trigon.robot.utilities.LinearInterpolation;

import java.util.ArrayList;

public class ShooterConstants {
    public static final double SHOOTER_GEAR_RATIO = 1;

    static final double TOLERANCE_REVOLUTIONS = 0.1;
    static final LinearInterpolation VELOCITY_INTERPOLATION = generateInterpolation();
    static final double FEEDING_MOTOR_VOLTAGE = 4;

//    private static final double MAX_DISPLAYABLE_VELOCITY = 10;
//    static final SpeedMechanism2d SHOOTING_MECHANISM = new SpeedMechanism2d("Mechanisms/ShooterMechanism", MAX_DISPLAYABLE_VELOCITY);

    private static LinearInterpolation generateInterpolation() {
        final ArrayList<LinearInterpolation.Point> points = new ArrayList<>();
        for (ShootingConstants.ShootingPosition position : ShootingConstants.SHOOTING_POSITIONS)
            points.add(new LinearInterpolation.Point(position.distanceMeters(), position.shooterVelocityRevolutionsPerSecond()));
        return new LinearInterpolation(points);
    }
}
