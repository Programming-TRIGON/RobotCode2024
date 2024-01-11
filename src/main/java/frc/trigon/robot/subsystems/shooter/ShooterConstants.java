package frc.trigon.robot.subsystems.shooter;

import frc.trigon.robot.constants.ShootingConstants;
import frc.trigon.robot.utilities.LinearInterpolation;

import java.util.ArrayList;

public class ShooterConstants {
    static final double TOLERANCE_ROTATIONS = 0.1;
    static final LinearInterpolation INTERPOLATION = generateInterpolation();
    static final double FEEDING_MOTOR_VOLTAGE = 4;

    private static LinearInterpolation generateInterpolation() {
        final ArrayList<LinearInterpolation.Point> points = new ArrayList<>();
        for (ShootingConstants.ShootingPosition position : ShootingConstants.SHOOTING_POSITIONS)
            points.add(new LinearInterpolation.Point(position.distanceMeters(), position.shooterVelocityRotationsPerSecond()));
        return new LinearInterpolation(points);
    }
}
