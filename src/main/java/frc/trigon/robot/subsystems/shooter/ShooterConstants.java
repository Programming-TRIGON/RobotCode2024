package frc.trigon.robot.subsystems.shooter;

import frc.trigon.robot.constants.ShootingConstants;
import frc.trigon.robot.utilities.LinearInterpolation;
import frc.trigon.robot.utilities.SpeedMechanism2d;

import java.util.ArrayList;

public class ShooterConstants {
    public static final double SHOOTER_GEAR_RATIO = 1;
    static final double TOP_TO_BOTTOM_SHOOTING_RATIO = 1;

    static final double TOLERANCE_REVOLUTIONS = 0.1;
    static final LinearInterpolation VELOCITY_INTERPOLATION = generateInterpolation();

    private static final double MAX_DISPLAYABLE_VELOCITY = 10;
    static final SpeedMechanism2d
            TOP_SHOOTING_MECHANISM = new SpeedMechanism2d("Mechanisms/TopShooterMechanism", MAX_DISPLAYABLE_VELOCITY),
            BOTTOM_SHOOTING_MECHANISM = new SpeedMechanism2d("Mechanisms/BottomShooterMechanism", MAX_DISPLAYABLE_VELOCITY);

    private static LinearInterpolation generateInterpolation() {
        final ArrayList<LinearInterpolation.Point> points = new ArrayList<>();
        for (ShootingConstants.ShootingPosition position : ShootingConstants.SHOOTING_POSITIONS)
            points.add(new LinearInterpolation.Point(position.distanceMeters(), position.shooterVelocityRevolutionsPerSecond()));
        return new LinearInterpolation(points);
    }
}
