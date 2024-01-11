package frc.trigon.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;

public class ShootingConstants {
    public static final ShootingPosition[] SHOOTING_POSITIONS = {
    };

    public record ShootingPosition(double distanceMeters, double shooterVelocityRotationsPerSecond, Rotation2d pitch) {
    }
}
