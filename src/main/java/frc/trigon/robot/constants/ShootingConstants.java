package frc.trigon.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.utilities.LinearInterpolation;

public class ShootingConstants {
    public static final LinearInterpolation DISTANCE_OFFSET_INTERPOLATION = new LinearInterpolation(

    );
    public static final ShootingPosition[] SHOOTING_POSITIONS = {

    };

    /**
     * A record to represent a shooting position / waypoint.
     *
     * @param distanceMeters                      the distance from the speaker in meters
     * @param shooterVelocityRevolutionsPerSecond the shooter velocity in revolutions per second
     * @param pitch                               the pitcher pitch
     */
    public record ShootingPosition(double distanceMeters, double shooterVelocityRevolutionsPerSecond,
                                   Rotation2d pitch) {
    }

    /**
     * A record to represent the target position in the speaker to shoot to
     * since we won't always want to shoot to the middle of the shooter.
     *
     * @param distanceToSpeaker the distance to the target position in the speaker
     * @param angleToSpeaker    the angle to the target position in the speaker
     */
    public record ShootingTarget(double distanceToSpeaker, Rotation2d angleToSpeaker) {
    }
}
