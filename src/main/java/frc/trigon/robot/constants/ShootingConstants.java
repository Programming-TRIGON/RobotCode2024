package frc.trigon.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.utilities.LinearInterpolation;

public class ShootingConstants {
    /**
     * A linear interpolation to get the distance offset to the speaker.
     * This is the offset from the center of the speaker to where we should shoot to.
     * The y value is the atan2 angle to the speaker in degrees.
     * The x value is the distance offset in meters, to give us the position to shoot to on the speaker.
     */
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
                                   Rotation2d pitch, double timeInAirSeconds) {
    }
}
