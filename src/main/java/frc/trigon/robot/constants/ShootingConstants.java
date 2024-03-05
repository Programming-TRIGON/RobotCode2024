package frc.trigon.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;

public class ShootingConstants {
    public static final ShootingPosition[] SHOOTING_POSITIONS = {
            new ShootingPosition(1.425145, 80, Rotation2d.fromDegrees(60), 0),
            new ShootingPosition(1.618750, 80, Rotation2d.fromDegrees(58), 0),
            new ShootingPosition(1.857015, 80, Rotation2d.fromDegrees(54), 0),
            new ShootingPosition(2.039561, 80, Rotation2d.fromDegrees(52), 0),
            new ShootingPosition(2.263245, 80, Rotation2d.fromDegrees(49), 0),
            new ShootingPosition(2.463506, 80, Rotation2d.fromDegrees(47), 0),
            new ShootingPosition(2.643892, 80, Rotation2d.fromDegrees(43), 0),
            new ShootingPosition(2.823944, 80, Rotation2d.fromDegrees(41), 0),
            new ShootingPosition(2.987751, 80, Rotation2d.fromDegrees(39), 0),
            new ShootingPosition(3.206550, 80, Rotation2d.fromDegrees(38), 0),
            new ShootingPosition(3.393300, 80, Rotation2d.fromDegrees(37), 0),
            new ShootingPosition(3.577490, 80, Rotation2d.fromDegrees(36), 0),
            new ShootingPosition(3.801666, 80, Rotation2d.fromDegrees(35), 0),
            new ShootingPosition(3.994704, 80, Rotation2d.fromDegrees(34), 0),
            new ShootingPosition(4.201635, 80, Rotation2d.fromDegrees(33), 0),
            new ShootingPosition(4.384726, 80, Rotation2d.fromDegrees(32.5), 0),
            new ShootingPosition(4.588776, 80, Rotation2d.fromDegrees(32), 0),
            new ShootingPosition(4.814911, 80, Rotation2d.fromDegrees(32), 0),
            new ShootingPosition(5.010912, 80, Rotation2d.fromDegrees(33), 0),
    };

    public static final int SHOOTING_VELOCITY_DISTANCE_CHECKS = 0;

    public static final double CLOSE_SHOT_VELOCITY_METERS_PER_SECOND = -80;
    public static final Rotation2d CLOSE_SHOT_ANGLE = Rotation2d.fromDegrees(58);

    /**
     * A record to represent a shooting position / waypoint.
     *
     * @param distanceMeters                      the distance from the speaker in meters
     * @param shooterVelocityRevolutionsPerSecond the shooter velocity in revolutions per second
     * @param pitch                               the pitcher pitch
     * @param timeInAirSeconds                    the time in the air in seconds
     */
    public record ShootingPosition(double distanceMeters, double shooterVelocityRevolutionsPerSecond,
                                   Rotation2d pitch, double timeInAirSeconds) {
        public String toString() {
            return ("ShootingPosition{" +
                    "Distance as Meters: " + distanceMeters +
                    ", Shooter Velocity as Revolutions Per Second: " + shooterVelocityRevolutionsPerSecond +
                    ", Pitch as Degrees: " + pitch.getDegrees() +
                    ", Time in the Air as Seconds: " + timeInAirSeconds +
                    "}"
            );
        }
    }
}
