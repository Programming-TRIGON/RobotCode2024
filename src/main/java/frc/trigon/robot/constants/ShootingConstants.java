package frc.trigon.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;

public class ShootingConstants {
    public static final ShootingPosition[] SHOOTING_POSITIONS = {
            new ShootingPosition(1.287678, 80, Rotation2d.fromDegrees(62)),
            new ShootingPosition(1.439430, 80, Rotation2d.fromDegrees(60)),
            new ShootingPosition(1.645260, 80, Rotation2d.fromDegrees(57)),
            new ShootingPosition(1.841616, 80, Rotation2d.fromDegrees(54)),
            new ShootingPosition(2.035879, 80, Rotation2d.fromDegrees(52)),
            new ShootingPosition(2.243164, 80, Rotation2d.fromDegrees(46)),
            new ShootingPosition(2.456622, 80, Rotation2d.fromDegrees(44)),
            new ShootingPosition(2.644851, 80, Rotation2d.fromDegrees(42)),
            new ShootingPosition(2.834507, 80, Rotation2d.fromDegrees(40)),
            new ShootingPosition(3.039509, 80, Rotation2d.fromDegrees(38)),
            new ShootingPosition(3.230490, 80, Rotation2d.fromDegrees(37)),
            new ShootingPosition(3.449656, 80, Rotation2d.fromDegrees(35)),
            new ShootingPosition(3.624291, 80, Rotation2d.fromDegrees(34.5)),
            new ShootingPosition(3.820556, 80, Rotation2d.fromDegrees(33)),
            new ShootingPosition(4.046602, 80, Rotation2d.fromDegrees(33)),
            new ShootingPosition(4.242297, 80, Rotation2d.fromDegrees(31.5)),
            new ShootingPosition(4.450438, 80, Rotation2d.fromDegrees(30.5)),
            new ShootingPosition(4.628484, 80, Rotation2d.fromDegrees(30)),
            new ShootingPosition(4.908269, 80, Rotation2d.fromDegrees(29))
//            new ShootingPosition(0, 80, Rotation2d.fromDegrees(0)),
    };
    public static final double POSE_PREDICTING_TIME = 0.1;
    public static final double G_FORCE = 9.8;

    public static final double CLOSE_SHOT_VELOCITY_METERS_PER_SECOND = -80;
    public static final Rotation2d CLOSE_SHOT_ANGLE = Rotation2d.fromDegrees(58);

    /**
     * A record to represent a shooting position / waypoint.
     *
     * @param distanceMeters                      the distance from the speaker in meters
     * @param shooterVelocityRevolutionsPerSecond the shooter velocity in revolutions per second
     * @param pitch                               the pitcher pitch
     */
    public record ShootingPosition(double distanceMeters, double shooterVelocityRevolutionsPerSecond,
                                   Rotation2d pitch) {
        public String toString() {
            return ("ShootingPosition{" +
                    "Distance as Meters: " + distanceMeters +
                    ", Shooter Velocity as Revolutions Per Second: " + shooterVelocityRevolutionsPerSecond +
                    ", Pitch as Degrees: " + pitch.getDegrees() +
                    "}"
            );
        }
    }
}
