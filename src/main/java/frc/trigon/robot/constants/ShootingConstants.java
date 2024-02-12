package frc.trigon.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.utilities.LinearInterpolation;

public class ShootingConstants {
    /**
     * A linear interpolation to get the y offset to the speaker.
     * This is the offset from the center of the speaker to where we should shoot to.
     * The y value is the atan2 angle to the speaker in degrees.
     * The x value is the distance offset on the y-axis in meters, to give us the position to shoot to on the speaker.
     */
    public static final LinearInterpolation Y_OFFSET_INTERPOLATION = new LinearInterpolation(

    );

    public static final ShootingPosition[] SHOOTING_POSITIONS = generateShootingPositionAsAtan2();
//            new ShootingPosition(0.2, 30, Rotation2d.fromDegrees(60), 0.01),
//            new ShootingPosition(2, 30, Rotation2d.fromDegrees(40), 0.02),
//            new ShootingPosition(3, 30, Rotation2d.fromDegrees(30), 0.03),
//            new ShootingPosition(4, 30, Rotation2d.fromDegrees(20), 0.04),
//            new ShootingPosition(5, 30, Rotation2d.fromDegrees(17), 0.05),
//    };

    private static ShootingPosition[] generateShootingPositionAsAtan2() {
        ShootingPosition[] shootingPositions = new ShootingPosition[80];

        for (int i = 0; i < 80; i += 1) {
            double xDistance = Math.abs(i * 0.1 - FieldConstants.SPEAKER_TRANSLATION.getX());
            double velocity = 30;
            double speakerZ = 1.984;
            double pitchFromAtan2 = Math.atan2(speakerZ, xDistance);
            shootingPositions[i] = new ShootingPosition(xDistance, velocity, Rotation2d.fromRadians(pitchFromAtan2), 0);
        }

        return shootingPositions;
    }

    public static final int SHOOTING_VELOCITY_DISTANCE_CHECKS = 10;

    public static final double CLOSE_SHOT_VELOCITY_METERS_PER_SECOND = 30;
    public static final Rotation2d CLOSE_SHOT_ANGLE = Rotation2d.fromDegrees(50);

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
