package frc.trigon.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;

public class ShootingConstants {
    public static final ShootingPosition[] SHOOTING_POSITIONS = {
            new ShootingPosition(1.159017, 80, Rotation2d.fromDegrees(56), 0),
            new ShootingPosition(1.372836, 80, Rotation2d.fromDegrees(54.000000), 0),
            new ShootingPosition(1.560613, 80, Rotation2d.fromDegrees(53), 0),
            new ShootingPosition(1.742136, 80, Rotation2d.fromDegrees(50), 0),
            new ShootingPosition(1.971171, 80, Rotation2d.fromDegrees(48.5), 0),
            new ShootingPosition(2.196812, 80, Rotation2d.fromDegrees(45), 0),
            new ShootingPosition(2.362655, 80, Rotation2d.fromDegrees(43), 0),
            new ShootingPosition(2.508624, 80, Rotation2d.fromDegrees(42), 0),
            new ShootingPosition(2.644452, 80, Rotation2d.fromDegrees(40), 0),
            new ShootingPosition(2.695863, 80, Rotation2d.fromDegrees(39), 0),
            new ShootingPosition(2.958307, 80, Rotation2d.fromDegrees(38.000000), 0),
            new ShootingPosition(3.130263, 80, Rotation2d.fromDegrees(37), 0),
            new ShootingPosition(3.306207, 80, Rotation2d.fromDegrees(36), 0),
            new ShootingPosition(3.541305, 80, Rotation2d.fromDegrees(34), 0),
            new ShootingPosition(3.727267, 80, Rotation2d.fromDegrees(33), 0),
            new ShootingPosition(3.943222, 80, Rotation2d.fromDegrees(32), 0),
            new ShootingPosition(4.170544, 80, Rotation2d.fromDegrees(31), 0),
            new ShootingPosition(4.456653, 80, Rotation2d.fromDegrees(30), 0),
    };

    private static ShootingPosition[] generateShootingPositionAsAtan2() {
        ShootingPosition[] shootingPositions = new ShootingPosition[80];

        for (int i = 0; i < 80; i += 1) {
            double xDistance = Math.abs(i * 0.1 - FieldConstants.SPEAKER_TRANSLATION.getX());
            double velocity = -50;
            double speakerZ = 2.5;
            double pitchFromAtan2 = Math.atan2(speakerZ, xDistance);
            shootingPositions[i] = new ShootingPosition(xDistance, velocity, Rotation2d.fromRadians(pitchFromAtan2), 0);
        }


        return shootingPositions;
    }

    public static final int SHOOTING_VELOCITY_DISTANCE_CHECKS = 1;

    public static final double CLOSE_SHOT_VELOCITY_METERS_PER_SECOND = -40;
    public static final Rotation2d CLOSE_SHOT_ANGLE = Rotation2d.fromDegrees(60);

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
