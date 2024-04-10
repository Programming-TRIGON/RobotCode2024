package frc.trigon.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class FieldConstants {
    public static final double
            FIELD_LENGTH_METERS = 16.54175,
            FIELD_WIDTH_METERS = 8.21;
    public static Translation2d SPEAKER_TRANSLATION = new Translation2d(0.085, 5.547);
    public static Pose2d IN_FRONT_OF_AMP_POSE = new Pose2d(1.842, 8.204 - 0.48, Rotation2d.fromDegrees(90));
}
