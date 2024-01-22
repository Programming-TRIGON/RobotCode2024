package frc.trigon.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.constants.ShootingConstants;
import frc.trigon.robot.subsystems.swerve.Swerve;

import java.util.ArrayList;

public class ShootingCalculations {
    private static final LinearInterpolation
            DISTANCE_OFFSET_INTERPOLATION = ShootingConstants.DISTANCE_OFFSET_INTERPOLATION,
            SHOOTING_VELOCITY_INTERPOLATION = generateShootingVelocityInterpolation(),
            PITCH_INTERPOLATION = generatePitchInterpolation(),
            AIR_TIME_INTERPOLATION = generateAirTimeInterpolation();
    private static final Swerve SWERVE = Swerve.getInstance();

    /**
     * @return the velocity the top shooting motor should reach in order to shoot to the speaker, in revolutions per second
     */
    public static double calculateTargetTopShootingVelocity() {
        return SHOOTING_VELOCITY_INTERPOLATION.predict(getOffsettedDistanceToSpeakerAfterMovement());
    }

    /**
     * @return the pitch the pitcher should reach in order to shoot to the speaker
     */
    public static Rotation2d calculateTargetPitch() {
        return Rotation2d.fromDegrees(PITCH_INTERPOLATION.predict(getOffsettedDistanceToSpeakerAfterMovement()));
    }

    /**
     * @return the angle (yaw) the robot should reach in order to shoot to the speaker
     */
    public static Rotation2d calculateTargetRobotAngle() {
        final Translation2d speakerPositionAfterMovement = getSpeakerPositionAfterMovement();
        final double distanceOffset = getDistanceOffsetToSpeaker(speakerPositionAfterMovement);
        return getAngleToSpeaker(distanceOffset, speakerPositionAfterMovement);
    }

    /**
     * @return {@link ShootingCalculations#getDistanceOffsetToSpeaker} with a parameter of {@link ShootingCalculations#getSpeakerPositionAfterMovement}
     */
    private static double getOffsettedDistanceToSpeakerAfterMovement() {
        final Translation2d speakerPositionAfterMovement = getSpeakerPositionAfterMovement();
        return getOffsettedDistanceToSpeaker(speakerPositionAfterMovement);
    }

    /**
     * The offsetted distance from the speaker to the robot. This is just {@link ShootingCalculations#getDistanceOffsetToSpeaker} + the real distance norm from the speaker.
     *
     * @param speakerPosition the speaker's position. This won't always be the static position, since sometimes we would want to shoot to the speaker while driving, see {@link ShootingCalculations#getSpeakerPositionAfterMovement}
     * @return the offsetted distance from to the speaker
     */
    private static double getOffsettedDistanceToSpeaker(Translation2d speakerPosition) {
        final Pose2d mirroredAlliancePose = RobotContainer.POSE_ESTIMATOR.getCurrentPose().toMirroredAlliancePose();
        final double distanceOffset = getDistanceOffsetToSpeaker(speakerPosition);
        return mirroredAlliancePose.getTranslation().getDistance(speakerPosition) + distanceOffset;
    }

    /**
     * Calculate how much we should aim away from the middle of the shooter, since we won't want to shoot to the middle of the speaker when shooting from the side with an angle.
     *
     * @param speakerPosition the speaker's position. This won't always be the static position, since sometimes we would want to shoot to the speaker while driving, see {@link ShootingCalculations#getSpeakerPositionAfterMovement}
     * @return the distance offset in meters
     */
    private static double getDistanceOffsetToSpeaker(Translation2d speakerPosition) {
        final Rotation2d angleToMiddleOfSpeaker = getAngleToSpeaker(0, speakerPosition);
        final int signum = (int) Math.signum(angleToMiddleOfSpeaker.getDegrees());
        return ShootingConstants.DISTANCE_OFFSET_INTERPOLATION.predict(angleToMiddleOfSpeaker.getDegrees() * signum) * signum;
    }

    /**
     * Use's {@linkplain java.lang.Math#atan2} to calculate the angle we should face in order to aim at the speaker.
     *
     * @param yOffset         an offset from the speaker's middle, since we won't always want to shoot to the middle of the speaker, see {@link ShootingCalculations#getDistanceOffsetToSpeaker}
     * @param speakerPosition the speaker's position. This won't always be the static position, since sometimes we would want to shoot to the speaker while driving, see {@link ShootingCalculations#getSpeakerPositionAfterMovement}
     * @return the angle we should face in order to aim at the speaker
     */
    private static Rotation2d getAngleToSpeaker(double yOffset, Translation2d speakerPosition) {
        final Pose2d mirroredAlliancePose = RobotContainer.POSE_ESTIMATOR.getCurrentPose().toMirroredAlliancePose();
        final Translation2d difference = mirroredAlliancePose.getTranslation().minus(speakerPosition.plus(new Translation2d(0, yOffset)));
        return Rotation2d.fromRadians(Math.atan2(difference.getY(), difference.getX()));
    }

    /**
     * @return the speaker's position after the game piece will fly in the air. This is useful for shooting while driving
     */
    private static Translation2d getSpeakerPositionAfterMovement() {
        final Translation2d fieldRelativeVelocity = getFieldRelativeVelocity();
        final double airTimeSeconds = AIR_TIME_INTERPOLATION.predict(getOffsettedDistanceToSpeaker(FieldConstants.SPEAKER_TRANSLATION));
        return FieldConstants.SPEAKER_TRANSLATION.minus(fieldRelativeVelocity.times(airTimeSeconds));
    }

    private static Translation2d getFieldRelativeVelocity() {
        final ChassisSpeeds fieldRelativeSpeeds = SWERVE.getFieldRelativeVelocity();
        return new Translation2d(fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond);
    }

    private static LinearInterpolation generateAirTimeInterpolation() {
        final ArrayList<LinearInterpolation.Point> points = new ArrayList<>();
        for (ShootingConstants.ShootingPosition position : ShootingConstants.SHOOTING_POSITIONS)
            points.add(new LinearInterpolation.Point(position.distanceMeters(), position.timeInAirSeconds()));
        return new LinearInterpolation(points);
    }

    private static LinearInterpolation generatePitchInterpolation() {
        final ArrayList<LinearInterpolation.Point> points = new ArrayList<>();
        for (ShootingConstants.ShootingPosition position : ShootingConstants.SHOOTING_POSITIONS)
            points.add(new LinearInterpolation.Point(position.distanceMeters(), position.pitch().getRotations()));
        return new LinearInterpolation(points);
    }

    private static LinearInterpolation generateShootingVelocityInterpolation() {
        final ArrayList<LinearInterpolation.Point> points = new ArrayList<>();
        for (ShootingConstants.ShootingPosition position : ShootingConstants.SHOOTING_POSITIONS)
            points.add(new LinearInterpolation.Point(position.distanceMeters(), position.shooterVelocityRevolutionsPerSecond()));
        return new LinearInterpolation(points);
    }
}
