package frc.trigon.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.constants.ShootingConstants;

import java.util.ArrayList;

public class ShootingCalculations {
    private static ShootingCalculations INSTANCE = null;

    private final LinearInterpolation
            Y_OFFSET_INTERPOLATION = ShootingConstants.Y_OFFSET_INTERPOLATION,
            SHOOTING_VELOCITY_INTERPOLATION = generateShootingVelocityInterpolation(),
            PITCH_INTERPOLATION = generatePitchInterpolation(),
            AIR_TIME_INTERPOLATION = generateAirTimeInterpolation();
    private Translation2d speakerPositionAfterMovement = FieldConstants.SPEAKER_TRANSLATION;
    private double offsettedDistanceToSpeakerAfterMovement = 0;

    public static ShootingCalculations getInstance() {
        if (INSTANCE == null)
            INSTANCE = new ShootingCalculations();
        return INSTANCE;
    }

    private ShootingCalculations() {
    }

    public void updateCalculations() {
        speakerPositionAfterMovement = getSpeakerPositionAfterMovement();
        offsettedDistanceToSpeakerAfterMovement = getOffsettedDistanceToSpeaker(speakerPositionAfterMovement);
    }

    /**
     * @return the velocity the top shooting motor should reach in order to shoot to the speaker, in revolutions per second
     */
    public double calculateTargetTopShootingVelocity() {
        return SHOOTING_VELOCITY_INTERPOLATION.predict(offsettedDistanceToSpeakerAfterMovement);
    }

    /**
     * @return the pitch the pitcher should reach in order to shoot to the speaker
     */
    public Rotation2d calculateTargetPitch() {
        return Rotation2d.fromRotations(PITCH_INTERPOLATION.predict(offsettedDistanceToSpeakerAfterMovement));
    }

    /**
     * @return the angle (yaw) the robot should reach in order to shoot to the speaker
     */
    public Rotation2d calculateTargetRobotAngle() {
        final double yOffsetToSpeaker = getYOffsetToSpeaker(speakerPositionAfterMovement);
        return getAngleToSpeaker(yOffsetToSpeaker, speakerPositionAfterMovement);
    }

    /**
     * The offsetted distance from the speaker to the robot. This is just {@link ShootingCalculations#getYOffsetToSpeaker} + the real distance norm from the speaker.
     *
     * @param speakerPosition the speaker's position. This won't always be the static position, since sometimes we would want to shoot to the speaker while driving, see {@link ShootingCalculations#getSpeakerPositionAfterMovement}
     * @return the offsetted distance from to the speaker
     */
    private double getOffsettedDistanceToSpeaker(Translation2d speakerPosition) {
        final Pose2d mirroredAlliancePose = RobotContainer.POSE_ESTIMATOR.getCurrentPose().toMirroredAlliancePose();
        final double distanceOffset = getYOffsetToSpeaker(speakerPosition);
        return mirroredAlliancePose.getTranslation().getDistance(speakerPosition.plus(new Translation2d(0, distanceOffset)));
    }

    /**
     * Calculate how much we should aim away from the middle of the shooter on the y-axis, since we won't want to shoot to the middle of the speaker when shooting from the side with an angle.
     *
     * @param speakerPosition the speaker's position. This won't always be the static position, since sometimes we would want to shoot to the speaker while driving, see {@link ShootingCalculations#getSpeakerPositionAfterMovement}
     * @return the y-axis offset in meters
     */
    private double getYOffsetToSpeaker(Translation2d speakerPosition) {
        final Rotation2d angleToMiddleOfSpeaker = getAngleToSpeaker(0, speakerPosition);
        final int signum = (int) Math.signum(angleToMiddleOfSpeaker.getDegrees());
        return Y_OFFSET_INTERPOLATION.predict(angleToMiddleOfSpeaker.getDegrees() * signum) * signum;
    }

    /**
     * Use's {@linkplain java.lang.Math#atan2} to calculate the angle we should face in order to aim at the speaker.
     *
     * @param yOffset         an offset from the speaker's middle, since we won't always want to shoot to the middle of the speaker, see {@link ShootingCalculations#getYOffsetToSpeaker}
     * @param speakerPosition the speaker's position. This won't always be the static position, since sometimes we would want to shoot to the speaker while driving, see {@link ShootingCalculations#getSpeakerPositionAfterMovement}
     * @return the angle we should face in order to aim at the speaker
     */
    private Rotation2d getAngleToSpeaker(double yOffset, Translation2d speakerPosition) {
        final Pose2d mirroredAlliancePose = RobotContainer.POSE_ESTIMATOR.getCurrentPose().toMirroredAlliancePose();
        final Translation2d difference = mirroredAlliancePose.getTranslation().minus(speakerPosition.plus(new Translation2d(0, yOffset)));
        return Rotation2d.fromRadians(Math.atan2(difference.getY(), difference.getX()));
    }

    /**
     * @return the speaker's position after the game piece will fly in the air. This is useful for shooting while driving
     */
    private Translation2d getSpeakerPositionAfterMovement() {
        Translation2d lastSpeakerPosition = FieldConstants.SPEAKER_TRANSLATION;
        for (int i = 0; i < ShootingConstants.SHOOTING_VELOCITY_DISTANCE_CHECKS; i++) {
            final Translation2d fieldRelativeVelocity = getFieldRelativeVelocity();
            final double airTimeSeconds = AIR_TIME_INTERPOLATION.predict(getOffsettedDistanceToSpeaker(FieldConstants.SPEAKER_TRANSLATION));
            lastSpeakerPosition = FieldConstants.SPEAKER_TRANSLATION.minus(fieldRelativeVelocity.times(airTimeSeconds));
        }
        return lastSpeakerPosition;
    }

    private Translation2d getFieldRelativeVelocity() {
        final ChassisSpeeds fieldRelativeSpeeds = RobotContainer.SWERVE.getFieldRelativeVelocity();
        return new Translation2d(fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond);
    }

    private LinearInterpolation generateAirTimeInterpolation() {
        final ArrayList<LinearInterpolation.Point> points = new ArrayList<>();
        for (ShootingConstants.ShootingPosition position : ShootingConstants.SHOOTING_POSITIONS)
            points.add(new LinearInterpolation.Point(position.distanceMeters(), position.timeInAirSeconds()));
        return new LinearInterpolation(points);
    }

    private LinearInterpolation generatePitchInterpolation() {
        final ArrayList<LinearInterpolation.Point> points = new ArrayList<>();
        for (ShootingConstants.ShootingPosition position : ShootingConstants.SHOOTING_POSITIONS)
            points.add(new LinearInterpolation.Point(position.distanceMeters(), position.pitch().getRotations()));
        return new LinearInterpolation(points);
    }

    private LinearInterpolation generateShootingVelocityInterpolation() {
        final ArrayList<LinearInterpolation.Point> points = new ArrayList<>();
        for (ShootingConstants.ShootingPosition position : ShootingConstants.SHOOTING_POSITIONS)
            points.add(new LinearInterpolation.Point(position.distanceMeters(), position.shooterVelocityRevolutionsPerSecond()));
        return new LinearInterpolation(points);
    }
}
