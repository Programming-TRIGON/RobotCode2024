package frc.trigon.robot.utilities;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.constants.ShootingConstants;
import frc.trigon.robot.subsystems.shooter.ShooterConstants;
import org.littletonrobotics.junction.AutoLogOutput;

import java.util.ArrayList;

public class ShootingCalculations {
    private static ShootingCalculations INSTANCE = null;

    private final LinearInterpolation
            SHOOTING_VELOCITY_INTERPOLATION = generateShootingVelocityInterpolation(),
            PITCH_INTERPOLATION = generatePitchInterpolation();
    private Translation2d predictedTranslation = new Translation2d();
    private double distanceFromSpeaker = 0;

    public static ShootingCalculations getInstance() {
        if (INSTANCE == null)
            INSTANCE = new ShootingCalculations();
        return INSTANCE;
    }

    private ShootingCalculations() {
    }

    public void updateCalculations() {
        predictedTranslation = predictFutureTranslation();
        distanceFromSpeaker = getDistanceFromSpeaker(predictedTranslation);
    }

    /**
     * @return the velocity the shooting motor should reach in order to shoot to the speaker, in revolutions per second
     */
    public double calculateTargetShootingVelocity() {
        return -SHOOTING_VELOCITY_INTERPOLATION.predict(distanceFromSpeaker);
    }

    /**
     * @return the pitch the pitcher should reach in order to shoot to the speaker
     */
    public Rotation2d calculateTargetPitch() {
        return Rotation2d.fromRotations(PITCH_INTERPOLATION.predict(distanceFromSpeaker));
    }

    public Rotation2d calculateTargetPitchUsingProjectileMotion() {
        final double noteTangentialVelocity = angularVelocityToTangentialVelocity(calculateTargetShootingVelocity());
        final Pose3d shooterEndEffectorPose = calculateShooterEndEffectorFieldRelativePose();
        final double shooterEndEffectorDistanceFromSpeaker = shooterEndEffectorPose.getTranslation().toTranslation2d().getDistance(FieldConstants.SPEAKER_TRANSLATION.toTranslation2d());
        return calculateTargetPitchUsingProjectileMotion(shooterEndEffectorDistanceFromSpeaker, noteTangentialVelocity, shooterEndEffectorPose.getZ());
    }

    /**
     * Calculates the pitch the robot should reach in order to shoot to the speaker using projectile motion.
     * This uses the formula stated here: <a href="https://en.wikipedia.org/wiki/Projectile_motion#Angle_%CE%B8_required_to_hit_coordinate_(x,_y)">...</a>
     *
     * @param noteTangentialVelocity                the tangential velocity of the note
     * @param shooterEndEffectorDistanceFromSpeaker the distance from the speaker to the shooter's end effector on the xy plane
     * @param shooterEndEffectorHeight              the height of the shooter's end effector
     * @return the pitch the robot should reach in order to shoot to the speaker
     */
    private Rotation2d calculateTargetPitchUsingProjectileMotion(double noteTangentialVelocity, double shooterEndEffectorDistanceFromSpeaker, double shooterEndEffectorHeight) {
        final double gForce = ShootingConstants.G_FORCE;
        final double velocitySquared = Math.pow(noteTangentialVelocity, 2);
        final double velocity4thPower = Math.pow(velocitySquared, 2);
        final double distanceSquared = Math.pow(shooterEndEffectorDistanceFromSpeaker, 2);
        final double heightDifference = FieldConstants.SPEAKER_TRANSLATION.getZ() - shooterEndEffectorHeight;
        final double targetAngleRadians = Math.atan(
                velocitySquared - Math.sqrt(velocity4thPower - gForce * (gForce * distanceSquared + 2 * velocitySquared * heightDifference))
                        / (gForce * shooterEndEffectorDistanceFromSpeaker)
        );
        return Rotation2d.fromRadians(targetAngleRadians);
    }

    private Pose3d calculateShooterEndEffectorFieldRelativePose() {
        final Pose3d predictedPose = new Pose3d(new Pose2d(this.predictedTranslation, calculateTargetRobotAngle()));
        final Pose3d shooterPivotFieldRelative = predictedPose.plus(ShooterConstants.ROBOT_TO_PIVOT_POINT);
        final Transform3d pivotToEndEffector = new Transform3d(ShooterConstants.SHOOTER_LENGTH_METERS, 0, 0, new Rotation3d(0, RobotContainer.PITCHER.getTargetPitch().getRadians(), 0));
        return shooterPivotFieldRelative.plus(pivotToEndEffector);
    }

    private double angularVelocityToTangentialVelocity(double angularVelocity) {
        return angularVelocity * ShooterConstants.REVOLUTIONS_TO_METERS;
    }

    /**
     * @return the angle (yaw) the robot should reach in order to shoot to the speaker
     */
    public Rotation2d calculateTargetRobotAngle() {
        return getAngleToSpeaker(predictedTranslation);
    }

    @AutoLogOutput(key = "DistanceFromSpeaker")
    public double getDistanceFromSpeaker() {
        return distanceFromSpeaker;
    }

    /**
     * Calculates the distance from the speaker to the robot.
     *
     * @param predictedPose the predicted pose of the robot
     * @return the distance from to the speaker
     */
    private double getDistanceFromSpeaker(Translation2d predictedPose) {
        return predictedPose.getDistance(FieldConstants.SPEAKER_TRANSLATION.toTranslation2d());
    }

    /**
     * Uses {@linkplain java.lang.Math#atan2} to calculate the angle we should face in order to aim at the speaker.
     *
     * @param predictedPose the predicted pose of the robot
     * @return the angle the robot should face in order to aim at the speaker
     */
    private Rotation2d getAngleToSpeaker(Translation2d predictedPose) {
        final Translation2d difference = predictedPose.minus(FieldConstants.SPEAKER_TRANSLATION.toTranslation2d());
        return Rotation2d.fromRadians(Math.atan2(difference.getY(), difference.getX()));
    }

    private Translation2d predictFutureTranslation() {
        final Translation2d fieldRelativeVelocity = getFieldRelativeVelocity();
        final Pose2d currentMirroredTranslation = RobotContainer.POSE_ESTIMATOR.getCurrentPose().toMirroredAlliancePose();
        final Pose2d predictedPose = currentMirroredTranslation.transformBy(new Transform2d(fieldRelativeVelocity.times(ShootingConstants.POSE_PREDICTING_TIME), Rotation2d.fromDegrees(0)));
        return predictedPose.getTranslation();
    }

    private Translation2d getFieldRelativeVelocity() {
        final ChassisSpeeds fieldRelativeSpeeds = RobotContainer.SWERVE.getFieldRelativeVelocity();
        return new Translation2d(fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond);
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