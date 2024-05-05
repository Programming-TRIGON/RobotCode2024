package frc.trigon.robot.utilities;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.constants.ShootingConstants;
import frc.trigon.robot.subsystems.shooter.ShooterConstants;
import frc.trigon.robot.utilities.mirrorable.MirrorableRotation2d;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShootingCalculations {
    private static ShootingCalculations INSTANCE = null;
    private Translation2d predictedTranslation = new Translation2d();
    @AutoLogOutput(key = "ShootingCalculations/EndEffectorDistanceFromSpeaker")
    private double previousEndEffectorDistanceFromSpeaker = 0;

    public static ShootingCalculations getInstance() {
        if (INSTANCE == null)
            INSTANCE = new ShootingCalculations();
        return INSTANCE;
    }

    private ShootingCalculations() {
    }

    public void updateCalculations() {
        predictedTranslation = predictFutureTranslation(calculateTimeInAir());
    }

    /**
     * @return the velocity the shooting motor should reach in order to shoot to the speaker, in revolutions per second
     */
    public double calculateTargetShootingVelocity() {
        return ShootingConstants.SHOOTING_VELOCITY_REVOLUTIONS_PER_SECOND;
    }

    /**
     * @return the angle (yaw) the robot should reach in order to face the speaker
     */
    public MirrorableRotation2d calculateTargetRobotAngle() {
        return getAngleToSpeaker(predictedTranslation);
    }

    /**
     * @return the pitch the pitcher should reach in order to shoot to the speaker. This is calculated using projectile motion.
     */
    public Rotation2d calculateTargetPitch() {
        final double noteTangentialVelocity = angularVelocityToTangentialVelocity(calculateTargetShootingVelocity());
        final Pose3d shooterEndEffectorPose = calculateShooterEndEffectorFieldRelativePose();
        final double shooterEndEffectorDistanceFromSpeaker = shooterEndEffectorPose.getTranslation().toTranslation2d().getDistance(FieldConstants.SPEAKER_TRANSLATION.get().toTranslation2d());
        previousEndEffectorDistanceFromSpeaker = shooterEndEffectorDistanceFromSpeaker;
        return calculateTargetPitchUsingProjectileMotion(noteTangentialVelocity, shooterEndEffectorDistanceFromSpeaker, shooterEndEffectorPose.getZ());
    }

    /**
     * Calculates the pitch the pitcher should reach in order to shoot to the speaker using projectile motion.
     * This will fully calculate the target pitch using physics.
     *
     * @param noteTangentialVelocity                the tangential velocity of the shooter
     * @param shooterEndEffectorDistanceFromSpeaker the distance from the speaker to the shooter's end effector on the xy plane
     * @param shooterEndEffectorHeight              the height of the shooter's end effector
     * @return the pitch the robot should reach in order to shoot to the speaker
     * @link <a href="https://en.wikipedia.org/wiki/Projectile_motion#Angle_%CE%B8_required_to_hit_coordinate_(x,_y)">Projectile Motion</a>
     */
    private Rotation2d calculateTargetPitchUsingProjectileMotion(double noteTangentialVelocity, double shooterEndEffectorDistanceFromSpeaker, double shooterEndEffectorHeight) {
        final double gForce = ShootingConstants.G_FORCE;
        final double velocitySquared = Math.pow(noteTangentialVelocity, 2);
        final double velocity4thPower = Math.pow(noteTangentialVelocity, 4);
        final double distanceSquared = Math.pow(shooterEndEffectorDistanceFromSpeaker, 2);
        final double heightDifference = FieldConstants.SPEAKER_TRANSLATION.get().getZ() - shooterEndEffectorHeight;
        final double squareRoot = Math.sqrt(
                velocity4thPower - (gForce * ((gForce * distanceSquared) + (2 * velocitySquared * heightDifference)))
        );
        final double numerator = velocitySquared - squareRoot;
        final double denominator = gForce * shooterEndEffectorDistanceFromSpeaker;
        final double fraction = numerator / denominator;
        return Rotation2d.fromRadians(Math.atan(fraction));
    }

    /**
     * Calculates the shooter's end effector's 3d pose on the field.
     * The end effector is the furthest point of the shooter from the pivot point,
     * and where the note leaves the shooter.
     *
     * @return the shooter's end effector's 3d pose on the field
     */
    private Pose3d calculateShooterEndEffectorFieldRelativePose() {
        final Pose3d endEffectorSelfRelativePose = calculateShooterEndEffectorSelfRelativePose();
        final Transform3d robotToEndEffector = endEffectorSelfRelativePose.minus(new Pose3d());
        final Pose3d predictedPose = new Pose3d(new Pose2d(this.predictedTranslation, calculateTargetRobotAngle().get()));
        return predictedPose.transformBy(robotToEndEffector);
    }

    /**
     * Calculates the shooter's end effector's 3d pose relative to the robot.
     * The end effector is the furthest point of the shooter from the pivot point,
     * and where the note leaves the shooter.
     *
     * @return the shooter's end effector's 3d pose relative to the robot
     */
    private Pose3d calculateShooterEndEffectorSelfRelativePose() {
        final Pose3d pivotPoint = ShooterConstants.ROBOT_RELATIVE_PIVOT_POINT.transformBy(new Transform3d(new Translation3d(), new Rotation3d(0, -RobotContainer.PITCHER.getTargetPitch().getRadians(), 0)));
        final Transform3d pivotToEndEffector = new Transform3d(ShooterConstants.SHOOTER_LENGTH_METERS, 0, 0, new Rotation3d());
        return pivotPoint.plus(pivotToEndEffector);
    }

    /**
     * Calculates how much time the note will spend in the air until it reaches the speaker.
     * This is calculated using t = x / v.
     * x being the xy distance from the shooter's end effector to the speaker.
     * v being the xy velocity of the note.
     * t being the time the note will spend in the air.
     *
     * @return the time the note will spend in the air
     */
    private double calculateTimeInAir() {
        final Rotation2d previousAngle = RobotContainer.PITCHER.getTargetPitch();
        final double xyVelocity = previousAngle.getCos() * angularVelocityToTangentialVelocity(-calculateTargetShootingVelocity());
        return previousEndEffectorDistanceFromSpeaker / xyVelocity;
    }

    /**
     * Converts a given shooter's angular velocity to the shooter's tangential velocity.
     *
     * @param angularVelocity the angular velocity of the shooter
     * @return the tangential velocity of the shooter
     */
    private double angularVelocityToTangentialVelocity(double angularVelocity) {
        return angularVelocity / ShooterConstants.REVOLUTIONS_TO_METERS;
    }

    /**
     * Uses {@linkplain java.lang.Math#atan2} to calculate the angle to face the speaker.
     *
     * @param predictedPose the predicted pose of the robot
     * @return the angle the robot should face in order to aim at the speaker
     */
    private MirrorableRotation2d getAngleToSpeaker(Translation2d predictedPose) {
        final Translation2d difference = predictedPose.minus(FieldConstants.SPEAKER_TRANSLATION.get().toTranslation2d());
        return MirrorableRotation2d.fromRadians(Math.atan2(difference.getY(), difference.getX()), false);
    }

    /**
     * Predicts where the robot will be in a given amount of time on the xy plane.
     *
     * @param predictionTime the amount of time to predict the robot's position in, in seconds
     * @return the predicted position of the robot
     */
    private Translation2d predictFutureTranslation(double predictionTime) {
        Logger.recordOutput("ShootingCalculations/NoteTimeInAir", predictionTime);
        final Translation2d fieldRelativeVelocity = getFieldRelativeVelocity();
        final Translation2d currentPose = RobotContainer.POSE_ESTIMATOR.getCurrentPose().getTranslation();
        return currentPose.plus(fieldRelativeVelocity.times(predictionTime));
    }

    /**
     * @return the robot's velocity relative to field in the xy plane
     */
    private Translation2d getFieldRelativeVelocity() {
        final ChassisSpeeds fieldRelativeSpeeds = RobotContainer.SWERVE.getFieldRelativeVelocity();
        return new Translation2d(fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond);
    }
}