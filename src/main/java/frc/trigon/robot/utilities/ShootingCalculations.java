package frc.trigon.robot.utilities;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.constants.ShootingConstants;
import frc.trigon.robot.subsystems.pitcher.PitcherConstants;
import frc.trigon.robot.subsystems.shooter.ShooterConstants;
import frc.trigon.robot.utilities.mirrorable.MirrorableRotation2d;
import frc.trigon.robot.utilities.mirrorable.MirrorableTranslation3d;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShootingCalculations {
    private static ShootingCalculations INSTANCE = null;
    @AutoLogOutput(key = "ShootingCalculations/EndEffectorXYDistanceFromShootingTarget")
    private double previousEndEffectorXYDistanceFromShootingTarget = 0;
    private TargetShootingState targetShootingState = new TargetShootingState(new MirrorableRotation2d(new Rotation2d(), false), new Rotation2d(), 0);

    public static ShootingCalculations getInstance() {
        if (INSTANCE == null)
            INSTANCE = new ShootingCalculations();
        return INSTANCE;
    }

    private ShootingCalculations() {
    }

    /**
     * Updates the {@linkplain ShootingCalculations#targetShootingState} class variable to contain the target state for delivery.
     */
    public void updateCalculationsForDelivery() {
        Logger.recordOutput("ShootingCalculations/TargetDeliveryPose", FieldConstants.TARGET_DELIVERY_POSITION.get());
        targetShootingState = calculateTargetShootingState(FieldConstants.TARGET_DELIVERY_POSITION, ShootingConstants.DELIVERY_VELOCITY_REVOLUTIONS_PER_SECOND, true);
    }

    /**
     * Updates the {@linkplain ShootingCalculations#targetShootingState} class variable to contain the target state for shooting at the speaker.
     */
    public void updateCalculationsForSpeakerShot() {
        Logger.recordOutput("ShootingCalculations/TargetSpeakerPose", FieldConstants.SPEAKER_TRANSLATION.get());
        targetShootingState = calculateTargetShootingState(FieldConstants.SPEAKER_TRANSLATION, ShootingConstants.SPEAKER_SHOT_VELOCITY_REVOLUTIONS_PER_SECOND, false);
    }

    /**
     * @return the target state of the robot to shoot at the provided shooting target
     */
    public TargetShootingState getTargetShootingState() {
        return targetShootingState;
    }

    /**
     * Converts a given shooter's angular velocity to the shooter's tangential velocity.
     *
     * @param angularVelocity the angular velocity of the shooter
     * @return the tangential velocity of the shooter
     */
    public double angularVelocityToTangentialVelocity(double angularVelocity) {
        return angularVelocity / ShooterConstants.REVOLUTIONS_TO_METERS;
    }

    /**
     * @return the robot's velocity relative to field in the xy plane
     */
    public Translation2d getRobotFieldRelativeVelocity() {
        final ChassisSpeeds fieldRelativeSpeeds = RobotContainer.SWERVE.getFieldRelativeVelocity();
        return new Translation2d(fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond);
    }

    /**
     * Calculates the necessary pitch, robot yaw, and shooting velocity in order to shoot at the shooting target.
     *
     * @param shootingTarget                           the point we want the note reach
     * @param targetShootingVelocityRotationsPerSecond the given shooting velocity to calculate optimal pitch from
     * @param reachFromAbove                           should we reach to point from above, with an arch, or from below, as fast as possible
     *                                                 Shooting from above is useful for actions like delivery, whereas shooting from below is useful when we don't want to come from above, and in our case touch the upper speaker
     * @return the target state of the robot so the note will reach the shooting target, as a {@linkplain ShootingCalculations.TargetShootingState}
     */
    private TargetShootingState calculateTargetShootingState(MirrorableTranslation3d shootingTarget, double targetShootingVelocityRotationsPerSecond, boolean reachFromAbove) {
        final double noteTimeInAir = calculateNoteTimeInAir(targetShootingVelocityRotationsPerSecond);
        final Translation2d predictedTranslation = predictFutureTranslation(noteTimeInAir);
        final MirrorableRotation2d targetRobotAngle = getAngleToTarget(predictedTranslation, shootingTarget);
        final Rotation2d targetPitch = calculateTargetPitch(targetShootingVelocityRotationsPerSecond, reachFromAbove, predictedTranslation, targetRobotAngle, shootingTarget);
        return new TargetShootingState(targetRobotAngle, targetPitch, targetShootingVelocityRotationsPerSecond);
    }

    /**
     * Calculates the optimal pitch for the given parameters, using the Projectile Motion calculation.
     *
     * @param targetShootingVelocityRevolutionsPerSecond the exit velocity of the note, in revolutions per second
     * @param reachFromAbove                             should we reach to point from above, with an arch, or from below, as fast as possible
     *                                                   Shooting from above is useful for actions like delivery, whereas shooting from below is useful when we don't want to come from above, and in our case touch the upper speaker
     * @param predictedTranslation                       the predicted translation of the robot, to find the end effector's pose with
     * @param targetRobotAngle                           the previously calculated target robot angle, to find the end effector's pose with
     * @param shootingTarget                             the point we want the note reach
     * @return the pitch the pitcher should reach in order to shoot to the shooting target
     */
    private Rotation2d calculateTargetPitch(double targetShootingVelocityRevolutionsPerSecond, boolean reachFromAbove, Translation2d predictedTranslation, MirrorableRotation2d targetRobotAngle, MirrorableTranslation3d shootingTarget) {
        final double noteTangentialVelocity = angularVelocityToTangentialVelocity(targetShootingVelocityRevolutionsPerSecond);
        final Pose3d endEffectorFieldRelativePose = calculateShooterEndEffectorFieldRelativePose(RobotContainer.PITCHER.getTargetPitch(), predictedTranslation, targetRobotAngle);
        previousEndEffectorXYDistanceFromShootingTarget = endEffectorFieldRelativePose.getTranslation().toTranslation2d().getDistance(shootingTarget.get().toTranslation2d());
        final double endEffectorHeightDifferenceFromTarget = shootingTarget.get().getZ() - endEffectorFieldRelativePose.getZ();
        return calculateTargetPitchUsingProjectileMotion(noteTangentialVelocity, previousEndEffectorXYDistanceFromShootingTarget, endEffectorHeightDifferenceFromTarget, reachFromAbove);
    }

    /**
     * Calculates the pitch the pitcher should reach in order to shoot at the shooting target using projectile motion.
     * This will fully calculate the target pitch using physics.
     *
     * @param noteTangentialVelocity                         the tangential velocity of the shooter
     * @param shooterEndEffectorXYDistanceFromShootingTarget the xy distance from the shooting target to the shooter's end effector on the xy plane
     * @param endEffectorHeightDifferenceFromTarget          the height difference between the shooter's end effector and the shooting target
     * @param reachFromAbove                                 should we reach to point from above, with an arch, or from below, as fast as possible
     *                                                       Shooting from above is useful for actions like delivery, whereas shooting from below is useful when we don't want to come from above, and in our case touch the upper speaker
     * @return the pitch the robot should reach in order to shoot at the shooting target
     * @link <a href="https://en.wikipedia.org/wiki/Projectile_motion#Angle_%CE%B8_required_to_hit_coordinate_(x,_y)">Projectile Motion</a>
     */
    private Rotation2d calculateTargetPitchUsingProjectileMotion(double noteTangentialVelocity, double shooterEndEffectorXYDistanceFromShootingTarget, double endEffectorHeightDifferenceFromTarget, boolean reachFromAbove) {
        Logger.recordOutput("ShootingCalculations/NoteTangentialVelocity", noteTangentialVelocity);
        Logger.recordOutput("ShootingCalculations/ShooterEndEffectorHeight", endEffectorHeightDifferenceFromTarget);
        final double gForce = ShootingConstants.G_FORCE;
        final double velocitySquared = Math.pow(noteTangentialVelocity, 2);
        final double velocity4thPower = Math.pow(noteTangentialVelocity, 4);
        final double distanceSquared = Math.pow(shooterEndEffectorXYDistanceFromShootingTarget, 2);
        final double squareRoot = Math.sqrt(
                velocity4thPower - (gForce * ((gForce * distanceSquared) + (2 * velocitySquared * endEffectorHeightDifferenceFromTarget)))
        );
        final double numerator = reachFromAbove ? velocitySquared + squareRoot : velocitySquared - squareRoot;
        final double denominator = gForce * shooterEndEffectorXYDistanceFromShootingTarget;
        final double fraction = numerator / denominator;
        double angleRadians = Math.atan(fraction);
        if (Double.isNaN(angleRadians) || Double.isInfinite(angleRadians) || angleRadians < 0)
            angleRadians = PitcherConstants.DEFAULT_PITCH.getRadians();
        Logger.recordOutput("ShootingCalculations/TargetPitch", Math.toDegrees(angleRadians));
        return Rotation2d.fromRadians(angleRadians);
    }

    /**
     * Calculates the shooter's end effector's 3d pose on the field from the given parameters.
     * The end effector is the furthest point of the shooter from the pivot point,
     * and where the note leaves the shooter.
     *
     * @param pitcherAngle         the pitcher angle, to base off from
     * @param predictedTranslation the field relative predicted translation, to base off from
     * @param robotAngle           the robot angle, to base off from
     * @return the shooter's end effector's 3d pose on the field
     */
    public Pose3d calculateShooterEndEffectorFieldRelativePose(Rotation2d pitcherAngle, Translation2d predictedTranslation, MirrorableRotation2d robotAngle) {
        final Pose3d endEffectorSelfRelativePose = calculateShooterEndEffectorSelfRelativePose(pitcherAngle);
        final Transform3d robotToEndEffector = endEffectorSelfRelativePose.minus(new Pose3d());
        final Pose3d predictedPose = new Pose3d(new Pose2d(predictedTranslation, robotAngle.get()));
        return predictedPose.transformBy(robotToEndEffector);
    }

    /**
     * Calculates the shooter's end effector's 3d pose relative to the robot.
     * The end effector is the furthest point of the shooter from the pivot point,
     * and where the note leaves the shooter.
     *
     * @param pitcherAngle the pitcher angle, to base off from
     * @return the shooter's end effector's 3d pose relative to the robot
     */
    private Pose3d calculateShooterEndEffectorSelfRelativePose(Rotation2d pitcherAngle) {
        final Pose3d pivotPoint = ShooterConstants.ROBOT_RELATIVE_PIVOT_POINT.transformBy(new Transform3d(new Translation3d(), new Rotation3d(0, -pitcherAngle.getRadians(), 0)));
        return pivotPoint.plus(ShooterConstants.PIVOT_POINT_TO_NOTE_EXIT_POSITION);
    }

    /**
     * Calculates how much time the note will spend in the air until it reaches the shooting target.
     * This is calculated using t = x / v.
     * x being the xy distance from the shooter's end effector to the shooting target.
     * v being the xy velocity of the note.
     * t being the time the note will spend in the air.
     *
     * @param targetShootingVelocityRevolutionsPerSecond the exit velocity of the note, in revolutions per second
     * @return the time the note will spend in the air
     */
    private double calculateNoteTimeInAir(double targetShootingVelocityRevolutionsPerSecond) {
        final Rotation2d previousAngle = RobotContainer.PITCHER.getTargetPitch();
        final double xyVelocity = previousAngle.getCos() * angularVelocityToTangentialVelocity(targetShootingVelocityRevolutionsPerSecond);
        return previousEndEffectorXYDistanceFromShootingTarget / xyVelocity;
    }

    /**
     * Uses {@linkplain java.lang.Math#atan2} to calculate the angle to face the shooting target.
     *
     * @param predictedTranslation the predicted pose of the robot
     * @param shootingTarget       the shootingTarget of the shooting. What we want the projectile to reach
     * @return the angle (yaw) the robot should reach in order to face the shooting target
     */
    private MirrorableRotation2d getAngleToTarget(Translation2d predictedTranslation, MirrorableTranslation3d shootingTarget) {
        final Translation2d difference = predictedTranslation.minus(shootingTarget.get().toTranslation2d());
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
        final Translation2d fieldRelativeVelocity = getRobotFieldRelativeVelocity();
        final Translation2d currentPose = RobotContainer.POSE_ESTIMATOR.getCurrentPose().getTranslation();
        return currentPose.plus(fieldRelativeVelocity.times(predictionTime));
    }

    public record TargetShootingState(MirrorableRotation2d targetRobotAngle, Rotation2d targetPitch,
                                      double targetShootingVelocityRevolutionsPerSecond) {
    }
}