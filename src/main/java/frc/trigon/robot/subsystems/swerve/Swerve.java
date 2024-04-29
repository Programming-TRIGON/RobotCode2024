package frc.trigon.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.MotorSubsystem;
import frc.trigon.robot.utilities.mirrorable.Mirrorable;
import frc.trigon.robot.utilities.mirrorable.MirrorablePose2d;
import frc.trigon.robot.utilities.mirrorable.MirrorableRotation2d;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class Swerve extends MotorSubsystem {
    public static final Lock ODOMETRY_LOCK = new ReentrantLock();
    private final SwerveInputsAutoLogged swerveInputs = new SwerveInputsAutoLogged();
    private final SwerveIO swerveIO = SwerveIO.generateIO();
    private final SwerveConstants constants = SwerveConstants.generateConstants();
    private final SwerveModuleIO[] modulesIO;

    public Swerve() {
        setName("Swerve");
        modulesIO = getModulesIO();
        configurePathPlanner();
        constants.getProfiledRotationController().enableContinuousInput(-180, 180);
    }

    @Override
    public void periodic() {
        ODOMETRY_LOCK.lock();
        updateAllInputs();
        ODOMETRY_LOCK.unlock();

        updatePoseEstimatorStates();
        updateNetworkTables();
    }

    @Override
    public void stop() {
        for (SwerveModuleIO currentModule : modulesIO)
            currentModule.stop();
    }

    @Override
    public void setBrake(boolean brake) {
        for (SwerveModuleIO currentModule : modulesIO)
            currentModule.setBrake(brake);
    }

    public SwerveConstants getConstants() {
        return constants;
    }

    public Rotation2d getHeading() {
        final double inputtedHeading = MathUtil.inputModulus(swerveInputs.gyroYawDegrees, -180, 180);
        return Rotation2d.fromDegrees(inputtedHeading);
    }

    public void setHeading(Rotation2d heading) {
        swerveIO.setHeading(heading);
    }

    public ChassisSpeeds getSelfRelativeVelocity() {
        return constants.getKinematics().toChassisSpeeds(getModuleStates());
    }

    // TODO: make sure this is used correctly. Maybe the getDriveRelativeAngle should not be used instead of the getHeading
    public ChassisSpeeds getFieldRelativeVelocity() {
        return ChassisSpeeds.fromFieldRelativeSpeeds(getSelfRelativeVelocity(), getDriveRelativeAngle());
    }

    public Translation3d getGyroAcceleration() {
        return new Translation3d(swerveInputs.accelerationX, swerveInputs.accelerationY, swerveInputs.accelerationZ);
    }

    public Rotation2d getPitch() {
        return Rotation2d.fromDegrees(swerveInputs.gyroPitchDegrees);
    }

    /**
     * Checks if the robot is at a pose.
     *
     * @param pose2d a pose, relative to the blue alliance driver station's right corner
     * @return whether the robot is at the pose
     */
    public boolean atPose(MirrorablePose2d pose2d) {
        final Pose2d mirroredPose = pose2d.get();
        return atXAxisPosition(mirroredPose.getX()) && atYAxisPosition(mirroredPose.getY()) && atAngle(pose2d.getRotation());
    }

    public boolean atXAxisPosition(double xAxisPosition) {
        final double currentXAxisVelocity = getFieldRelativeVelocity().vxMetersPerSecond;
        return atTranslationPosition(RobotContainer.POSE_ESTIMATOR.getCurrentPose().getX(), xAxisPosition, currentXAxisVelocity);
    }

    public boolean atYAxisPosition(double yAxisPosition) {
        final double currentYAxisVelocity = getFieldRelativeVelocity().vyMetersPerSecond;
        return atTranslationPosition(RobotContainer.POSE_ESTIMATOR.getCurrentPose().getY(), yAxisPosition, currentYAxisVelocity);
    }

    public boolean atAngle(MirrorableRotation2d angle) {
        var at = Math.abs(angle.get().getDegrees() - RobotContainer.POSE_ESTIMATOR.getCurrentPose().getRotation().getDegrees()) < SwerveConstants.ROTATION_TOLERANCE_DEGREES;
        Logger.recordOutput("Swerve/AtTargetAngle", at);
        return at;
    }

    public SwerveModulePosition[] getWheelPositions() {
        final SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[modulesIO.length];
        for (int i = 0; i < modulesIO.length; i++)
            swerveModulePositions[i] = modulesIO[i].getOdometryPosition(modulesIO[i].getLastOdometryUpdateIndex());
        return swerveModulePositions;
    }

    public void runWheelRadiusCharacterization(double omegaRadsPerSec) {
        selfRelativeDrive(new ChassisSpeeds(0, 0, omegaRadsPerSec));
    }

    /**
     * Locks the swerve, so it'll be hard to move it. This will make the modules look in the middle of a robot in an "x" shape.
     */
    void lockSwerve() {
        final SwerveModuleState
                right = new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                left = new SwerveModuleState(0, Rotation2d.fromDegrees(45));

        modulesIO[0].setTargetState(left);
        modulesIO[1].setTargetState(right);
        modulesIO[2].setTargetState(right);
        modulesIO[3].setTargetState(left);
    }

    /**
     * Moves the robot to a certain pose using PID.
     *
     * @param targetPose the target pose, relative to the blue alliance driver station's right corner
     */
    void pidToPose(MirrorablePose2d targetPose) {
        final Pose2d currentPose = RobotContainer.POSE_ESTIMATOR.getCurrentPose();
        final Pose2d mirroredTargetPose = targetPose.get();
        final double xSpeed = constants.getTranslationsController().calculate(currentPose.getX(), mirroredTargetPose.getX());
        final double ySpeed = constants.getTranslationsController().calculate(currentPose.getY(), mirroredTargetPose.getY());
        final int direction = Mirrorable.isRedAlliance() ? -1 : 1;
        final ChassisSpeeds targetFieldRelativeSpeeds = new ChassisSpeeds(
                xSpeed * direction,
                ySpeed * direction,
                calculateProfiledAngleSpeedToTargetAngle(targetPose.getRotation())
        );
        selfRelativeDrive(fieldRelativeSpeedsToSelfRelativeSpeeds(targetFieldRelativeSpeeds));
    }

    void initializeDrive(boolean closedLoop) {
        setClosedLoop(closedLoop);
        resetRotationController();
    }

    void resetRotationController() {
        constants.getProfiledRotationController().reset(RobotContainer.POSE_ESTIMATOR.getCurrentPose().getRotation().getDegrees());
    }

    void setClosedLoop(boolean closedLoop) {
        for (SwerveModuleIO currentModule : modulesIO)
            currentModule.setDriveMotorClosedLoop(closedLoop);
    }

    /**
     * Drives the swerve with the given powers and a target angle, relative to the field's frame of reference.
     *
     * @param xPower      the x power
     * @param yPower      the y power
     * @param targetAngle the target angle, relative to the blue alliance's forward position
     */
    void fieldRelativeDrive(double xPower, double yPower, MirrorableRotation2d targetAngle) {
        final ChassisSpeeds speeds = selfRelativeSpeedsFromFieldRelativePowers(xPower, yPower, 0);
        speeds.omegaRadiansPerSecond = calculateProfiledAngleSpeedToTargetAngle(targetAngle);
        Logger.recordOutput("Swerve/AnglePID/TargetAngle", MathUtil.inputModulus(targetAngle.get().getDegrees(), 0, 360));
        Logger.recordOutput("Swerve/AnglePID/AngleSetpoint", MathUtil.inputModulus(constants.getProfiledRotationController().getSetpoint().position, 0, 360));
        Logger.recordOutput("Swerve/AnglePID/CurrentAngle", MathUtil.inputModulus(RobotContainer.POSE_ESTIMATOR.getCurrentPose().getRotation().getDegrees(), 0, 360));

        selfRelativeDrive(speeds);
    }

    /**
     * Drives the swerve with the given powers, relative to the field's frame of reference.
     *
     * @param xPower     the x power
     * @param yPower     the y power
     * @param thetaPower the theta power
     */
    void fieldRelativeDrive(double xPower, double yPower, double thetaPower) {
        final ChassisSpeeds speeds = selfRelativeSpeedsFromFieldRelativePowers(xPower, yPower, thetaPower);
        selfRelativeDrive(speeds);
    }

    /**
     * Drives the swerve with the given powers, relative to the robot's frame of reference.
     *
     * @param xPower     the x power
     * @param yPower     the y power
     * @param thetaPower the theta power
     */
    void selfRelativeDrive(double xPower, double yPower, double thetaPower) {
        final ChassisSpeeds speeds = powersToSpeeds(xPower, yPower, thetaPower);
        selfRelativeDrive(speeds);
    }

    /**
     * Drives the swerve with the given powers and a target angle, relative to the robot's frame of reference.
     *
     * @param xPower      the x power
     * @param yPower      the y power
     * @param targetAngle the target angle
     */
    void selfRelativeDrive(double xPower, double yPower, MirrorableRotation2d targetAngle) {
        final ChassisSpeeds speeds = powersToSpeeds(xPower, yPower, 0);
        speeds.omegaRadiansPerSecond = calculateProfiledAngleSpeedToTargetAngle(targetAngle);
        Logger.recordOutput("Stuff/TargetAngle", MathUtil.inputModulus(targetAngle.get().getDegrees(), 0, 360));
        Logger.recordOutput("Stuff/AngleSetpoint", MathUtil.inputModulus(constants.getProfiledRotationController().getSetpoint().position, 0, 360));
        Logger.recordOutput("Stuff/CurrentAngle", MathUtil.inputModulus(RobotContainer.POSE_ESTIMATOR.getCurrentPose().getRotation().getDegrees(), 0, 360));

        selfRelativeDrive(speeds);
    }

    private void selfRelativeDrive(ChassisSpeeds chassisSpeeds) {
        chassisSpeeds = discretize(chassisSpeeds);
        if (isStill(chassisSpeeds)) {
            stop();
            return;
        }

        final SwerveModuleState[] swerveModuleStates = constants.getKinematics().toSwerveModuleStates(chassisSpeeds);
        setTargetModuleStates(swerveModuleStates);
    }

    private void setTargetModuleStates(SwerveModuleState[] swerveModuleStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, constants.getMaxSpeedMetersPerSecond());
        for (int i = 0; i < modulesIO.length; i++)
            modulesIO[i].setTargetState(swerveModuleStates[i]);
    }

    /**
     * When the robot drives while rotating it skews a bit to the side.
     * This should fix the chassis speeds, so they won't make the robot skew while rotating.
     *
     * @param chassisSpeeds the chassis speeds to fix skewing for
     * @return the fixed speeds
     */
    private ChassisSpeeds discretize(ChassisSpeeds chassisSpeeds) {
        return ChassisSpeeds.discretize(chassisSpeeds, RobotConstants.PERIODIC_TIME_SECONDS);
    }

    private void updatePoseEstimatorStates() {
        final int odometryUpdates = swerveInputs.odometryUpdatesYawDegrees.length;
        final SwerveDriveWheelPositions[] swerveWheelPositions = new SwerveDriveWheelPositions[odometryUpdates];
        final Rotation2d[] gyroRotations = new Rotation2d[odometryUpdates];

        for (int i = 0; i < odometryUpdates; i++) {
            swerveWheelPositions[i] = getSwerveWheelPositions(i);
            gyroRotations[i] = Rotation2d.fromDegrees(swerveInputs.odometryUpdatesYawDegrees[i]);
        }

        RobotContainer.POSE_ESTIMATOR.updatePoseEstimatorStates(swerveWheelPositions, gyroRotations, swerveInputs.odometryUpdatesTimestamp);
    }

    private SwerveDriveWheelPositions getSwerveWheelPositions(int odometryUpdateIndex) {
        final SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[modulesIO.length];
        for (int i = 0; i < modulesIO.length; i++)
            swerveModulePositions[i] = modulesIO[i].getOdometryPosition(odometryUpdateIndex);
        return new SwerveDriveWheelPositions(swerveModulePositions);
    }

    private void updateAllInputs() {
        swerveIO.updateInputs(swerveInputs);
        Logger.processInputs("Swerve", swerveInputs);

        for (SwerveModuleIO currentModule : modulesIO)
            currentModule.periodic();
    }

    private void configurePathPlanner() {
        AutoBuilder.configureHolonomic(
                RobotContainer.POSE_ESTIMATOR::getCurrentPose,
//                (pose) -> RobotContainer.POSE_ESTIMATOR.resetPose(RobotContainer.POSE_ESTIMATOR.getCurrentPose()),
                (pose) -> {
                },
                this::getSelfRelativeVelocity,
                this::selfRelativeDrive,
                constants.getPathFollowerConfig(),
                Mirrorable::isRedAlliance,
                this
        );
    }

    private boolean atTranslationPosition(double currentTranslationPosition, double targetTranslationPosition, double currentTranslationVelocity) {
        return Math.abs(currentTranslationPosition - targetTranslationPosition) < SwerveConstants.TRANSLATION_TOLERANCE_METERS &&
                Math.abs(currentTranslationVelocity) < SwerveConstants.TRANSLATION_VELOCITY_TOLERANCE;
    }

    private double calculateProfiledAngleSpeedToTargetAngle(MirrorableRotation2d targetAngle) {
        final Rotation2d currentAngle = RobotContainer.POSE_ESTIMATOR.getCurrentPose().getRotation();
        return Units.degreesToRadians(constants.getProfiledRotationController().calculate(currentAngle.getDegrees(), targetAngle.get().getDegrees()));
    }

    private ChassisSpeeds selfRelativeSpeedsFromFieldRelativePowers(double xPower, double yPower, double thetaPower) {
        final ChassisSpeeds fieldRelativeSpeeds = powersToSpeeds(xPower, yPower, thetaPower);
        return fieldRelativeSpeedsToSelfRelativeSpeeds(fieldRelativeSpeeds);
    }

    private ChassisSpeeds fieldRelativeSpeedsToSelfRelativeSpeeds(ChassisSpeeds fieldRelativeSpeeds) {
        return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getDriveRelativeAngle());
    }

    private Rotation2d getDriveRelativeAngle() {
        final Rotation2d currentAngle = RobotContainer.POSE_ESTIMATOR.getCurrentPose().getRotation();
        return Mirrorable.isRedAlliance() ? currentAngle.rotateBy(Rotation2d.fromDegrees(180)) : currentAngle;
    }

    private ChassisSpeeds powersToSpeeds(double xPower, double yPower, double thetaPower) {
        return new ChassisSpeeds(
                xPower * constants.getMaxSpeedMetersPerSecond(),
                yPower * constants.getMaxSpeedMetersPerSecond(),
                Math.pow(thetaPower, 2) * Math.signum(thetaPower) * constants.getMaxRotationalSpeedRadiansPerSecond()
        );
    }

    private void updateNetworkTables() {
        Logger.recordOutput("Swerve/Velocity/Rot", getSelfRelativeVelocity().omegaRadiansPerSecond);
        Logger.recordOutput("Swerve/Velocity/X", getSelfRelativeVelocity().vxMetersPerSecond);
        Logger.recordOutput("Swerve/Velocity/Y", getSelfRelativeVelocity().vyMetersPerSecond);
    }

    @AutoLogOutput(key = "Swerve/CurrentStates")
    private SwerveModuleState[] getModuleStates() {
        final SwerveModuleState[] states = new SwerveModuleState[modulesIO.length];

        for (int i = 0; i < modulesIO.length; i++)
            states[i] = modulesIO[i].getCurrentState();

        return states;
    }

    @AutoLogOutput(key = "Swerve/TargetStates")
    @SuppressWarnings("unused")
    private SwerveModuleState[] getTargetStates() {
        final SwerveModuleState[] states = new SwerveModuleState[modulesIO.length];

        for (int i = 0; i < modulesIO.length; i++)
            states[i] = modulesIO[i].getTargetState();

        return states;
    }

    /**
     * Returns whether the given chassis speeds are considered to be "still" by the swerve neutral deadband.
     *
     * @param chassisSpeeds the chassis speeds to check
     * @return true if the chassis speeds are considered to be "still"
     */
    private boolean isStill(ChassisSpeeds chassisSpeeds) {
        return Math.abs(chassisSpeeds.vxMetersPerSecond) <= SwerveConstants.DRIVE_NEUTRAL_DEADBAND &&
                Math.abs(chassisSpeeds.vyMetersPerSecond) <= SwerveConstants.DRIVE_NEUTRAL_DEADBAND &&
                Math.abs(chassisSpeeds.omegaRadiansPerSecond) <= SwerveConstants.ROTATION_NEUTRAL_DEADBAND;
    }

    private SwerveModuleIO[] getModulesIO() {
        if (RobotConstants.IS_REPLAY) {
            return new SwerveModuleIO[]{
                    new SwerveModuleIO("FrontLeft"),
                    new SwerveModuleIO("FrontRight"),
                    new SwerveModuleIO("RearLeft"),
                    new SwerveModuleIO("RearRight")
            };
        }

        return constants.getModulesIO().get();
    }
}