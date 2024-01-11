package frc.trigon.robot.subsystems.shooter;


import edu.wpi.first.math.geometry.Translation2d;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.littletonrobotics.junction.Logger;

public class Shooter extends MotorSubsystem {
    private static final Shooter INSTANCE = new Shooter();
    private final ShooterInputsAutoLogged shooterInputs = new ShooterInputsAutoLogged();
    private final ShooterIO shooterIO = ShooterIO.generateIO();
    private double targetVelocityRotationsPerSecond = 0;

    public static Shooter getInstance() {
        return INSTANCE;
    }

    private Shooter() {
        setName("Shooter");
    }

    @Override
    public void stop() {
        shooterIO.stop();
    }

    @Override
    public void periodic() {
        shooterIO.updateInputs(shooterInputs);
        Logger.processInputs("Shooter", shooterInputs);
    }

    public boolean atTargetShootingVelocity() {
        return Math.abs(shooterInputs.shootingVelocityRotationsPerSecond - targetVelocityRotationsPerSecond) < ShooterConstants.TOLERANCE_ROTATIONS;
    }

    void shootAtSpeaker() {
        shooterIO.setTargetFeedingMotorVoltage(ShooterConstants.FEEDING_MOTOR_VOLTAGE);
        setTargetShootingVelocity(calculateShootingAtSpeakerVelocity());
    }

    void setTargetShootingVelocity(double targetVelocityRotationsPerSecond) {
        shooterIO.setTargetShootingVelocity(targetVelocityRotationsPerSecond);
        this.targetVelocityRotationsPerSecond = targetVelocityRotationsPerSecond;
    }

    private double calculateShootingAtSpeakerVelocity() {
        final Translation2d currentMirroredAllianceTranslation = RobotContainer.POSE_ESTIMATOR.getCurrentPose().toMirroredAlliancePose().getTranslation();
        final double distanceToSpeaker = currentMirroredAllianceTranslation.getDistance(FieldConstants.SPEAKER_TRANSLATION);
        return ShooterConstants.INTERPOLATION.predict(distanceToSpeaker);
    }
}

