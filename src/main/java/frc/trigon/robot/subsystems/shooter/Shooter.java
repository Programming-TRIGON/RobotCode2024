package frc.trigon.robot.subsystems.shooter;


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
        targetVelocityRotationsPerSecond = 0;
    }

    @Override
    public void periodic() {
        shooterIO.updateInputs(shooterInputs);
        Logger.processInputs("Shooter", shooterInputs);
        updateMechanism();
    }

    public boolean atTargetShootingVelocity() {
        return Math.abs(shooterInputs.shootingVelocityRotationsPerSecond - targetVelocityRotationsPerSecond) < ShooterConstants.TOLERANCE_ROTATIONS;
    }

    void shootAtSpeaker(double distanceToSpeaker) {
        shooterIO.setTargetFeedingMotorVoltage(ShooterConstants.FEEDING_MOTOR_VOLTAGE);
        setTargetShootingVelocity(calculateShootingAtSpeakerVelocity(distanceToSpeaker));
    }

    void setTargetShootingVelocity(double targetVelocityRotationsPerSecond) {
        shooterIO.setTargetShootingVelocity(targetVelocityRotationsPerSecond);
        this.targetVelocityRotationsPerSecond = targetVelocityRotationsPerSecond;
    }

    private double calculateShootingAtSpeakerVelocity(double distanceToSpeaker) {
        return ShooterConstants.VELOCITY_INTERPOLATION.predict(distanceToSpeaker);
    }

    private void updateMechanism() {
//        ShooterConstants.SHOOTING_MECHANISM.setVelocity(shooterInputs.shootingVelocityRotationsPerSecond, targetVelocityRotationsPerSecond);
    }
}

