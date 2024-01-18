package frc.trigon.robot.subsystems.shooter;

import frc.trigon.robot.subsystems.MotorSubsystem;
import org.littletonrobotics.junction.Logger;

public class Shooter extends MotorSubsystem {
    private static final Shooter INSTANCE = new Shooter();
    private final ShooterInputsAutoLogged shooterInputs = new ShooterInputsAutoLogged();
    private final ShooterIO shooterIO = ShooterIO.generateIO();
    private double
            targetTopVelocityRevolutionsPerSecond = 0,
            targetBottomVelocityRevolutionsPerSecond = 0;

    public static Shooter getInstance() {
        return INSTANCE;
    }

    private Shooter() {
        setName("Shooter");
    }

    @Override
    public void stop() {
        shooterIO.stop();
        targetTopVelocityRevolutionsPerSecond = 0;
    }

    @Override
    public void periodic() {
        shooterIO.updateInputs(shooterInputs);
        Logger.processInputs("Shooter", shooterInputs);
        updateMechanism();
    }

    public boolean atTargetShootingVelocity() {
        return Math.abs(shooterInputs.topVelocityRevolutionsPerSecond - targetTopVelocityRevolutionsPerSecond) < ShooterConstants.TOLERANCE_REVOLUTIONS &&
                Math.abs(shooterInputs.bottomVelocityRevolutionsPerSecond - targetBottomVelocityRevolutionsPerSecond) < ShooterConstants.TOLERANCE_REVOLUTIONS;
    }

    void shootAtSpeaker(double distanceToSpeaker) {
        final double targetTopVelocityRevolutionsPerSecond = calculateShootingAtSpeakerTopVelocity(distanceToSpeaker);
        final double targetBottomVelocityRevolutionsPerSecond = targetTopVelocityRevolutionsPerSecond * ShooterConstants.TOP_TO_BOTTOM_SHOOTING_RATIO;
        setTargetVelocity(targetTopVelocityRevolutionsPerSecond, targetBottomVelocityRevolutionsPerSecond);
    }

    void setTargetVelocity(double targetTopVelocityRevolutionsPerSecond, double targetBottomVelocityRevolutionsPerSecond) {
        final double targetTopVoltage = ShooterConstants.TOP_CONTROLLER.calculate(shooterInputs.topVelocityRevolutionsPerSecond, targetTopVelocityRevolutionsPerSecond);
        final double targetBottomVoltage = ShooterConstants.TOP_CONTROLLER.calculate(shooterInputs.bottomVelocityRevolutionsPerSecond, targetBottomVelocityRevolutionsPerSecond);

        shooterIO.setTargetTopVoltage(targetTopVoltage);
        shooterIO.setTargetBottomVoltage(targetBottomVoltage);

        this.targetTopVelocityRevolutionsPerSecond = targetTopVelocityRevolutionsPerSecond;
        this.targetBottomVelocityRevolutionsPerSecond = targetBottomVelocityRevolutionsPerSecond;
    }

    void resetControllers() {
        ShooterConstants.TOP_CONTROLLER.reset(shooterInputs.topVelocityRevolutionsPerSecond);
        ShooterConstants.BOTTOM_CONTROLLER.reset(shooterInputs.bottomVelocityRevolutionsPerSecond);
    }

    private double calculateShootingAtSpeakerTopVelocity(double distanceToSpeaker) {
        return ShooterConstants.VELOCITY_INTERPOLATION.predict(distanceToSpeaker);
    }

    private void updateMechanism() {
        ShooterConstants.TOP_SHOOTING_MECHANISM.updateMechanism(shooterInputs.topVelocityRevolutionsPerSecond, targetTopVelocityRevolutionsPerSecond);
        ShooterConstants.BOTTOM_SHOOTING_MECHANISM.updateMechanism(shooterInputs.bottomVelocityRevolutionsPerSecond, targetBottomVelocityRevolutionsPerSecond);
    }
}

