package frc.trigon.robot.subsystems.shooter;

import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.MotorSubsystem;
import frc.trigon.robot.utilities.CurrentWatcher;
import frc.trigon.robot.utilities.ShootingCalculations;
import org.littletonrobotics.junction.Logger;

public class Shooter extends MotorSubsystem {
    private final ShootingCalculations shootingCalculations = ShootingCalculations.getInstance();
    private final ShooterInputsAutoLogged shooterInputs = new ShooterInputsAutoLogged();
    private final ShooterIO shooterIO = ShooterIO.generateIO();
    private double targetVelocityRevolutionsPerSecond = 0;

    public Shooter() {
        setName("Shooter");
        configureCurrentWatcher();
    }

    @Override
    public void stop() {
        shooterIO.stop();
        targetVelocityRevolutionsPerSecond = 0;
    }

    @Override
    public void periodic() {
        shooterIO.updateInputs(shooterInputs);
        Logger.processInputs("Shooter", shooterInputs);
        updateMechanism();
    }

    public boolean atTargetShootingVelocity() {
        return Math.abs(shooterInputs.velocityRevolutionsPerSecond - targetVelocityRevolutionsPerSecond) < ShooterConstants.TOLERANCE_REVOLUTIONS;
    }

    void shootAtSpeaker() {
        final double targetTopVelocityRevolutionsPerSecond = shootingCalculations.calculateTargetTopShootingVelocity();
        setTargetVelocity(targetTopVelocityRevolutionsPerSecond);
    }

    void setTargetVelocity(double targetVelocityRevolutionsPerSecond) {
        final double targetVoltage = ShooterConstants.STATE_SPACE_CONTROLLER.calculate(shooterInputs.velocityRevolutionsPerSecond, targetVelocityRevolutionsPerSecond);
        shooterIO.setTargetVoltage(targetVoltage);
        this.targetVelocityRevolutionsPerSecond = targetVelocityRevolutionsPerSecond;
    }

    void resetController() {
        ShooterConstants.STATE_SPACE_CONTROLLER.reset(shooterInputs.velocityRevolutionsPerSecond);
    }

    private void updateMechanism() {
        ShooterConstants.SHOOTING_MECHANISM.updateMechanism(shooterInputs.velocityRevolutionsPerSecond, targetVelocityRevolutionsPerSecond);
    }

    private void configureCurrentWatcher() {
        new CurrentWatcher(
                () -> shooterInputs.current,
                ShooterConstants.SHOOTING_CURRENT,
                ShooterConstants.SHOOTING_TIME_THRESHOLD,
                () -> OperatorConstants.DRIVER_CONTROLLER.rumble(ShooterConstants.SHOOTING_RUMBLE_DURATION_SECONDS, ShooterConstants.SHOOTING_RUMBLE_POWER)
        );
    }
}

