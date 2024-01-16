package frc.trigon.robot.subsystems.shooter.placeholdershooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.trigon.robot.subsystems.shooter.ShooterIO;
import frc.trigon.robot.subsystems.shooter.ShooterInputsAutoLogged;

public class PLACEHOLDERShooterIO extends ShooterIO {
    private final TalonFX
            topMotor = PLACEHOLDERShooterConstants.TOP_SHOOTING_MOTOR,
            bottomMotor = PLACEHOLDERShooterConstants.BOTTOM_SHOOTING_MOTOR;
    private final VelocityTorqueCurrentFOC
            topVelocityRequest = new VelocityTorqueCurrentFOC(0),
            bottomVelocityRequest = new VelocityTorqueCurrentFOC(0);

    @Override
    protected void updateInputs(ShooterInputsAutoLogged inputs) {
        refreshStatusSignals();

        inputs.topVelocityRevolutionsPerSecond = PLACEHOLDERShooterConstants.TOP_MOTOR_VELOCITY_SIGNAL.getValue();
        inputs.bottomVelocityRevolutionsPerSecond = PLACEHOLDERShooterConstants.BOTTOM_MOTOR_VELOCITY_SIGNAL.getValue();
    }

    @Override
    protected void setTargetTopVelocity(double targetVelocityRevolutionsPerSecond) {
        topMotor.setControl(topVelocityRequest.withVelocity(targetVelocityRevolutionsPerSecond));
    }

    @Override
    protected void setTargetBottomVelocity(double targetVelocityRevolutionsPerSecond) {
        bottomMotor.setControl(bottomVelocityRequest.withVelocity(targetVelocityRevolutionsPerSecond));
    }

    @Override
    protected void stop() {
        bottomMotor.stopMotor();
        topMotor.stopMotor();
    }

    private void refreshStatusSignals() {
        BaseStatusSignal.refreshAll(
                PLACEHOLDERShooterConstants.TOP_MOTOR_VELOCITY_SIGNAL,
                PLACEHOLDERShooterConstants.BOTTOM_MOTOR_VELOCITY_SIGNAL
        );
    }
}
