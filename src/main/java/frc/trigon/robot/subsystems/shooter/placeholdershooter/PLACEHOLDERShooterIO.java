package frc.trigon.robot.subsystems.shooter.placeholdershooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.trigon.robot.subsystems.shooter.ShooterIO;
import frc.trigon.robot.subsystems.shooter.ShooterInputsAutoLogged;

public class PLACEHOLDERShooterIO extends ShooterIO {
    private final TalonFX
            shootingMotor = PLACEHOLDERShooterConstants.SHOOTING_MOTOR,
            feedingMotor = PLACEHOLDERShooterConstants.FEEDING_MOTOR;
    private final VelocityTorqueCurrentFOC shootingVelocityRequest = new VelocityTorqueCurrentFOC(0);
    private final VoltageOut feedingVoltageRequest = new VoltageOut(0);

    @Override
    protected void updateInputs(ShooterInputsAutoLogged inputs) {
        updateStatusSignals();

        inputs.shootingVelocityRevolutionsPerSecond = PLACEHOLDERShooterConstants.SHOOTING_MOTOR_VELOCITY_SIGNAL.getValue();
        inputs.feedingVoltage = PLACEHOLDERShooterConstants.FEEDING_MOTOR_VOLTAGE_SIGNAL.getValue();
    }

    @Override
    protected void setTargetShootingVelocity(double targetVelocityRevolutionsPerSecond) {
        shootingMotor.setControl(shootingVelocityRequest.withVelocity(targetVelocityRevolutionsPerSecond));
    }

    @Override
    protected void setTargetFeedingMotorVoltage(double voltage) {
        feedingMotor.setControl(feedingVoltageRequest.withOutput(voltage));
    }

    @Override
    protected void stop() {
        shootingMotor.stopMotor();
        feedingMotor.stopMotor();
    }

    private void updateStatusSignals() {
        BaseStatusSignal.refreshAll(
                PLACEHOLDERShooterConstants.SHOOTING_MOTOR_VELOCITY_SIGNAL,
                PLACEHOLDERShooterConstants.FEEDING_MOTOR_VOLTAGE_SIGNAL
        );
    }
}
