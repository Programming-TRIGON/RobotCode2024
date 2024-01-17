package frc.trigon.robot.subsystems.shooter.placeholdershooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.trigon.robot.subsystems.shooter.ShooterIO;
import frc.trigon.robot.subsystems.shooter.ShooterInputsAutoLogged;

public class PLACEHOLDERShooterIO extends ShooterIO {
    private final TalonFX
            topMotor = PLACEHOLDERShooterConstants.TOP_SHOOTING_MOTOR,
            bottomMotor = PLACEHOLDERShooterConstants.BOTTOM_SHOOTING_MOTOR;
    private final VelocityVoltage
            topVelocityRequest = new VelocityVoltage(0).withEnableFOC(PLACEHOLDERShooterConstants.FOC_ENABLED),
            bottomVelocityRequest = new VelocityVoltage(0).withEnableFOC(PLACEHOLDERShooterConstants.FOC_ENABLED);
    private final VoltageOut
            topVoltageRequest = new VoltageOut(0).withEnableFOC(PLACEHOLDERShooterConstants.FOC_ENABLED),
            bottomVoltageRequest = new VoltageOut(0).withEnableFOC(PLACEHOLDERShooterConstants.FOC_ENABLED);

    @Override
    protected void updateInputs(ShooterInputsAutoLogged inputs) {
        refreshStatusSignals();

        inputs.topPositionRevolutions = PLACEHOLDERShooterConstants.TOP_MOTOR_POSITION_SIGNAL.getValue();
        inputs.topVelocityRevolutionsPerSecond = PLACEHOLDERShooterConstants.TOP_MOTOR_VELOCITY_SIGNAL.getValue();
        inputs.topVoltage = PLACEHOLDERShooterConstants.TOP_MOTOR_VOLTAGE_SIGNAL.getValue();

        inputs.bottomPositionRevolutions = PLACEHOLDERShooterConstants.BOTTOM_MOTOR_POSITION_SIGNAL.getValue();
        inputs.bottomVelocityRevolutionsPerSecond = PLACEHOLDERShooterConstants.BOTTOM_MOTOR_VELOCITY_SIGNAL.getValue();
        inputs.bottomVoltage = PLACEHOLDERShooterConstants.BOTTOM_MOTOR_VOLTAGE_SIGNAL.getValue();
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
    protected void setTargetTopVoltage(double targetVoltage) {
        topMotor.setControl(topVoltageRequest.withOutput(targetVoltage));
    }

    @Override
    protected void setTargetBottomVoltage(double targetVoltage) {
        bottomMotor.setControl(bottomVoltageRequest.withOutput(targetVoltage));
    }

    @Override
    protected void stop() {
        bottomMotor.stopMotor();
        topMotor.stopMotor();
    }

    private void refreshStatusSignals() {
        BaseStatusSignal.refreshAll(
                PLACEHOLDERShooterConstants.TOP_MOTOR_VELOCITY_SIGNAL,
                PLACEHOLDERShooterConstants.TOP_MOTOR_POSITION_SIGNAL,
                PLACEHOLDERShooterConstants.TOP_MOTOR_VOLTAGE_SIGNAL,
                PLACEHOLDERShooterConstants.BOTTOM_MOTOR_VELOCITY_SIGNAL,
                PLACEHOLDERShooterConstants.BOTTOM_MOTOR_POSITION_SIGNAL,
                PLACEHOLDERShooterConstants.BOTTOM_MOTOR_VOLTAGE_SIGNAL
        );
    }
}
