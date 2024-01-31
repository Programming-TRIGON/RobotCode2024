package frc.trigon.robot.subsystems.shooter.placeholdershooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.trigon.robot.subsystems.shooter.ShooterIO;
import frc.trigon.robot.subsystems.shooter.ShooterInputsAutoLogged;

public class PLACEHOLDERShooterIO extends ShooterIO {
    private final TalonFX masterMotor = PLACEHOLDERShooterConstants.MASTER_MOTOR;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(PLACEHOLDERShooterConstants.FOC_ENABLED);

    @Override
    protected void updateInputs(ShooterInputsAutoLogged inputs) {
        refreshStatusSignals();

        inputs.positionRevolutions = PLACEHOLDERShooterConstants.MASTER_MOTOR_POSITION_SIGNAL.getValue();
        inputs.velocityRevolutionsPerSecond = PLACEHOLDERShooterConstants.MASTER_MOTOR_VELOCITY_SIGNAL.getValue();
        inputs.voltage = PLACEHOLDERShooterConstants.MASTER_MOTOR_VOLTAGE_SIGNAL.getValue();
        inputs.current = PLACEHOLDERShooterConstants.MASTER_MOTOR_CURRENT_SIGNAL.getValue();
    }

    @Override
    protected void setTargetVoltage(double targetVoltage) {
        masterMotor.setControl(voltageRequest.withOutput(targetVoltage));
    }

    @Override
    protected void stop() {
        masterMotor.stopMotor();
    }

    private void refreshStatusSignals() {
        BaseStatusSignal.refreshAll(
                PLACEHOLDERShooterConstants.MASTER_MOTOR_VELOCITY_SIGNAL,
                PLACEHOLDERShooterConstants.MASTER_MOTOR_POSITION_SIGNAL,
                PLACEHOLDERShooterConstants.MASTER_MOTOR_VOLTAGE_SIGNAL
        );
    }
}
