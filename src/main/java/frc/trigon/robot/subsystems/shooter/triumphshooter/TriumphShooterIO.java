package frc.trigon.robot.subsystems.shooter.triumphshooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.trigon.robot.subsystems.shooter.ShooterIO;
import frc.trigon.robot.subsystems.shooter.ShooterInputsAutoLogged;

public class TriumphShooterIO extends ShooterIO {
    private final TalonFX masterMotor = TriumphShooterConstants.MASTER_MOTOR;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(TriumphShooterConstants.FOC_ENABLED);

    @Override
    protected void updateInputs(ShooterInputsAutoLogged inputs) {
        refreshStatusSignals();

        inputs.positionRevolutions = TriumphShooterConstants.MASTER_MOTOR_POSITION_SIGNAL.getValue();
        inputs.velocityRevolutionsPerSecond = TriumphShooterConstants.MASTER_MOTOR_VELOCITY_SIGNAL.getValue();
        inputs.voltage = TriumphShooterConstants.MASTER_MOTOR_VOLTAGE_SIGNAL.getValue();
        inputs.current = TriumphShooterConstants.MASTER_MOTOR_CURRENT_SIGNAL.getValue();
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
                TriumphShooterConstants.MASTER_MOTOR_VELOCITY_SIGNAL,
                TriumphShooterConstants.MASTER_MOTOR_POSITION_SIGNAL,
                TriumphShooterConstants.MASTER_MOTOR_VOLTAGE_SIGNAL
        );
    }
}
