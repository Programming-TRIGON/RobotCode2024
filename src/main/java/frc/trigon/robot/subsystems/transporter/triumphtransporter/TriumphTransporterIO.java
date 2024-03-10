package frc.trigon.robot.subsystems.transporter.triumphtransporter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.trigon.robot.subsystems.transporter.TransporterIO;
import frc.trigon.robot.subsystems.transporter.TransporterInputsAutoLogged;

public class TriumphTransporterIO extends TransporterIO {
    private final TalonFX motor = TriumphTransporterConstants.MOTOR;
    private final VoltageOut voltageRequest = new VoltageOut(0);

    @Override
    protected void updateInputs(TransporterInputsAutoLogged inputs) {
        refreshStatusSignals();
        inputs.motorVoltage = TriumphTransporterConstants.VOLTAGE_SIGNAL.getValue();
        inputs.motorCurrent = TriumphTransporterConstants.CURRENT_SIGNAL.getValue();
        inputs.motorVelocityRevolutionsPerSecond = TriumphTransporterConstants.VELOCITY_SIGNAL.getValue();
        inputs.sensorTriggered = !TriumphTransporterConstants.BEAM_BREAK.get();
    }

    @Override
    protected void setTargetVoltage(double targetVoltage) {
        motor.setControl(voltageRequest.withOutput(targetVoltage));
    }

    @Override
    protected void stopMotor() {
        motor.stopMotor();
    }

    private void refreshStatusSignals() {
        BaseStatusSignal.refreshAll(
                TriumphTransporterConstants.VOLTAGE_SIGNAL,
                TriumphTransporterConstants.CURRENT_SIGNAL,
                TriumphTransporterConstants.VELOCITY_SIGNAL
        );
    }
}
