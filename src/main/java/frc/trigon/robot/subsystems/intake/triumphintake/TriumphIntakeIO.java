package frc.trigon.robot.subsystems.intake.triumphintake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.trigon.robot.subsystems.intake.IntakeIO;
import frc.trigon.robot.subsystems.intake.IntakeInputsAutoLogged;

public class TriumphIntakeIO extends IntakeIO {
    private final TalonFX motor = TriumphIntakeConstants.MOTOR;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(TriumphIntakeConstants.FOC_ENABLED);

    @Override
    protected void updateInputs(IntakeInputsAutoLogged inputs) {
        refreshStatusSignals();

        inputs.velocityRevolutionsPerSecond = TriumphIntakeConstants.COLLECTION_MOTOR_VELOCITY_SIGNAL.getValue();
        inputs.voltage = TriumphIntakeConstants.COLLECTION_MOTOR_VOLTAGE_SIGNAL.getValue();
        inputs.current = TriumphIntakeConstants.COLLECTION_MOTOR_CURRENT_SIGNAL.getValue();
    }

    @Override
    protected void setTargetVoltage(double voltage) {
        motor.setControl(voltageRequest.withOutput(voltage));
    }

    @Override
    protected void stop() {
        motor.stopMotor();
    }

    private void refreshStatusSignals() {
        BaseStatusSignal.refreshAll(
                TriumphIntakeConstants.COLLECTION_MOTOR_VELOCITY_SIGNAL,
                TriumphIntakeConstants.COLLECTION_MOTOR_CURRENT_SIGNAL,
                TriumphIntakeConstants.COLLECTION_MOTOR_VOLTAGE_SIGNAL
        );
    }
}
