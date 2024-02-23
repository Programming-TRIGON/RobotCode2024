package frc.trigon.robot.subsystems.roller.triumphroller;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.trigon.robot.subsystems.roller.RollerIO;
import frc.trigon.robot.subsystems.roller.RollerInputsAutoLogged;

public class TriumphRollerIO extends RollerIO {
    private final TalonFX motor = TriumphRollerConstants.MOTOR;
    private final VoltageOut voltageRequest = new VoltageOut(0);

    @Override
    protected void updateInputs(RollerInputsAutoLogged inputs) {
        refreshStatusSignals();
        inputs.motorVoltage = TriumphRollerConstants.VOLTAGE_SIGNAL.getValue();
        inputs.motorCurrent = TriumphRollerConstants.CURRENT_SIGNAL.getValue();
        inputs.motorVelocityRevolutionsPerSecond = TriumphRollerConstants.VELOCITY_SIGNAL.getValue();
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
                TriumphRollerConstants.VOLTAGE_SIGNAL,
                TriumphRollerConstants.CURRENT_SIGNAL,
                TriumphRollerConstants.VELOCITY_SIGNAL
        );
    }
}
