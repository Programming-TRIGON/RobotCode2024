package frc.trigon.robot.subsystems.roller.triumphroller;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Ultrasonic;
import frc.trigon.robot.subsystems.roller.RollerIO;
import frc.trigon.robot.subsystems.roller.RollerInputsAutoLogged;

public class TriumphRollerIO extends RollerIO {
    private final TalonFX motor = TriumphRollerConstants.MOTOR;
    private final Ultrasonic ultrasonicSensor = TriumphRollerConstants.ULTRASONIC_SENSOR;
    private final VoltageOut voltageRequest = new VoltageOut(0);

    @Override
    protected void updateInputs(RollerInputsAutoLogged inputs) {
        refreshStatusSignals();
        inputs.motorVoltage = TriumphRollerConstants.VOLTAGE_STATUS_SIGNAL.getValue();
        inputs.motorCurrent = TriumphRollerConstants.CURRENT_STATUS_SIGNAL.getValue();
        inputs.motorVelocityRevolutionsPerSecond = TriumphRollerConstants.VELOCITY_STATUS_SIGNAL.getValue();

        inputs.noteDetectedBySensor = noteDetectedBySensor();
    }

    @Override
    protected void setTargetVoltage(double targetVoltage) {
        motor.setControl(voltageRequest.withOutput(targetVoltage));
    }

    @Override
    protected void stopMotor() {
        motor.stopMotor();
    }

    private boolean noteDetectedBySensor() {
        return ultrasonicSensor.getRangeMM() < TriumphRollerConstants.MAXIMUM_ULTRASONIC_DISTANCE_MILLIMETERS;
    }

    private void refreshStatusSignals() {
        BaseStatusSignal.refreshAll(
                TriumphRollerConstants.VOLTAGE_STATUS_SIGNAL,
                TriumphRollerConstants.CURRENT_STATUS_SIGNAL,
                TriumphRollerConstants.VELOCITY_STATUS_SIGNAL
        );
    }
}
