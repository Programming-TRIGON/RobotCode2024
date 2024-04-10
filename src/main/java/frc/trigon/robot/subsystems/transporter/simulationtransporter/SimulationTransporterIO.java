package frc.trigon.robot.subsystems.transporter.simulationtransporter;

import com.ctre.phoenix6.controls.VoltageOut;
import frc.trigon.robot.simulation.FlywheelSimulation;
import frc.trigon.robot.subsystems.transporter.TransporterIO;
import frc.trigon.robot.subsystems.transporter.TransporterInputsAutoLogged;

public class SimulationTransporterIO extends TransporterIO {
    private final FlywheelSimulation motor = SimulationTransporterConstants.MOTOR;
    private final VoltageOut voltageRequest = new VoltageOut(0);

    @Override
    protected void updateInputs(TransporterInputsAutoLogged inputs) {
        inputs.motorVoltage = motor.getVoltage();
        inputs.motorCurrent = motor.getCurrent();
        inputs.motorVelocityRevolutionsPerSecond = motor.getVelocityRevolutionsPerSecond();
    }

    @Override
    protected void setTargetVoltage(double targetVoltage) {
        motor.setControl(voltageRequest.withOutput(targetVoltage));
    }

    @Override
    protected void stopMotor() {
        motor.stop();
    }
}
