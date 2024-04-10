package frc.trigon.robot.subsystems.intake.simulationintake;

import com.ctre.phoenix6.controls.VoltageOut;
import frc.trigon.robot.simulation.FlywheelSimulation;
import frc.trigon.robot.subsystems.intake.IntakeIO;
import frc.trigon.robot.subsystems.intake.IntakeInputsAutoLogged;

public class SimulationIntakeIO extends IntakeIO {
    private final FlywheelSimulation motor = SimulationIntakeConstants.MOTOR;
    private final VoltageOut voltageRequest = new VoltageOut(0);

    @Override
    protected void updateInputs(IntakeInputsAutoLogged inputs) {
        inputs.velocityRevolutionsPerSecond = motor.getVelocityRevolutionsPerSecond();
        inputs.voltage = motor.getVoltage();
        inputs.current = motor.getCurrent();
    }

    @Override
    protected void setTargetVoltage(double voltage) {
        motor.setControl(voltageRequest.withOutput(voltage));
    }

    @Override
    protected void stop() {
        motor.stop();
    }
}
