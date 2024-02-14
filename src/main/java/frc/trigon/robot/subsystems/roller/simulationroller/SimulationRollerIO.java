package frc.trigon.robot.subsystems.roller.simulationroller;

import com.ctre.phoenix6.controls.VoltageOut;
import frc.trigon.robot.simulation.FlywheelSimulation;
import frc.trigon.robot.subsystems.roller.RollerIO;
import frc.trigon.robot.subsystems.roller.RollerInputsAutoLogged;

public class SimulationRollerIO extends RollerIO {
    private final FlywheelSimulation motor = SimulationRollerConstants.MOTOR;
    private final VoltageOut voltageRequest = new VoltageOut(0);

    @Override
    protected void updateInputs(RollerInputsAutoLogged inputs) {
        inputs.motorVoltage = motor.getVoltage();
        inputs.motorCurrent = motor.getCurrent();
        inputs.motorVelocityRevolutionsPerSecond = motor.getVelocityRevolutionsPerSecond();

        inputs.noteDetectedBySensor = false;
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
