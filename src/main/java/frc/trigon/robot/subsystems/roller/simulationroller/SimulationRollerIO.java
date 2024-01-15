package frc.trigon.robot.subsystems.roller.simulationroller;

import com.ctre.phoenix6.controls.VelocityVoltage;
import frc.trigon.robot.motorsimulation.SimpleMotorSimulation;
import frc.trigon.robot.subsystems.roller.RollerIO;
import frc.trigon.robot.subsystems.roller.RollerInputsAutoLogged;

public class SimulationRollerIO extends RollerIO {
    private final SimpleMotorSimulation motorSim = SimulationRollerConstants.MOTOR_SIMULATION;
    private final VelocityVoltage voltageRequest = new VelocityVoltage(0);

    @Override
    protected void updateInputs(RollerInputsAutoLogged inputs) {
        inputs.motorVoltage = motorSim.getVoltage();
        inputs.motorCurrent = motorSim.getCurrent();
        inputs.motorVelocity = motorSim.getVelocity();

        inputs.infraredSensorTriggered = false;
    }

    @Override
    protected void setTargetVelocityRotationsPerSecond(double velocity) {
        motorSim.setControl(voltageRequest.withVelocity(velocity));
    }

    @Override
    protected void stopMotor() {
        motorSim.stop();
    }
}
