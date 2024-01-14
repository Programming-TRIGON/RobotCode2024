package frc.trigon.robot.subsystems.roller.simulationroller;

import com.ctre.phoenix6.controls.VelocityVoltage;
import frc.trigon.robot.motorsimulation.SimpleMotorSimulation;
import frc.trigon.robot.subsystems.roller.RollerConstants;
import frc.trigon.robot.subsystems.roller.RollerIO;
import frc.trigon.robot.subsystems.roller.RollerInputsAutoLogged;

public class SimulationRollerIO extends RollerIO {
    private final SimpleMotorSimulation motorSim = SimulationRollerConstants.MOTOR_SIMULATION;
    private final VelocityVoltage currentRequest = new VelocityVoltage(0);

    @Override
    protected void updateInputs(RollerInputsAutoLogged inputs) {
        inputs.motorVoltage = motorSim.getVoltage();
        inputs.motorCurrent = motorSim.getCurrent();

        inputs.infraredSensorTriggered = false;
    }

    @Override
    protected void setTargetVelocityState(double velocity) {
        motorSim.setControl(currentRequest.withVelocity(velocity));
    }

    @Override
    protected void stopMotor() {
        motorSim.stop();
    }
}
