package frc.trigon.robot.subsystems.roller.simulationroller;

import com.ctre.phoenix6.controls.VelocityVoltage;
import frc.trigon.robot.simulation.SimpleMotorSimulation;
import frc.trigon.robot.subsystems.roller.RollerIO;
import frc.trigon.robot.subsystems.roller.RollerInputsAutoLogged;

public class SimulationRollerIO extends RollerIO {
    private final SimpleMotorSimulation motor = SimulationRollerConstants.MOTOR;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withEnableFOC(SimulationRollerConstants.FOC_ENABLED);

    @Override
    protected void updateInputs(RollerInputsAutoLogged inputs) {
        inputs.motorVoltage = motor.getVoltage();
        inputs.motorCurrent = motor.getCurrent();
        inputs.motorVelocityRevolutionsPerSecond = motor.getVelocity();

        inputs.infraredSensorTriggered = false;
    }

    @Override
    protected void setTargetVelocity(double targetVelocityRevolutionsPerSecond) {
        motor.setControl(velocityRequest.withVelocity(targetVelocityRevolutionsPerSecond));
    }

    @Override
    protected void stopMotor() {
        motor.stop();
    }
}
