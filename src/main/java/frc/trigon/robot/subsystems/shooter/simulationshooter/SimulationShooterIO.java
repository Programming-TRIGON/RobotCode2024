package frc.trigon.robot.subsystems.shooter.simulationshooter;

import com.ctre.phoenix6.controls.VoltageOut;
import frc.trigon.robot.simulation.FlywheelSimulation;
import frc.trigon.robot.subsystems.shooter.ShooterIO;
import frc.trigon.robot.subsystems.shooter.ShooterInputsAutoLogged;

public class SimulationShooterIO extends ShooterIO {
    private final FlywheelSimulation motor = SimulationShooterConstants.MOTOR;
    private final VoltageOut voltageRequest = new VoltageOut(0);

    @Override
    protected void updateInputs(ShooterInputsAutoLogged inputs) {
        inputs.positionRevolutions = motor.getPositionRevolutions();
        inputs.velocityRevolutionsPerSecond = motor.getVelocityRevolutionsPerSecond();
        inputs.voltage = motor.getVoltage();
        inputs.current = motor.getCurrent();
    }

    @Override
    protected void setTargetVoltage(double targetVoltage) {
        motor.setControl(voltageRequest.withOutput(targetVoltage));
    }

    @Override
    protected void stop() {
        motor.stop();
    }
}
