package frc.trigon.robot.subsystems.shooter.simulationshooter;

import com.ctre.phoenix6.controls.VelocityVoltage;
import frc.trigon.robot.simulation.FlywheelSimulation;
import frc.trigon.robot.subsystems.shooter.ShooterIO;
import frc.trigon.robot.subsystems.shooter.ShooterInputsAutoLogged;

public class SimulationShooterIO extends ShooterIO {
    private final FlywheelSimulation motor = SimulationShooterConstants.MOTOR;
    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);

    @Override
    protected void updateInputs(ShooterInputsAutoLogged inputs) {
        inputs.positionRevolutions = motor.getPositionRevolutions();
        inputs.velocityRevolutionsPerSecond = motor.getVelocityRevolutionsPerSecond();
        inputs.voltage = motor.getVoltage();
        inputs.current = motor.getCurrent();
    }

    @Override
    protected void setTargetVelocity(double targetVelocity) {
        motor.setControl(velocityVoltage.withVelocity(targetVelocity));
    }

    @Override
    protected void stop() {
        motor.stop();
    }
}
