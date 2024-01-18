package frc.trigon.robot.subsystems.shooter.simulationshooter;

import com.ctre.phoenix6.controls.VoltageOut;
import frc.trigon.robot.simulation.FlywheelSimulation;
import frc.trigon.robot.subsystems.shooter.ShooterIO;
import frc.trigon.robot.subsystems.shooter.ShooterInputsAutoLogged;

public class SimulationShooterIO extends ShooterIO {
    private final FlywheelSimulation
            topMotor = SimulationShooterConstants.TOP_MOTOR,
            bottomMotor = SimulationShooterConstants.BOTTOM_MOTOR;
    private final VoltageOut
            topVoltageRequest = new VoltageOut(0),
            bottomVoltageRequest = new VoltageOut(0);

    @Override
    protected void updateInputs(ShooterInputsAutoLogged inputs) {
        inputs.topPositionRevolutions = topMotor.getPosition();
        inputs.topVelocityRevolutionsPerSecond = topMotor.getVelocity();
        inputs.topVoltage = topMotor.getVoltage();

        inputs.bottomPositionRevolutions = bottomMotor.getPosition();
        inputs.bottomVelocityRevolutionsPerSecond = bottomMotor.getVelocity();
        inputs.bottomVoltage = bottomMotor.getVoltage();
    }

    @Override
    protected void setTargetTopVoltage(double targetVoltage) {
        topMotor.setControl(topVoltageRequest.withOutput(targetVoltage));
    }

    @Override
    protected void setTargetBottomVoltage(double targetVoltage) {
        bottomMotor.setControl(bottomVoltageRequest.withOutput(targetVoltage));
    }

    @Override
    protected void stop() {
        topMotor.stop();
        bottomMotor.stop();
    }
}
