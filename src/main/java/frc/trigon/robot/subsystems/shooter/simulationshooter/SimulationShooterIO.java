package frc.trigon.robot.subsystems.shooter.simulationshooter;

import com.ctre.phoenix6.controls.VelocityVoltage;
import frc.trigon.robot.simulation.SimpleMotorSimulation;
import frc.trigon.robot.subsystems.shooter.ShooterIO;
import frc.trigon.robot.subsystems.shooter.ShooterInputsAutoLogged;

public class SimulationShooterIO extends ShooterIO {
    private final SimpleMotorSimulation
            topMotor = SimulationShooterConstants.TOP_MOTOR,
            bottomMotor = SimulationShooterConstants.BOTTOM_MOTOR;
    private final VelocityVoltage
            bottomVelocityRequest = new VelocityVoltage(0),
            bottomVoltageRequest = new VelocityVoltage(0);

    @Override
    protected void updateInputs(ShooterInputsAutoLogged inputs) {
        inputs.topPositionRevolutions = topMotor.getPosition();
        inputs.topVelocityRevolutionsPerSecond = topMotor.getVelocity();
        inputs.bottomPositionRevolutions = bottomMotor.getPosition();

        inputs.bottomPositionRevolutions = bottomMotor.getPosition();
        inputs.bottomVelocityRevolutionsPerSecond = bottomMotor.getVelocity();
        inputs.bottomVoltage = bottomMotor.getVoltage();
    }

    @Override
    protected void setTargetTopVelocity(double targetVelocityRevolutionsPerSecond) {
        topMotor.setControl(bottomVelocityRequest.withVelocity(targetVelocityRevolutionsPerSecond));
    }

    @Override
    protected void setTargetBottomVelocity(double targetVelocityRevolutionsPerSecond) {
        bottomMotor.setControl(bottomVoltageRequest.withVelocity(targetVelocityRevolutionsPerSecond));
    }

    @Override
    protected void stop() {
        topMotor.stop();
        bottomMotor.stop();
    }
}
