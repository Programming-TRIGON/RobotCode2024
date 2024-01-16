package frc.trigon.robot.subsystems.shooter.simulationshooter;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import frc.trigon.robot.simulation.SimpleMotorSimulation;
import frc.trigon.robot.subsystems.shooter.ShooterIO;
import frc.trigon.robot.subsystems.shooter.ShooterInputsAutoLogged;

public class SimulationShooterIO extends ShooterIO {
    private final SimpleMotorSimulation
            shootingMotor = SimulationShooterConstants.SHOOTING_MOTOR,
            feedingMotor = SimulationShooterConstants.FEEDING_MOTOR;
    private final VelocityVoltage shootingVelocityRequest = new VelocityVoltage(0);
    private final VoltageOut feedingVoltageRequest = new VoltageOut(0);

    @Override
    protected void updateInputs(ShooterInputsAutoLogged inputs) {
        inputs.bottomVelocityRevolutionsPerSecond = shootingMotor.getVelocity();
        inputs.topVelocityRevolutionsPerSecond = feedingMotor.getVoltage();
    }

    @Override
    protected void setTargetTopVelocity(double targetVelocityRevolutionsPerSecond) {
        shootingMotor.setControl(shootingVelocityRequest.withVelocity(targetVelocityRevolutionsPerSecond));
    }

    @Override
    protected void setTargetBottomVelocity(double targetVelocityRevolutionsPerSecond) {
        feedingMotor.setControl(feedingVoltageRequest.withOutput(targetVelocityRevolutionsPerSecond));
    }

    @Override
    protected void stop() {
        shootingMotor.stop();
        feedingMotor.stop();
    }
}
