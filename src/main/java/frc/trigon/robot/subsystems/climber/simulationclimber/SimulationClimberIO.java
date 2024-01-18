package frc.trigon.robot.subsystems.climber.simulationclimber;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import frc.trigon.robot.simulation.ElevatorSimulation;
import frc.trigon.robot.subsystems.climber.ClimberIO;
import frc.trigon.robot.subsystems.climber.ClimberInputsAutoLogged;

public class SimulationClimberIO extends ClimberIO {
    private final ElevatorSimulation
            rightMotor = SimulationClimberConstants.RIGHT_MOTOR,
            leftMotor = SimulationClimberConstants.LEFT_MOTOR;
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);

    @Override
    protected void updateInputs(ClimberInputsAutoLogged inputs) {
        inputs.rightMotorPositionMeters = rightMotor.getPosition();
        inputs.rightMotorVelocityMetersPerSecond = rightMotor.getVelocity();
        inputs.rightMotorProfiledSetpointMeters = rightMotor.getProfiledSetpoint();
        inputs.rightMotorVoltage = rightMotor.getVoltage();
        inputs.rightMotorCurrent = rightMotor.getCurrent();

        inputs.leftMotorPositionMeters = leftMotor.getPosition();
        inputs.leftMotorVelocityMetersPerSecond = leftMotor.getVelocity();
        inputs.leftMotorProfiledSetpointMeters = leftMotor.getProfiledSetpoint();
        inputs.leftMotorVoltage = leftMotor.getVoltage();
        inputs.leftMotorCurrent = leftMotor.getCurrent();
    }

    @Override
    protected void setTargetPosition(double averagePositionMeters) {
        rightMotor.setControl(positionRequest.withPosition(averagePositionMeters));
        leftMotor.setControl(positionRequest.withPosition(averagePositionMeters));
    }

    @Override
    protected void stop() {
        rightMotor.stop();
        leftMotor.stop();
    }
}
