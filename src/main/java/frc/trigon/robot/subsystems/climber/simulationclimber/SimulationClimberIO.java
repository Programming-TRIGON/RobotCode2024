package frc.trigon.robot.subsystems.climber.simulationclimber;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import frc.trigon.robot.simulation.ElevatorSimulation;
import frc.trigon.robot.subsystems.climber.ClimberConstants;
import frc.trigon.robot.subsystems.climber.ClimberIO;
import frc.trigon.robot.subsystems.climber.ClimberInputsAutoLogged;
import frc.trigon.robot.utilities.Conversions;

public class SimulationClimberIO extends ClimberIO {
    private final ElevatorSimulation
            rightMotor = SimulationClimberConstants.RIGHT_MOTOR,
            leftMotor = SimulationClimberConstants.LEFT_MOTOR;
    private final MotionMagicVoltage
            rightMotorPositionRequest = new MotionMagicVoltage(0),
            leftMotorPositionRequest = new MotionMagicVoltage(0);

    @Override
    protected void updateInputs(ClimberInputsAutoLogged inputs) {
        inputs.rightMotorPositionMeters = Conversions.revolutionsToDistance(rightMotor.getPosition(), ClimberConstants.DRUM_RADIUS);
        inputs.rightMotorVelocityMetersPerSecond = Conversions.revolutionsToDistance(rightMotor.getVelocity(), ClimberConstants.DRUM_RADIUS);
        inputs.rightMotorProfiledSetpointMeters = Conversions.revolutionsToDistance(rightMotor.getProfiledSetpoint(), ClimberConstants.DRUM_RADIUS);
        inputs.rightMotorVoltage = rightMotor.getVoltage();
        inputs.rightMotorCurrent = rightMotor.getCurrent();

        inputs.leftMotorPositionMeters = Conversions.revolutionsToDistance(leftMotor.getPosition(), ClimberConstants.DRUM_RADIUS);
        inputs.leftMotorVelocityMetersPerSecond = Conversions.revolutionsToDistance(leftMotor.getVelocity(), ClimberConstants.DRUM_RADIUS);
        inputs.leftMotorProfiledSetpointMeters = Conversions.revolutionsToDistance(leftMotor.getProfiledSetpoint(), ClimberConstants.DRUM_RADIUS);
        inputs.leftMotorVoltage = leftMotor.getVoltage();
        inputs.leftMotorCurrent = leftMotor.getCurrent();
    }

    @Override
    protected void setTargetPosition(double averagePositionMeters) {
        rightMotor.setControl(rightMotorPositionRequest.withPosition(averagePositionMeters));
        leftMotor.setControl(leftMotorPositionRequest.withPosition(averagePositionMeters));
    }

    @Override
    protected void stop() {
        rightMotor.stop();
        leftMotor.stop();
    }
}
