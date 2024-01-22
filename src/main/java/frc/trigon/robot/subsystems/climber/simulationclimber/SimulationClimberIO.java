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
        inputs.rightMotorPositionMeters = Conversions.revolutionsToDistance(rightMotor.getPosition(), ClimberConstants.DIAMETER_METERS);
        inputs.rightMotorVelocityMetersPerSecond = Conversions.revolutionsToDistance(rightMotor.getVelocity(), ClimberConstants.DIAMETER_METERS);
        inputs.rightMotorProfiledSetpointMeters = Conversions.revolutionsToDistance(rightMotor.getProfiledSetpoint(), ClimberConstants.DIAMETER_METERS);
        inputs.rightMotorVoltage = rightMotor.getVoltage();
        inputs.rightMotorCurrent = rightMotor.getCurrent();

        inputs.leftMotorPositionMeters = Conversions.revolutionsToDistance(leftMotor.getPosition(), ClimberConstants.DIAMETER_METERS);
        inputs.leftMotorVelocityMetersPerSecond = Conversions.revolutionsToDistance(leftMotor.getVelocity(), ClimberConstants.DIAMETER_METERS);
        inputs.leftMotorProfiledSetpointMeters = Conversions.revolutionsToDistance(leftMotor.getProfiledSetpoint(), ClimberConstants.DIAMETER_METERS);
        inputs.leftMotorVoltage = leftMotor.getVoltage();
        inputs.leftMotorCurrent = leftMotor.getCurrent();
    }

    @Override
    protected void setPositionMeters(double targetPositionMeters) {
        rightMotor.setControl(rightMotorPositionRequest.withPosition(Conversions.distanceToRevolutions(targetPositionMeters, ClimberConstants.DIAMETER_METERS)));
        leftMotor.setControl(leftMotorPositionRequest.withPosition(Conversions.distanceToRevolutions(targetPositionMeters, ClimberConstants.DIAMETER_METERS)));
    }

    @Override
    protected void stop() {
        rightMotor.stop();
        leftMotor.stop();
    }
}
