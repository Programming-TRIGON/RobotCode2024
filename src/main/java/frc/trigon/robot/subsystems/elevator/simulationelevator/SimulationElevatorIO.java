package frc.trigon.robot.subsystems.elevator.simulationelevator;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import frc.trigon.robot.simulation.ElevatorSimulation;
import frc.trigon.robot.subsystems.elevator.ElevatorConstants;
import frc.trigon.robot.subsystems.elevator.ElevatorIO;
import frc.trigon.robot.subsystems.elevator.ElevatorInputsAutoLogged;
import frc.trigon.robot.utilities.Conversions;

public class SimulationElevatorIO extends ElevatorIO {
    private final ElevatorSimulation motor = SimulationElevatorConstants.MOTOR;
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);

    @Override
    protected void updateInputs(ElevatorInputsAutoLogged inputs) {
        inputs.motorVoltage = motor.getVoltage();
        inputs.positionMeters = getMotorPositionMeters();
        inputs.velocityMetersPerSecond = getMotorVelocityMetersPerSecond();
        inputs.profiledSetpointMeters = getProfiledSetpointMeters();
    }

    @Override
    protected void setTargetPosition(double targetPositionMeters) {
        motor.setControl(positionRequest.withPosition(calculateMotorVoltage(targetPositionMeters)));

    }

    @Override
    protected void stop() {
        motor.stop();
    }

    private double calculateMotorVoltage(double targetPositionMeters) {
        final double pidOutput = SimulationElevatorConstants.PROFILED_PID_CONTROLLER.calculate(motor.getPosition(), targetPositionMeters);
        final double feedforward = SimulationElevatorConstants.FEEDFORWARD.calculate(SimulationElevatorConstants.PROFILED_PID_CONTROLLER.getGoal().velocity);
        return pidOutput + feedforward;
    }

    private double getMotorPositionMeters() {
        return Conversions.revolutionsToDistance(motor.getPosition(), getWheelDiameter());
    }

    private double getMotorVelocityMetersPerSecond() {
        return Conversions.revolutionsToDistance(motor.getVelocity(), getWheelDiameter());
    }

    private double getWheelDiameter() {
        return ElevatorConstants.DRUM_RADIUS_METERS * 2;
    }

    private double getProfiledSetpointMeters() {
        return Conversions.revolutionsToDistance(SimulationElevatorConstants.PROFILED_PID_CONTROLLER.getSetpoint().position, getWheelDiameter());
    }
}
